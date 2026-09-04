# vad_mode.py  （基于你原来的文件，添加融合功能）
# -- coding: utf-8 --
import os, time, base64, asyncio
import cv2
import pyaudio
import queue
import threading
import json
import socket
import websockets
import sys
import subprocess
import numpy as np
import webrtcvad

from omni_realtime_client import OmniRealtimeClient, TurnDetectionMode

# ================= 融合新增：系统提示词 + UDP 通信 + 文本解析 =================
sys.stdout.reconfigure(line_buffering=True)
SYSTEM_PROMPT = """
你是一个智能辅助导航助手，用户是盲人，你通过语音与其交互。你必须用中文回复。
你同时接收用户的语音和摄像头图像。
当用户询问周围环境（如“前面有什么”、“描述一下”、“看”等）时，请直接描述你看到的场景，重点说明前方是否有障碍物、人、门、楼梯等，语气亲切。
其他对话请正常回答，不要输出任何特殊标记。
注意：你的回复将通过语音播放，所以不要包含任何不可读的符号或标签。
"""

SYSTEM_PROMPT += """
当你收到以“导航系统通知：”开头的用户消息时，这是一个自动系统通知，不是用户说的话。
请直接根据通知内容，用自然、亲切的语气告知用户当前发生了什么，不要添加任何标记（如[NAV_A]等）。
回复要简短，通常一句话即可。
"""

# 导航事件码 -> 给 AI 的提示文本（AI 会据此用自然语言告知用户）
NAV_EVENT_PROMPTS = {
    "ARRIVED":        "导航系统通知：您已经到达目的地，请告知用户“已到达目的地”。",
    "BLOCKED":        "导航系统通知：前方暂时无法通过，请告知用户“耐心等待，不要移动“。",
    "PATH_CLEAR":     "导航系统通知：路径已经畅通，请告知用户可以继续前进。",
    "PLAN_FAILED":    "导航系统通知：路径规划失败，前方可能不通，请告知用户无法继续导航。",
    "NAV_STARTED":    "导航系统通知：导航已经开始，请告知用户跟随振动指引前行。",
    "NAV_STOPPED":    "导航系统通知：导航已停止，请告知用户。",
    "OFF_TRACK_STOP": "导航系统通知：用户偏离路线过多，不用告知用户",
}

client = None


# ---- 与图像缓存相关 ----
latest_image = None            # 最新一帧 JPEG 字节
latest_image_lock = threading.Lock()
scene_requested_event = asyncio.Event()   # 当需要场景描述时设置为 True

nav_paused = False
nav_event_queue = queue.Queue()   # 线程安全队列，用于向异步主循环传递导航事件

def send_udp_cmd(cmd: str):
    """向导航模块发送指令（127.0.0.1:5000）"""
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.sendto(cmd.encode(), ('127.0.0.1', 5000))
        sock.close()
    except Exception as e:
        print(f"[UDP] send error: {e}")

def udp_listener():
    """监听导航模块状态（127.0.0.1:5001），将事件码放入队列"""
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(('127.0.0.1', 5001))
    print("[UDP] Perception listener on port 5001")
    while True:
        try:
            data, _ = sock.recvfrom(1024)
            cmd = data.decode().strip()
            print(f"[UDP] Received: {cmd}")
            # 如果是导航事件码，放入队列供主循环处理
            if cmd in NAV_EVENT_PROMPTS:
                nav_event_queue.put(cmd)
            # 其他状态码（如 MUTE_NAV 等）可在这里处理，暂时忽略
        except Exception as e:
            print(f"[UDP] recv error: {e}")

response_text = ""                  # 累积 AI 文本

def on_text_delta(text: str):
    print("\n🔵 DEBUG: on_text_delta called")
    global response_text
    response_text += text
    print(text, end="", flush=True)  # 实时显示

# ================= 音频、麦克风、播放 =================

audio_queue = queue.Queue()
audio_player = None
interrupt_flag = threading.Event()

mic_enabled = asyncio.Event()
mic_enabled.set()
is_playing = False

RATE = 24000
CHUNK = 3200
FORMAT = pyaudio.paInt16
CHANNELS = 1

def clear_audio_queue():
    with audio_queue.mutex:
        audio_queue.queue.clear()

def handle_interrupt():
    print("[Interrupt] 用户语音来临，停止当前播放")
    interrupt_flag.set()
    clear_audio_queue()

def audio_player_thread():
    p = pyaudio.PyAudio()
    try:
        stream = p.open(format=FORMAT, channels=CHANNELS,
                        rate=RATE, output=True,
                        frames_per_buffer=CHUNK)
    except Exception as e:
        print("[AudioPlayer] 无可用输出设备，跳过播放:", e)
        return

    try:
        while True:
            data = audio_queue.get()
            if data is None:
                break
            if interrupt_flag.is_set():
                interrupt_flag.clear()
                continue
            stream.write(data)
            audio_queue.task_done()
    finally:
        stream.stop_stream()
        stream.close()
        p.terminate()

def start_audio_player():
    global audio_player
    if audio_player is None or not audio_player.is_alive():
        audio_player = threading.Thread(target=audio_player_thread, daemon=True)
        audio_player.start()

def handle_audio_data(audio_bytes: bytes):
    global is_playing
    if not is_playing:
        is_playing = True
        mic_enabled.clear()
        send_udp_cmd("MUTE_NAV")           # <--- 新增：通知导航静音
    audio_queue.put(audio_bytes)

def handle_response_done(event):
    global is_playing, response_text, nav_paused
    is_playing = False

    async def restore():
        global nav_paused, response_text
        await asyncio.to_thread(audio_queue.join)
        # 延迟 0.8 秒再开麦克风，避免扬声器尾音/回声被 VAD 误判
        await asyncio.sleep(0.8)
        mic_enabled.set()
        send_udp_cmd("UNMUTE_NAV")
        if nav_paused:
            send_udp_cmd("RESUME_NAV")
            nav_paused = False
        response_text = ""
    loop = asyncio.get_event_loop()
    loop.create_task(restore())

def on_text_done(transcript: str):
    global nav_paused
    print(f"[用户输入转写] {transcript}")

    # 场景/描述意图检测
    scene_keywords = ["描述", "看", "前面有", "环境", "场景", "这是什么", "前面"]
    if any(kw in transcript for kw in scene_keywords):
        if not nav_paused:
            send_udp_cmd("PAUSE_NAV")
            nav_paused = True
        # 触发场景识别
        loop = asyncio.get_event_loop()
        loop.call_soon_threadsafe(scene_requested_event.set)
        return  # 不请求普通回复

    # 导航指令检测
    nav_commands = [
        ("去座位", "GO_A"),
        ("去门口", "GO_B"),
        ("停止导航", "STOP"),
        ("暂停导航", "PAUSE_NAV"),
        ("恢复导航", "RESUME_NAV"),
    ]
    for keyword, cmd in nav_commands:
        if keyword in transcript:
            send_udp_cmd(cmd)
            if cmd == "PAUSE_NAV":
                nav_paused = True
            elif cmd == "RESUME_NAV":
                nav_paused = False
            # 导航指令由导航模块通过事件码触发 AI 播报，这里不请求回复
            return

    # 普通对话：请求 AI 回复（音频已经提交）
    if client:
        asyncio.create_task(client.send_event({"type": "response.create"}))

import webrtcvad

async def scene_handler(client: OmniRealtimeClient):
    """等待 scene_requested_event，然后取图并让 AI 描述场景。"""
    while True:
        await scene_requested_event.wait()
        scene_requested_event.clear()
        # 循环等待，直到当前响应结束（最多等 5 秒）
        timeout = 5
        while client._is_responding and timeout > 0:
            await asyncio.sleep(0.1)
            timeout -= 0.1
        # 取消当前可能正在生成的响应（避免无意义的回复）
        try:
            await client.send_event({"type": "response.cancel"})
            await asyncio.sleep(0.1)
        except Exception:
            pass

        # 获取最新图像
        with latest_image_lock:
            img = latest_image
        if img is None:
            # 如果没有图像可用，用纯文本提示代替
            print("无图像可用，使用纯文本提示")
            await client.trigger_response_with_text("请告诉用户：暂时无法获取图像")
            continue

        # 让 AI 基于图像描述前方场景
        mic_enabled.clear()   # 描述期间暂停麦克风
        await client.trigger_response_with_image_and_text(
            img,
            "请用中文简单描述你看到的场景，重点说明前方是否有障碍物、人、门、楼梯等，语气亲切。"
        )

async def start_microphone_streaming(client: OmniRealtimeClient):
    DEVICE = "pulse"
    DEVICE_RATE = 48000
    TARGET_RATE = 16000   # webrtcvad 只支持 8k/16k/32k，我们重采样到 16k 用于 VAD
    CHUNK_DEVICE = int(DEVICE_RATE * 0.02)   # 20ms 帧
    CHUNK_TARGET = int(TARGET_RATE * 0.02)   # 320 样本

    vad = webrtcvad.Vad(2)   # 灵敏度 0~3，2 适中
    SPEECH_TIMEOUT = 0.5      # 说话停顿 0.5 秒视为结束
    SILENCE_FRAMES = int(SPEECH_TIMEOUT / 0.02)

    speech_frames = []
    silence_count = 0
    is_speaking = False

    cmd = [
        "arecord",
        "-D", DEVICE,
        "-r", str(DEVICE_RATE),
        "-c", "1",
        "-f", "S16_LE",
        "--buffer-size", str(CHUNK_DEVICE * 4),
        "--period-size", str(CHUNK_DEVICE),
        "-t", "raw",
        "-q"
    ]
    proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL)
    print(f"[Mic] 已启动 arecord (Manual VAD)，设备 {DEVICE}")

    try:
        while True:
            await mic_enabled.wait()
            raw = proc.stdout.read(CHUNK_DEVICE * 2)
            if not raw:
                break
            # 如果 AI 正在回复，丢弃这帧音频，防止回声触发新的回复
            if client._is_responding:
                is_speaking = False
                silence_count = 0
                await asyncio.sleep(0.01)
                continue
            # 重采样到 16k 用于 VAD
            audio_np = np.frombuffer(raw, dtype=np.int16)
            target_len = int(len(audio_np) * TARGET_RATE / DEVICE_RATE)
            resampled_16k = np.interp(
                np.linspace(0, len(audio_np) - 1, target_len),
                np.arange(len(audio_np)),
                audio_np
            ).astype(np.int16)

            # 同时准备 24k 数据用于上传
            resampled_24k = np.interp(
                np.linspace(0, len(audio_np) - 1, int(len(audio_np) * 24000 / DEVICE_RATE)),
                np.arange(len(audio_np)),
                audio_np
            ).astype(np.int16)

            # 上传音频到 API
            b64 = base64.b64encode(resampled_24k[:3200].tobytes()).decode("utf-8")
            await client.send_event({
                "type": "input_audio_buffer.append",
                "audio": b64,
            })

            # 本地 VAD 判断
            is_speech = vad.is_speech(resampled_16k[:CHUNK_TARGET].tobytes(), TARGET_RATE)
            if is_speech:
                if not is_speaking:
                    print("[VAD] 语音开始", flush=True)
                is_speaking = True
                silence_count = 0
            else:
                if is_speaking:
                    silence_count += 1
                    if silence_count >= SILENCE_FRAMES:
                        print("[VAD] 语音结束，已提交音频，等待转写", flush=True)
                        mic_enabled.clear()
                        is_speaking = False
                        silence_count = 0
                        # 只提交音频，不请求回复，回复由 on_text_done 根据意图决定
                        await client.commit_audio_buffer()
            await asyncio.sleep(0.01)
    finally:
        proc.kill()
        proc.wait()

async def rosbridge_image_sub(client: OmniRealtimeClient,
                               topic='/zed/zed_node/rgb/color/rect/image/compressed',
                               fps=2.0):
    """订阅压缩图像，只缓存最新一帧，不自动发给 API。"""
    global latest_image
    uri = "ws://localhost:9090"
    interval = 1.0 / fps
    while True:
        try:
            async with websockets.connect(uri) as ws:
                subscribe_msg = {
                    "op": "subscribe",
                    "topic": topic,
                    "type": "sensor_msgs/CompressedImage"
                }
                await ws.send(json.dumps(subscribe_msg))
                print(f"[Rosbridge] Subscribed to {topic}")

                last_save = 0.0
                async for message in ws:
                    data = json.loads(message)
                    if data.get("op") == "publish" and data.get("topic") == topic:
                        now = time.time()
                        if now - last_save < interval:
                            continue
                        last_save = now
                        img_b64 = data.get("msg", {}).get("data", "")
                        if img_b64:
                            # 解码 base64 → 原始 JPEG 字节
                            img_bytes = base64.b64decode(img_b64)
                            with latest_image_lock:
                                latest_image = img_bytes   # 缓存最新帧
                            # 注意：这里不再 append_image，以免 AI 自动描述
        except (websockets.ConnectionClosed, OSError) as e:
            print(f"[Rosbridge] Connection lost: {e}, retrying in 3s...")
            await asyncio.sleep(3)

# ================= 主入口 =================

async def main():
    start_audio_player()

    # 启动 UDP 监听
    threading.Thread(target=udp_listener, daemon=True).start()
    print("[Main] UDP listener started")
    global client
    client = OmniRealtimeClient(
        base_url="wss://dashscope.aliyuncs.com/api-ws/v1/realtime",       
        api_key="sk-8f96bae0f4f94f698a0be72e4cfa5e59",        
        model="qwen3-omni-flash-realtime",
        voice="Cherry",
        turn_detection_mode=TurnDetectionMode.MANUAL,
        on_text_delta=on_text_delta,          # 使用我们自定义的文本解析
        on_audio_delta=handle_audio_data,
        on_interrupt=handle_interrupt,
        on_text_done=on_text_done,
        extra_event_handlers={"response.done": handle_response_done},
        instructions=SYSTEM_PROMPT            # 注入系统提示词
    )

    async def nav_event_processor(client: OmniRealtimeClient):
        """从队列中取出导航事件，触发 AI 播报"""
        while True:
            try:
                event_code = nav_event_queue.get_nowait()
                if nav_paused:   # 场景对话中，先不播报导航事件
                    continue
                prompt = NAV_EVENT_PROMPTS.get(event_code)
                if prompt:
                    mic_enabled.clear()
                    print(f"[NavEvent] Processing: {event_code} -> {prompt[:40]}...")
                    await asyncio.sleep(0.3)
                    await client.trigger_response_with_text(prompt)
            except queue.Empty:
                pass
            await asyncio.sleep(0.1)

    async def startup_test(client: OmniRealtimeClient):
        await asyncio.sleep(2.0)
        print("\n[StartupTest] >>> 发送文本: 系统启动成功\n", flush=True)
        mic_enabled.clear()
        await asyncio.sleep(0.8)   # 确保麦克风已停止上传
        await client.trigger_response_with_text("系统启动成功")
        await asyncio.sleep(8.0)
        print("\n[StartupTest] <<< 测试结束\n", flush=True)

    try:
        await client.connect()
        tasks = [
            asyncio.create_task(client.handle_messages()),
            asyncio.create_task(start_microphone_streaming(client)),
            asyncio.create_task(rosbridge_image_sub(client)),
            asyncio.create_task(nav_event_processor(client)),
            asyncio.create_task(startup_test(client)),
            asyncio.create_task(scene_handler(client)),
        ]
        await asyncio.gather(*tasks)   # 改为 gather，避免一个任务崩溃导致全部退出
    except Exception as e:
        print("Error:", e)
    finally:
        audio_queue.put(None)
        if audio_player:
            audio_player.join(timeout=1)
        await client.close()

if __name__ == "__main__":
    asyncio.run(main())