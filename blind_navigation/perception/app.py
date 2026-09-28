"""语音、图像与导航事件的多模态感知进程。"""

from __future__ import annotations

import asyncio
import base64
import json
import queue
import socket
import subprocess
import threading
import time
from typing import Any

import numpy as np
import pyaudio
import webrtcvad
import websockets

from blind_navigation.common.protocol import (
    NavigationCommand,
    NavigationEvent,
    UdpEndpoints,
    parse_event,
)
from blind_navigation.common.settings import PerceptionSettings
from blind_navigation.perception.intents import IntentKind, detect_intent
from blind_navigation.perception.realtime_client import (
    OmniRealtimeClient,
    TurnDetectionMode,
)

SYSTEM_PROMPT = """
你是一个面向盲人用户的智能辅助导航助手，必须使用简洁、自然的中文语音回答。
你会接收用户语音和摄像头图像。用户询问周围环境时，优先说明正前方的障碍物、
行人、门和楼梯等安全相关信息。收到“导航系统通知：”开头的消息时，将其改写为
一句亲切、清楚的用户提示，不要输出标签或不可朗读的符号。
""".strip()

NAV_EVENT_PROMPTS = {
    NavigationEvent.ARRIVED: "导航系统通知：已经到达目的地。",
    NavigationEvent.BLOCKED: "导航系统通知：前方暂时无法通过，请耐心等待，不要移动。",
    NavigationEvent.PATH_CLEAR: "导航系统通知：路径已经畅通，可以继续前进。",
    NavigationEvent.PLAN_FAILED: "导航系统通知：路径规划失败，当前无法继续导航。",
    NavigationEvent.NAV_STARTED: "导航系统通知：导航已经开始，请跟随振动指引前行。",
    NavigationEvent.NAV_STOPPED: "导航系统通知：导航已停止。",
    NavigationEvent.OFF_TRACK_STOP: "导航系统通知：用户偏离路线过多，无需播报。",
}


class AudioPlayer:
    """串行播放 API 返回的 PCM 音频。"""

    def __init__(self, rate: int = 24_000, chunk: int = 3_200) -> None:
        self.rate = rate
        self.chunk = chunk
        self.queue: queue.Queue[bytes | None] = queue.Queue()
        self.interrupted = threading.Event()
        self.ready = threading.Event()
        self.available = False
        self.thread: threading.Thread | None = None

    def start(self) -> None:
        self.thread = threading.Thread(target=self._run, daemon=True, name="audio-player")
        self.thread.start()
        self.ready.wait(timeout=3.0)

    def _run(self) -> None:
        audio = pyaudio.PyAudio()
        try:
            stream = audio.open(
                format=pyaudio.paInt16,
                channels=1,
                rate=self.rate,
                output=True,
                frames_per_buffer=self.chunk,
            )
        except Exception as exc:
            print(f"[Audio] 无可用输出设备，禁用播放: {exc}")
            audio.terminate()
            self.ready.set()
            return

        self.available = True
        self.ready.set()
        try:
            while True:
                data = self.queue.get()
                try:
                    if data is None:
                        return
                    if self.interrupted.is_set():
                        self.interrupted.clear()
                        continue
                    stream.write(data)
                finally:
                    self.queue.task_done()
        finally:
            stream.stop_stream()
            stream.close()
            audio.terminate()

    def enqueue(self, data: bytes) -> None:
        if self.available:
            self.queue.put(data)

    def interrupt(self) -> None:
        self.interrupted.set()
        while True:
            try:
                self.queue.get_nowait()
            except queue.Empty:
                break
            else:
                self.queue.task_done()

    async def wait_until_idle(self) -> None:
        await asyncio.to_thread(self.queue.join)

    def close(self) -> None:
        if self.available:
            self.queue.put(None)
        if self.thread:
            self.thread.join(timeout=1.0)


class PerceptionApplication:
    def __init__(
        self,
        settings: PerceptionSettings,
        endpoints: UdpEndpoints | None = None,
    ) -> None:
        self.settings = settings
        self.endpoints = endpoints or UdpEndpoints()
        self.player = AudioPlayer(rate=settings.audio_output_rate)
        self.client: OmniRealtimeClient | None = None
        self.latest_image: bytes | None = None
        self.latest_image_lock = threading.Lock()
        self.nav_events: queue.Queue[NavigationEvent] = queue.Queue()
        self.nav_paused = False
        self.is_playing = False
        self.response_text = ""
        self.mic_enabled = asyncio.Event()
        self.mic_enabled.set()
        self.scene_requested = asyncio.Event()

    def send_command(self, command: NavigationCommand) -> None:
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
                sock.sendto(
                    str(command).encode(),
                    (self.endpoints.host, self.endpoints.command_port),
                )
        except OSError as exc:
            print(f"[UDP] 导航命令发送失败: {exc}")

    def listen_navigation_events(self) -> None:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
            sock.bind((self.endpoints.host, self.endpoints.event_port))
            print(f"[UDP] 事件监听端口: {self.endpoints.event_port}")
            while True:
                try:
                    data, _ = sock.recvfrom(1024)
                    event = parse_event(data.decode("utf-8", errors="replace"))
                    if event is not None:
                        self.nav_events.put(event)
                except OSError as exc:
                    print(f"[UDP] 事件接收失败: {exc}")

    def on_text_delta(self, text: str) -> None:
        self.response_text += text
        if self.settings.debug:
            print(text, end="", flush=True)

    def on_interrupt(self) -> None:
        print("[Audio] 用户打断当前播放")
        self.player.interrupt()

    def on_audio_data(self, audio_bytes: bytes) -> None:
        if not self.is_playing:
            self.is_playing = True
            self.mic_enabled.clear()
            self.send_command(NavigationCommand.MUTE)
        self.player.enqueue(audio_bytes)

    def on_response_done(self, _: dict[str, Any]) -> None:
        self.is_playing = False
        asyncio.get_running_loop().create_task(self._restore_after_response())

    async def _restore_after_response(self) -> None:
        await self.player.wait_until_idle()
        await asyncio.sleep(0.8)
        self.mic_enabled.set()
        self.send_command(NavigationCommand.UNMUTE)
        if self.nav_paused:
            self.send_command(NavigationCommand.RESUME)
            self.nav_paused = False
        self.response_text = ""

    def on_transcript_done(self, transcript: str) -> None:
        print(f"[语音识别] {transcript}")
        intent = detect_intent(transcript)
        if intent.kind is IntentKind.SCENE:
            if not self.nav_paused:
                self.send_command(NavigationCommand.PAUSE)
                self.nav_paused = True
            self.scene_requested.set()
            return
        if intent.kind is IntentKind.NAVIGATION and intent.command is not None:
            self.send_command(intent.command)
            if intent.command is NavigationCommand.PAUSE:
                self.nav_paused = True
            elif intent.command is NavigationCommand.RESUME:
                self.nav_paused = False
            return
        if self.client:
            asyncio.create_task(self.client.send_event({"type": "response.create"}))

    async def scene_handler(self) -> None:
        assert self.client is not None
        while True:
            await self.scene_requested.wait()
            self.scene_requested.clear()
            for _ in range(50):
                if not self.client.is_responding:
                    break
                await asyncio.sleep(0.1)
            if self.client.is_responding:
                await self.client.send_event({"type": "response.cancel"})
                await asyncio.sleep(0.1)
            with self.latest_image_lock:
                image = self.latest_image
            self.mic_enabled.clear()
            if image is None:
                await self.client.trigger_response_with_text("请告知用户暂时无法获取图像。")
            else:
                await self.client.trigger_response_with_image_and_text(
                    image,
                    "请简要描述前方场景，优先说明障碍物、人、门和楼梯。",
                )

    async def microphone_stream(self) -> None:
        assert self.client is not None
        target_rate = 16_000
        frame_seconds = 0.02
        input_chunk = int(self.settings.audio_input_rate * frame_seconds)
        target_chunk = int(target_rate * frame_seconds)
        silence_frames = int(0.5 / frame_seconds)
        vad = webrtcvad.Vad(2)
        command = [
            "arecord",
            "-D",
            self.settings.audio_device,
            "-r",
            str(self.settings.audio_input_rate),
            "-c",
            "1",
            "-f",
            "S16_LE",
            "--buffer-size",
            str(input_chunk * 4),
            "--period-size",
            str(input_chunk),
            "-t",
            "raw",
            "-q",
        ]
        process = subprocess.Popen(
            command, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL
        )
        print(f"[Mic] arecord 已启动，设备: {self.settings.audio_device}")
        is_speaking = False
        silence_count = 0
        try:
            assert process.stdout is not None
            while True:
                await self.mic_enabled.wait()
                raw = await asyncio.to_thread(process.stdout.read, input_chunk * 2)
                if not raw:
                    raise RuntimeError("麦克风音频流已中断")
                if self.client.is_responding:
                    is_speaking = False
                    silence_count = 0
                    continue

                input_audio = np.frombuffer(raw, dtype=np.int16)
                audio_16k = self._resample(input_audio, target_rate)
                audio_24k = self._resample(input_audio, self.settings.audio_output_rate)
                await self.client.send_event(
                    {
                        "type": "input_audio_buffer.append",
                        "audio": base64.b64encode(audio_24k.tobytes()).decode("utf-8"),
                    }
                )
                speech = vad.is_speech(audio_16k[:target_chunk].tobytes(), target_rate)
                if speech:
                    is_speaking = True
                    silence_count = 0
                elif is_speaking:
                    silence_count += 1
                    if silence_count >= silence_frames:
                        self.mic_enabled.clear()
                        is_speaking = False
                        silence_count = 0
                        await self.client.commit_audio_buffer()
        finally:
            process.kill()
            process.wait()

    def _resample(self, audio: np.ndarray, target_rate: int) -> np.ndarray:
        target_length = int(
            len(audio) * target_rate / self.settings.audio_input_rate
        )
        return np.interp(
            np.linspace(0, len(audio) - 1, target_length),
            np.arange(len(audio)),
            audio,
        ).astype(np.int16)

    async def image_stream(self) -> None:
        interval = 1.0 / self.settings.image_fps
        while True:
            try:
                async with websockets.connect(self.settings.rosbridge_url) as websocket:
                    await websocket.send(
                        json.dumps(
                            {
                                "op": "subscribe",
                                "topic": self.settings.image_topic,
                                "type": "sensor_msgs/CompressedImage",
                            }
                        )
                    )
                    print(f"[Rosbridge] 已订阅 {self.settings.image_topic}")
                    last_saved = 0.0
                    async for message in websocket:
                        data = json.loads(message)
                        if (
                            data.get("op") != "publish"
                            or data.get("topic") != self.settings.image_topic
                        ):
                            continue
                        now = time.monotonic()
                        if now - last_saved < interval:
                            continue
                        encoded = data.get("msg", {}).get("data", "")
                        if encoded:
                            with self.latest_image_lock:
                                self.latest_image = base64.b64decode(encoded)
                            last_saved = now
            except (websockets.ConnectionClosed, OSError) as exc:
                print(f"[Rosbridge] 连接中断，3 秒后重试: {exc}")
                await asyncio.sleep(3)

    async def process_navigation_events(self) -> None:
        assert self.client is not None
        while True:
            if (
                self.nav_paused
                or self.client.is_responding
                or self.is_playing
                or not self.mic_enabled.is_set()
            ):
                await asyncio.sleep(0.1)
                continue
            try:
                event = self.nav_events.get_nowait()
            except queue.Empty:
                await asyncio.sleep(0.1)
                continue
            prompt = NAV_EVENT_PROMPTS[event]
            if event is NavigationEvent.OFF_TRACK_STOP:
                continue
            self.mic_enabled.clear()
            await asyncio.sleep(0.3)
            await self.client.trigger_response_with_text(prompt)

    async def announce_startup(self) -> None:
        if not self.settings.startup_announcement:
            return
        assert self.client is not None
        await asyncio.sleep(2.0)
        self.mic_enabled.clear()
        await self.client.trigger_response_with_text("系统启动成功。")

    async def run(self) -> None:
        self.player.start()
        threading.Thread(
            target=self.listen_navigation_events,
            daemon=True,
            name="udp-event-listener",
        ).start()
        self.client = OmniRealtimeClient(
            base_url=self.settings.api_base_url,
            api_key=self.settings.api_key,
            model=self.settings.model,
            voice=self.settings.voice,
            turn_detection_mode=TurnDetectionMode.MANUAL,
            on_text_delta=self.on_text_delta,
            on_audio_delta=self.on_audio_data,
            on_interrupt=self.on_interrupt,
            on_text_done=self.on_transcript_done,
            extra_event_handlers={"response.done": self.on_response_done},
            instructions=SYSTEM_PROMPT,
            debug=self.settings.debug,
        )
        try:
            await self.client.connect()
            await asyncio.gather(
                self.client.handle_messages(),
                self.microphone_stream(),
                self.image_stream(),
                self.process_navigation_events(),
                self.announce_startup(),
                self.scene_handler(),
            )
        finally:
            self.player.close()
            await self.client.close()


async def _async_main() -> None:
    app = PerceptionApplication(PerceptionSettings.from_env())
    await app.run()


def main() -> None:
    try:
        asyncio.run(_async_main())
    except KeyboardInterrupt:
        print("感知进程已停止")


if __name__ == "__main__":
    main()
