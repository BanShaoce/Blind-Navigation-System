# omni_realtime_client.py
# -- coding: utf-8 --
import asyncio
import websockets
import json
import base64
import time
from typing import Optional, Callable, Dict, Any
from enum import Enum

DEBUG = True

class TurnDetectionMode(Enum):
    SERVER_VAD = "server_vad"
    MANUAL = "manual"

class OmniRealtimeClient:
    """
    与 Omni Realtime API 交互的客户端（含调试输出）。
    """
    def __init__(
        self,
        base_url: str,
        api_key: str,
        model: str = "",
        voice: str = "Ethan",
        turn_detection_mode: TurnDetectionMode = TurnDetectionMode.MANUAL,
        on_text_delta: Optional[Callable[[str], None]] = None,
        on_audio_delta: Optional[Callable[[bytes], None]] = None,
        on_interrupt: Optional[Callable[[], None]] = None,
        extra_event_handlers: Optional[Dict[str, Callable[[Dict[str, Any]], None]]] = None,
        instructions: Optional[str] = None, 
        on_text_done: Optional[Callable[[str], None]] = None                      # == NEW: 接收系统提示词
    ):
        self.base_url = base_url
        self.api_key = api_key
        self.model = model
        self.voice = voice
        self.ws = None
        self.on_text_delta = on_text_delta
        self.on_audio_delta = on_audio_delta
        self.on_interrupt = on_interrupt
        self.turn_detection_mode = turn_detection_mode
        self.extra_event_handlers = extra_event_handlers or {}
        self._is_responding = False
        self.instructions = instructions                            # == NEW: 存储提示词
        self.on_text_done = on_text_done
        #self._vad_enabled = True          # 当前是否启用 VAD
        #self._pending_vad_restore = False # 是否需要在回复完成后恢复 VAD
        #self._session_updated_event = asyncio.Event()
        self._event_counter = 0
        self._input_audio_cleared_event = asyncio.Event()

    async def connect(self) -> None:
        url = f"{self.base_url}?model={self.model}"
        headers = {"Authorization": f"Bearer {self.api_key}"}
        print(f"[WS] Connecting to {url}")
        self.ws = await websockets.connect(url, extra_headers=headers)

        cfg: Dict[str, Any] = {
            "modalities": ["text", "audio"],
            "voice": self.voice,
            "input_audio_format": "pcm16",
            "output_audio_format": "pcm16",
            "input_audio_transcription": {"model": "gummy-realtime-v1"},
            "turn_detection": None if self.turn_detection_mode == TurnDetectionMode.MANUAL else {
                "type": "server_vad",
                "threshold": 0.1,
                "prefix_padding_ms": 500,
                "silence_duration_ms": 900
            }
        }
        # == MODIFIED: 如果提供了系统提示词，就注入到 session.update 配置里
        if self.instructions:
            cfg["instructions"] = self.instructions

        print(f"[WS] Sending session.update: {cfg}")
        await self.send_event({"type": "session.update", "session": cfg})
        #self._vad_config = cfg.get("turn_detection")  # 可能为 dict 或 None
        #self._vad_enabled = (self._vad_config is not None)

    async def trigger_response_with_image_and_text(self, image_bytes: bytes, text: str):
        print(f"[WS] Triggering AI response with image and text: {text[:50]}...")

        await self.send_event({"type": "input_audio_buffer.clear"})
        await asyncio.sleep(0.15)

        img_b64 = base64.b64encode(image_bytes).decode("utf-8")

        # 图像必须发送到 image buffer，而不是塞进 conversation.item.create 的 content
        await self.send_event({"type": "input_image_buffer.append", "image": img_b64})

        await self.send_event({
            "type": "conversation.item.create",
            "item": {
                "type": "message",
                "role": "user",
                "content": [{"type": "input_text", "text": text}]
            }
        })

        await self.send_event({"type": "response.create"})
    
    async def send_event(self, event: Dict[str, Any]) -> None:
        self._event_counter += 1
        eid = f"event_{int(time.time() * 1000)}_{self._event_counter}"
        event["event_id"] = eid
        if DEBUG:
            print(f"[WS ▶] {event['type']} (id={eid})")
        await self.ws.send(json.dumps(event))

    async def commit_audio_buffer(self) -> None:
        await self.send_event({"type": "input_audio_buffer.commit"})

    async def append_image(self, image_chunk: bytes) -> None:
        b64 = base64.b64encode(image_chunk).decode()
        await self.send_event({"type": "input_image_buffer.append", "image": b64})

    async def handle_messages(self) -> None:
        try:
            async for msg in self.ws:
                ev = json.loads(msg)
                t = ev.get("type", "<no-type>")

                # 只保留错误和重要状态，其余调试信息由 DEBUG 控制
                if DEBUG:
                    print(f"[WS ◀] Event: {t}, keys: {list(ev.keys())}")
                    if t.startswith("response."):
                        print(f"    -> full event: {json.dumps(ev, ensure_ascii=False)[:500]}")

                if t == "error":
                    print("[WS] ERROR:", ev.get("error"))
                    continue

                if t == "response.created":
                    self._is_responding = True

                if t == "input_audio_buffer.speech_started" and self._is_responding:
                    print("[Interrupt] 用户语音打断")
                    if self.on_interrupt:
                        self.on_interrupt()
                    await self.send_event({"type": "response.cancel"})
                    self._is_responding = False
                    continue

                if t == "response.text.delta" and self.on_text_delta:
                    self.on_text_delta(ev["delta"])

                # 阿里云实时转录文本 delta
                if t == "response.audio_transcript.delta" and self.on_text_delta:
                    self.on_text_delta(ev["delta"])

                if t == "conversation.item.input_audio_transcription.completed":
                    transcript = ev.get("transcript", "")
                    print(f"[DEBUG] transcript = {repr(transcript)}", flush=True)
                    if transcript and self.on_text_done:
                        self.on_text_done(transcript)

                if t == "response.audio.delta" and self.on_audio_delta:
                    self.on_audio_delta(base64.b64decode(ev["delta"]))

                if t == "input_audio_buffer.cleared":
                    self._input_audio_cleared_event.set()

                if t == "response.done":
                    self._is_responding = False

                if t in self.extra_event_handlers:
                    self.extra_event_handlers[t](ev)

        except Exception as e:
            print("[WS] Connection error:", e)

    async def trigger_response_with_text(self, text: str):
        print(f"[WS] Triggering AI response with text: {text[:50]}...")
        await self.send_event({"type": "input_audio_buffer.clear"})
        await asyncio.sleep(0.15)
        await self.send_event({
            "type": "conversation.item.create",
            "item": {
                "type": "message",
                "role": "user",
                "content": [{"type": "input_text", "text": text}]
            }
        })
        await self.send_event({"type": "response.create"})

    async def close(self) -> None:
        if self.ws:
            await self.ws.close()
            print("[WS] Closed")
