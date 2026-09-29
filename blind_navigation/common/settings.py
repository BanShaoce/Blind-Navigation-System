"""进程配置与环境变量解析。"""

from __future__ import annotations

import os
from dataclasses import dataclass


def _env_bool(name: str, default: bool) -> bool:
    value = os.getenv(name)
    if value is None:
        return default
    return value.strip().lower() in {"1", "true", "yes", "on"}


@dataclass(frozen=True, slots=True)
class PerceptionSettings:
    api_key: str
    api_base_url: str = "wss://dashscope.aliyuncs.com/api-ws/v1/realtime"
    model: str = "qwen3-omni-flash-realtime"
    voice: str = "Cherry"
    rosbridge_url: str = "ws://127.0.0.1:9090"
    image_topic: str = "/zed/zed_node/rgb/color/rect/image/compressed"
    image_fps: float = 2.0
    audio_device: str = "pulse"
    audio_input_rate: int = 48_000
    audio_output_rate: int = 24_000
    debug: bool = False
    startup_announcement: bool = True

    @classmethod
    def from_env(cls) -> "PerceptionSettings":
        api_key = os.getenv("DASHSCOPE_API_KEY", "").strip()
        if not api_key:
            raise RuntimeError(
                "缺少 DASHSCOPE_API_KEY。请复制 .env.example 中的变量并在启动前导出。"
            )
        return cls(
            api_key=api_key,
            api_base_url=os.getenv(
                "OMNI_API_BASE_URL", "wss://dashscope.aliyuncs.com/api-ws/v1/realtime"
            ),
            model=os.getenv("OMNI_MODEL", "qwen3-omni-flash-realtime"),
            voice=os.getenv("OMNI_VOICE", "Cherry"),
            rosbridge_url=os.getenv("ROSBRIDGE_URL", "ws://127.0.0.1:9090"),
            image_topic=os.getenv(
                "CAMERA_IMAGE_TOPIC",
                "/zed/zed_node/rgb/color/rect/image/compressed",
            ),
            image_fps=float(os.getenv("CAMERA_IMAGE_FPS", "2.0")),
            audio_device=os.getenv("AUDIO_INPUT_DEVICE", "pulse"),
            debug=_env_bool("BLIND_NAV_DEBUG", False),
            startup_announcement=_env_bool(
                "STARTUP_ANNOUNCEMENT", True
            ),
        )


@dataclass(frozen=True, slots=True)
class NavigationSettings:
    angle_threshold_deg: float = 10.0
    direction_threshold_deg: float = 10.0
    waypoint_reach_threshold: float = 0.4
    off_track_limit: float = 0.4
    off_track_check_interval: float = 1.5
    path_sample_distance: float = 0.5
