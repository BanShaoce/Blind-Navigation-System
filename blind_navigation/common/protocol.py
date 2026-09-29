"""导航与感知进程之间的本地 UDP 协议。"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum


class WireValue(str, Enum):
    def __str__(self) -> str:
        return self.value


class NavigationCommand(WireValue):
    """感知进程发送给导航进程的命令。"""

    GO_SEAT = "GO_A"
    GO_ENTRANCE = "GO_B"
    STOP = "STOP"
    PAUSE = "PAUSE_NAV"
    RESUME = "RESUME_NAV"
    MUTE = "MUTE_NAV"
    UNMUTE = "UNMUTE_NAV"


class NavigationEvent(WireValue):
    """导航进程发送给感知进程的事件。"""

    ARRIVED = "ARRIVED"
    BLOCKED = "BLOCKED"
    PATH_CLEAR = "PATH_CLEAR"
    PLAN_FAILED = "PLAN_FAILED"
    NAV_STARTED = "NAV_STARTED"
    NAV_STOPPED = "NAV_STOPPED"
    OFF_TRACK_STOP = "OFF_TRACK_STOP"


@dataclass(frozen=True, slots=True)
class UdpEndpoints:
    """仅监听回环地址，避免未授权的局域网控制。"""

    host: str = "127.0.0.1"
    command_port: int = 5000
    event_port: int = 5001


def parse_command(value: str) -> NavigationCommand | None:
    try:
        return NavigationCommand(value.strip())
    except ValueError:
        return None


def parse_event(value: str) -> NavigationEvent | None:
    try:
        return NavigationEvent(value.strip())
    except ValueError:
        return None
