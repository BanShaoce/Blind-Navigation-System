"""不依赖网络或硬件的轻量意图识别。"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum

from blind_navigation.common.protocol import NavigationCommand


class IntentKind(str, Enum):
    SCENE = "scene"
    NAVIGATION = "navigation"
    CHAT = "chat"


@dataclass(frozen=True, slots=True)
class Intent:
    kind: IntentKind
    command: NavigationCommand | None = None


SCENE_KEYWORDS = ("描述", "看", "前面有", "环境", "场景", "这是什么", "前面")
NAVIGATION_KEYWORDS = (
    ("去座位", NavigationCommand.GO_SEAT),
    ("去门口", NavigationCommand.GO_ENTRANCE),
    ("停止导航", NavigationCommand.STOP),
    ("暂停导航", NavigationCommand.PAUSE),
    ("恢复导航", NavigationCommand.RESUME),
)


def detect_intent(transcript: str) -> Intent:
    text = transcript.strip()
    if any(keyword in text for keyword in SCENE_KEYWORDS):
        return Intent(IntentKind.SCENE)
    for keyword, command in NAVIGATION_KEYWORDS:
        if keyword in text:
            return Intent(IntentKind.NAVIGATION, command)
    return Intent(IntentKind.CHAT)
