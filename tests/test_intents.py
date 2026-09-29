import unittest

from blind_navigation.common.protocol import NavigationCommand
from blind_navigation.perception.intents import IntentKind, detect_intent


class IntentTest(unittest.TestCase):
    def test_scene_intent_has_priority(self) -> None:
        self.assertIs(detect_intent("请看一下前面的环境").kind, IntentKind.SCENE)

    def test_navigation_intent_maps_to_protocol(self) -> None:
        intent = detect_intent("带我去门口")
        self.assertIs(intent.kind, IntentKind.NAVIGATION)
        self.assertIs(intent.command, NavigationCommand.GO_ENTRANCE)

    def test_chat_fallback(self) -> None:
        self.assertIs(detect_intent("现在几点").kind, IntentKind.CHAT)


if __name__ == "__main__":
    unittest.main()
