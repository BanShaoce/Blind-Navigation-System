import os
import unittest
from unittest.mock import patch

from blind_navigation.common.settings import PerceptionSettings


class SettingsTest(unittest.TestCase):
    def test_api_key_is_required(self) -> None:
        environment = os.environ.copy()
        environment.pop("DASHSCOPE_API_KEY", None)
        with patch.dict(os.environ, environment, clear=True):
            with self.assertRaisesRegex(RuntimeError, "DASHSCOPE_API_KEY"):
                PerceptionSettings.from_env()

    def test_settings_are_loaded_from_environment(self) -> None:
        with patch.dict(
            os.environ,
            {"DASHSCOPE_API_KEY": "test-key", "OMNI_VOICE": "TestVoice"},
            clear=True,
        ):
            settings = PerceptionSettings.from_env()
        self.assertEqual(settings.api_key, "test-key")
        self.assertEqual(settings.voice, "TestVoice")


if __name__ == "__main__":
    unittest.main()
