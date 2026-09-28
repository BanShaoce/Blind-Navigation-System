import unittest

from blind_navigation.common.protocol import (
    NavigationCommand,
    NavigationEvent,
    parse_command,
    parse_event,
)


class ProtocolTest(unittest.TestCase):
    def test_protocol_values_are_stable(self) -> None:
        self.assertEqual(str(NavigationCommand.GO_SEAT), "GO_A")
        self.assertEqual(str(NavigationEvent.ARRIVED), "ARRIVED")

    def test_unknown_wire_values_are_rejected(self) -> None:
        self.assertIs(parse_command(" GO_A "), NavigationCommand.GO_SEAT)
        self.assertIsNone(parse_event("UNKNOWN"))


if __name__ == "__main__":
    unittest.main()
