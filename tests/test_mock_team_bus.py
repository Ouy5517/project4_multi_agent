import unittest

from common.models import Vec2
from communication.mock_team_bus import MessageType, MockTeamBus, TeamMessage


class MockTeamBusTest(unittest.TestCase):
    def test_message_is_delivered_only_to_target_receiver(self) -> None:
        bus = MockTeamBus()
        message = TeamMessage(
            MessageType.PASS_TARGET, "R1", "R2", 0.0, Vec2(7.0, 4.0)
        )
        bus.publish(message)

        self.assertIsNone(bus.consume("R3", MessageType.PASS_TARGET))
        self.assertEqual(bus.consume("R2", MessageType.PASS_TARGET), message)
        self.assertEqual(len(bus.history), 1)


if __name__ == "__main__":
    unittest.main()

