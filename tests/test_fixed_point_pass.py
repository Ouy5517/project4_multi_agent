import unittest

from common.models import Vec2
from decision.pass_fsm import PassConfig
from simulation.fixed_point_simulator import FixedPointSimulator


class FixedPointPassTest(unittest.TestCase):
    def test_successful_pass_to_fixed_target(self) -> None:
        target = Vec2(7.0, 4.0)
        result = FixedPointSimulator(PassConfig(fixed_target=target)).run()

        self.assertTrue(result.success)
        self.assertEqual(result.final_state, "DONE")
        self.assertEqual(result.ball_owner_id, "R2")
        self.assertLessEqual(result.receiver_position.distance_to(target), 0.12)
        self.assertEqual(result.message_count, 2)

        actions = [event.action for event in result.events]
        self.assertIn("SEND_MESSAGE", actions)
        self.assertIn("RECEIVE_BALL", actions)

    def test_target_outside_field_fails_with_reason(self) -> None:
        result = FixedPointSimulator(
            PassConfig(fixed_target=Vec2(99.0, 99.0))
        ).run()

        self.assertFalse(result.success)
        self.assertEqual(result.final_state, "FAILED")
        self.assertIn("outside field", result.events[-1].detail)

    def test_short_duration_fails_instead_of_claiming_success(self) -> None:
        result = FixedPointSimulator().run(duration_s=0.2)

        self.assertFalse(result.success)
        self.assertEqual(result.final_state, "FAILED")
        self.assertIn("duration exhausted", result.events[-1].detail)


if __name__ == "__main__":
    unittest.main()

