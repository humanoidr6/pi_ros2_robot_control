import unittest
from goal_state import GoalState


class GoalStateTests(unittest.TestCase):
    def ready(self):
        state = GoalState()
        state.armed(state.arm_ticket())
        return state

    def test_initial_state_rejects_motion(self):
        with self.assertRaises(RuntimeError):
            GoalState().reserve()

    def test_stop_before_acceptance_rejects_late_goal(self):
        state = self.ready()
        ticket = state.reserve()
        self.assertIsNone(state.stop())
        self.assertFalse(state.accepted(ticket, object()))

    def test_stop_after_acceptance_returns_handle_to_cancel(self):
        state = self.ready()
        handle = object()
        self.assertTrue(state.accepted(state.reserve(), handle))
        self.assertIs(state.stop(), handle)

    def test_late_old_result_does_not_clear_new_goal(self):
        state = self.ready()
        old = object(); state.accepted(state.reserve(), old)
        state.stop(); state.armed(state.arm_ticket())
        new = object(); state.accepted(state.reserve(), new)
        self.assertFalse(state.completed(old))
        self.assertIs(state.handle, new)

    def test_stop_during_arm_prevents_activation(self):
        state = GoalState()
        ticket = state.arm_ticket()
        state.stop()
        self.assertFalse(state.armed(ticket))

    def test_duplicate_pending_goal_rejected(self):
        state = self.ready(); state.reserve()
        with self.assertRaises(RuntimeError):
            state.reserve()

    def test_completed_goal_requires_new_arm(self):
        state = self.ready(); handle = object()
        state.accepted(state.reserve(), handle)
        self.assertTrue(state.completed(handle))
        with self.assertRaises(RuntimeError):
            state.reserve()


if __name__ == '__main__':
    unittest.main()
