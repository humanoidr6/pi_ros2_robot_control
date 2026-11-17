import threading
import time
import unittest
from unittest.mock import Mock, patch

from remote import Remote


class RemoteTests(unittest.TestCase):
    def make_remote(self):
        remote = Remote.__new__(Remote)
        remote.root = Mock()
        remote.enabled = True
        remote.busy = False
        remote.speed = Mock(get=lambda: 100)
        remote.invert_l = Mock(get=lambda: False)
        remote.invert_r = Mock(get=lambda: True)
        remote.lock = threading.Lock()
        remote.target = (0, 0)
        remote.heartbeat = time.monotonic()
        return remote

    def test_corrected_wheel_polarity(self):
        remote = self.make_remote()
        # Desired chassis movement -> raw GPIO wheel command.
        for direction, expected in [((1, 1), (100, -100)),
                                    ((-1, -1), (-100, 100)),
                                    ((-1, 1), (-100, -100)),
                                    ((1, -1), (100, 100))]:
            remote.start(direction)
            self.assertEqual(remote.target, expected)

    def test_estop_and_save_disable_new_movement(self):
        for enabled, busy in [(False, False), (True, True)]:
            remote = self.make_remote()
            remote.enabled, remote.busy = enabled, busy
            remote.start((1, 1))
            self.assertEqual(remote.target, (0, 0))

    def test_stale_gui_heartbeat_discards_old_command(self):
        remote = self.make_remote()
        remote.target = (100, 100)
        remote.heartbeat = time.monotonic() - 1
        remote.alive = True

        def response(*args, **kwargs):
            remote.alive = False

        with patch('remote.request', side_effect=response) as send, patch('remote.time.sleep'):
            remote.commands()
        self.assertEqual(remote.target, (0, 0))
        send.assert_called_once_with('/stop', {}, timeout=0.25)

    def test_network_failure_discards_command(self):
        remote = self.make_remote()
        remote.target = (100, -100)
        remote.alive = True

        def failed_request(*args, **kwargs):
            remote.alive = False
            raise OSError('Disconnected')

        with patch('remote.request', side_effect=failed_request), patch('remote.time.sleep'):
            remote.commands()
        self.assertEqual(remote.target, (0, 0))

    def test_release_clears_motion(self):
        remote = self.make_remote()
        remote.start((1, 1))
        remote.stop()
        self.assertEqual(remote.target, (0, 0))


if __name__ == '__main__':
    unittest.main()
