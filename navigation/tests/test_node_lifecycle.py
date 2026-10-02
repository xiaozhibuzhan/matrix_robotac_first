"""ROS lifecycle regression tests using a fake context; no ROS or SDK runs."""
import unittest
from unittest import mock

from navigation.node import _run_ros_node


class ExternalShutdown(Exception):
    """Stand-in for rclpy.executors.ExternalShutdownException."""


class FakeROS:
    def __init__(self, events, interruption=None, signal_shutdown=False):
        self.events=events
        self.interruption=interruption
        self.signal_shutdown=signal_shutdown
        self.context_active=False
        self.context_shutdowns=0

    def init(self, *, args):
        self.args=args
        self.context_active=True
        self.events.append('init')

    def spin(self, node):
        self.events.append('spin')
        if self.signal_shutdown:
            self.try_shutdown()
        if self.interruption is not None:
            raise self.interruption

    def try_shutdown(self):
        self.events.append('try_shutdown')
        if self.context_active:
            self.context_active=False
            self.context_shutdowns+=1

    def ok(self):
        raise AssertionError('A check-then-shutdown sequence races with signals')

    def shutdown(self):
        raise AssertionError('Shutdown must use the idempotent ROS API')


class NodeLifecycleTests(unittest.TestCase):
    def make_node(self, events):
        node=mock.Mock()
        node.close.side_effect=lambda: events.append('close')
        node.destroy_node.side_effect=lambda: events.append('destroy')
        return node

    def test_normal_return_closes_node_before_ros_context(self):
        events=[]; ros=FakeROS(events); node=self.make_node(events)
        _run_ros_node(ros, lambda: node, ['--ros-args'], ExternalShutdown)
        self.assertEqual(ros.args, ['--ros-args'])
        self.assertEqual(events, ['init', 'spin', 'close', 'destroy', 'try_shutdown'])
        self.assertEqual(ros.context_shutdowns, 1)

    def test_interrupt_after_signal_shutdown_is_idempotent(self):
        for interruption in (KeyboardInterrupt(), ExternalShutdown()):
            with self.subTest(interruption=type(interruption).__name__):
                events=[]; ros=FakeROS(events, interruption, signal_shutdown=True)
                node=self.make_node(events)
                _run_ros_node(ros, lambda: node, [], ExternalShutdown)
                self.assertEqual(events, ['init', 'spin', 'try_shutdown', 'close', 'destroy', 'try_shutdown'])
                self.assertEqual(ros.context_shutdowns, 1)
                node.close.assert_called_once_with()
                node.destroy_node.assert_called_once_with()

    def test_unexpected_spin_error_is_not_hidden(self):
        events=[]; error=RuntimeError('unexpected navigation failure')
        ros=FakeROS(events, error); node=self.make_node(events)
        with self.assertRaises(RuntimeError) as caught:
            _run_ros_node(ros, lambda: node, [], ExternalShutdown)
        self.assertIs(caught.exception, error)
        self.assertEqual(events[-3:], ['close', 'destroy', 'try_shutdown'])
        self.assertFalse(ros.context_active)

    def test_constructor_failure_still_shuts_down_context(self):
        events=[]; ros=FakeROS(events); error=RuntimeError('startup failure')
        with self.assertRaises(RuntimeError) as caught:
            _run_ros_node(ros, mock.Mock(side_effect=error), [], ExternalShutdown)
        self.assertIs(caught.exception, error)
        self.assertEqual(events, ['init', 'try_shutdown'])
        self.assertFalse(ros.context_active)

    def test_close_failure_still_destroys_node_and_context(self):
        events=[]; ros=FakeROS(events); node=self.make_node(events)
        error=RuntimeError('close failure'); node.close.side_effect=error
        with self.assertRaises(RuntimeError) as caught:
            _run_ros_node(ros, lambda: node, [], ExternalShutdown)
        self.assertIs(caught.exception, error)
        node.destroy_node.assert_called_once_with()
        self.assertEqual(events[-2:], ['destroy', 'try_shutdown'])
        self.assertFalse(ros.context_active)

    def test_destroy_failure_still_shuts_down_context(self):
        events=[]; ros=FakeROS(events); node=self.make_node(events)
        error=RuntimeError('destroy failure'); node.destroy_node.side_effect=error
        with self.assertRaises(RuntimeError) as caught:
            _run_ros_node(ros, lambda: node, [], ExternalShutdown)
        self.assertIs(caught.exception, error)
        self.assertEqual(events[-2:], ['close', 'try_shutdown'])
        self.assertFalse(ros.context_active)

    def test_unexpected_shutdown_error_is_not_hidden(self):
        events=[]; ros=FakeROS(events); node=self.make_node(events)
        error=RuntimeError('unrelated context failure')
        ros.try_shutdown=mock.Mock(side_effect=error)
        with self.assertRaises(RuntimeError) as caught:
            _run_ros_node(ros, lambda: node, [], ExternalShutdown)
        self.assertIs(caught.exception, error)
        node.close.assert_called_once_with()
        node.destroy_node.assert_called_once_with()


if __name__=='__main__':
    unittest.main()
