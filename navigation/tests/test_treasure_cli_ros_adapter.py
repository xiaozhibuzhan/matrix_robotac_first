"""Exercise the real CLI adapter against the ROS 2 Humble public API.

No ROS installation is needed. In Humble, endpoint counts are exposed by
Publisher.get_subscription_count() and Node.count_publishers(topic_name).
Subscription deliberately has no get_publisher_count() method here.
"""
from __future__ import annotations

import json
import sys
import types
import unittest
from unittest.mock import patch

from navigation.treasure_cli import (
    ACK_TOPIC, COMMAND_TOPIC, STATUS_TOPIC, RosClient, run_request,
)


class HumbleString:
    __slots__ = ("data",)

    def __init__(self):
        self.data = ""


class HumbleQoSProfile:
    __slots__ = ("depth", "reliability", "durability")

    def __init__(self, *, depth, reliability, durability):
        self.depth = depth
        self.reliability = reliability
        self.durability = durability


class HumbleSubscription:
    __slots__ = ("topic", "callback", "qos")

    def __init__(self, topic, callback, qos):
        self.topic = topic
        self.callback = callback
        self.qos = qos


class HumblePublisher:
    __slots__ = ("runtime", "topic", "qos")

    def __init__(self, runtime, topic, qos):
        self.runtime = runtime
        self.topic = topic
        self.qos = qos

    def get_subscription_count(self):
        return self.runtime.subscription_counts.get(self.topic, 0)

    def publish(self, message):
        request = json.loads(message.data)
        self.runtime.published.append((self.topic, request))
        self.runtime.pending.append((ACK_TOPIC, {
            "request_id": request["request_id"],
            "success": True,
            "message": "round started",
            "status": {"round_state": "ACTIVE", "round_id": "test-round"},
        }))


class HumbleNode:
    __slots__ = ("runtime", "name", "subscriptions", "publishers", "destroyed")

    def __init__(self, runtime, name):
        self.runtime = runtime
        self.name = name
        self.subscriptions = []
        self.publishers = []
        self.destroyed = False

    def create_subscription(self, message_type, topic, callback, qos):
        if message_type is not HumbleString:
            raise TypeError("unexpected message type")
        subscription = HumbleSubscription(topic, callback, qos)
        self.subscriptions.append(subscription)
        return subscription

    def create_publisher(self, message_type, topic, qos):
        if message_type is not HumbleString:
            raise TypeError("unexpected message type")
        publisher = HumblePublisher(self.runtime, topic, qos)
        self.publishers.append(publisher)
        return publisher

    def count_publishers(self, topic_name):
        self.runtime.publisher_queries.append(topic_name)
        return self.runtime.publisher_counts.get(topic_name, 0)

    def destroy_node(self):
        self.destroyed = True


class HumbleRuntime:
    def __init__(self):
        self.now = 0.0
        self.running = False
        self.init_args = None
        self.nodes = []
        self.subscription_counts = {COMMAND_TOPIC: 1}
        self.publisher_counts = {ACK_TOPIC: 1}
        self.publisher_queries = []
        self.published = []
        self.pending = []

    def modules(self):
        rclpy = types.ModuleType("rclpy")
        rclpy.init = self.init
        rclpy.create_node = self.create_node
        rclpy.ok = lambda: self.running
        rclpy.spin_once = self.spin_once
        rclpy.shutdown = self.shutdown
        qos = types.ModuleType("rclpy.qos")
        qos.QoSProfile = HumbleQoSProfile
        qos.ReliabilityPolicy = types.SimpleNamespace(RELIABLE="reliable")
        qos.DurabilityPolicy = types.SimpleNamespace(
            VOLATILE="volatile", TRANSIENT_LOCAL="transient_local",
        )
        rclpy.qos = qos
        std_msgs = types.ModuleType("std_msgs")
        messages = types.ModuleType("std_msgs.msg")
        messages.String = HumbleString
        std_msgs.msg = messages
        return {
            "rclpy": rclpy, "rclpy.qos": qos,
            "std_msgs": std_msgs, "std_msgs.msg": messages,
        }

    def init(self, *, args):
        self.init_args = args
        self.running = True

    def create_node(self, name):
        node = HumbleNode(self, name)
        self.nodes.append(node)
        return node

    def spin_once(self, node, *, timeout_sec):
        self.now += timeout_sec
        if self.pending:
            topic, payload = self.pending.pop(0)
            message = HumbleString()
            message.data = json.dumps(payload)
            for subscription in node.subscriptions:
                if subscription.topic == topic:
                    subscription.callback(message)

    def shutdown(self):
        self.running = False


class TreasureRosAdapterTest(unittest.TestCase):
    def setUp(self):
        self.runtime = HumbleRuntime()
        modules = patch.dict(sys.modules, self.runtime.modules())
        modules.start()
        self.addCleanup(modules.stop)
        self.client = RosClient("request-1")
        self.addCleanup(self.client.close)

    def request(self, action="new-round", timeout=0.3):
        return run_request(
            self.client, {"request_id": "request-1", "action": action}, timeout,
            clock=lambda: self.runtime.now,
        )

    def test_constructs_humble_topics_qos_and_closes_resources(self):
        node = self.client.node
        self.assertEqual(self.runtime.init_args, [])
        self.assertEqual(
            [subscription.topic for subscription in node.subscriptions],
            [ACK_TOPIC, STATUS_TOPIC],
        )
        self.assertFalse(hasattr(self.client.ack_subscription, "get_publisher_count"))
        self.assertEqual(self.client.ack_subscription.qos.durability, "volatile")
        self.assertEqual(self.client.status_subscription.qos.durability, "transient_local")
        self.assertEqual(self.client.status_subscription.qos.depth, 1)
        self.assertEqual(self.client.publisher.topic, COMMAND_TOPIC)
        for endpoint in [*node.subscriptions, *node.publishers]:
            self.assertEqual(endpoint.qos.reliability, "reliable")
        self.client.close()
        self.assertTrue(node.destroyed)
        self.assertIsNone(self.client.node)
        self.assertFalse(self.runtime.running)

    def test_new_round_succeeds_with_humble_adapter(self):
        response = self.request()
        self.assertTrue(response["success"])
        self.assertEqual(response["status"]["round_state"], "ACTIVE")
        self.assertEqual(self.runtime.published, [(COMMAND_TOPIC, {
            "request_id": "request-1", "action": "new-round",
        })])
        self.assertIn(ACK_TOPIC, self.runtime.publisher_queries)

    def test_missing_command_subscriber_does_not_send(self):
        self.runtime.subscription_counts[COMMAND_TOPIC] = 0
        with self.assertRaisesRegex(TimeoutError, "no command was sent"):
            self.request()
        self.assertEqual(self.runtime.published, [])
        self.assertAlmostEqual(self.runtime.now, 0.3)

    def test_missing_ack_publisher_does_not_send(self):
        self.runtime.publisher_counts[ACK_TOPIC] = 0
        with self.assertRaisesRegex(TimeoutError, "no command was sent"):
            self.request()
        self.assertEqual(self.runtime.published, [])
        self.assertAlmostEqual(self.runtime.now, 0.3)

    def test_status_is_read_only_without_command_or_ack_peers(self):
        self.runtime.subscription_counts.clear()
        self.runtime.publisher_counts.clear()
        status = {"state": "IDLE", "round_id": "test-round"}
        self.runtime.pending.append((STATUS_TOPIC, status))
        self.assertEqual(self.request("status"), status)
        self.assertEqual(self.runtime.published, [])
        self.assertEqual(self.runtime.publisher_queries, [])


if __name__ == "__main__":
    unittest.main()
