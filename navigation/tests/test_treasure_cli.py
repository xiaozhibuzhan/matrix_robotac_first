"""CLI/protocol tests that intentionally run without an installed ROS stack."""
from __future__ import annotations

import contextlib
import io
import json
import unittest
import uuid

from navigation.treasure_cli import matching_ack, parse_request, run_request


class FakeClient:
    def __init__(self, *, ready_at=0.0, acknowledge_after=None, status=None):
        self.now = 0.0
        self.ready_at = ready_at
        self.acknowledge_after = acknowledge_after
        self.status = status
        self.ack = None
        self.sent = []
        self.running = True

    def is_ok(self):
        return self.running

    def command_ready(self):
        return self.now >= self.ready_at

    def publish_request(self, request):
        self.sent.append((self.now, dict(request)))

    def spin_once(self, timeout_sec):
        self.now += timeout_sec
        if self.sent and self.acknowledge_after is not None:
            if len(self.sent) >= self.acknowledge_after:
                self.ack = {
                    "request_id": self.sent[-1][1]["request_id"],
                    "success": True, "message": "accepted",
                }


class TreasureArgumentsTest(unittest.TestCase):
    def invalid(self, argv):
        with contextlib.redirect_stderr(io.StringIO()):
            with self.assertRaises(SystemExit) as raised:
                parse_request(argv)
        self.assertEqual(raised.exception.code, 2)

    def test_confirm_requires_deliberate_disappearance_flag(self):
        self.invalid(["confirm"])
        args, request = parse_request(["confirm", "--disappeared"])
        self.assertEqual(args.timeout, 15.0)
        self.assertIs(request["disappeared"], True)
        self.assertEqual(uuid.UUID(request["request_id"]).version, 4)

    def test_select_requires_explicit_finite_radius(self):
        self.invalid(["select", "box-1"])
        for radius in ("nan", "inf", "-inf"):
            self.invalid(["select", "box-1", "--radius=" + radius])
        _, request = parse_request(
            ["select", "box-1", "--radius", "0.18"], request_id="test-id",
        )
        self.assertEqual(request, {
            "request_id": "test-id", "action": "select",
            "box_id": "box-1", "radius": 0.18,
        })

    def test_radius_range_is_checked_by_controller(self):
        _, request = parse_request(["select", "box-1", "--radius=-1"])
        self.assertEqual(request["radius"], -1)

    def test_resume_preserves_round_identity(self):
        _, request = parse_request(["resume", "round_20261009"])
        self.assertEqual(request["round_id"], "round_20261009")
        self.invalid(["resume", "  "])

    def test_timeout_is_finite_nonnegative_before_or_after_command(self):
        for argv in (["--timeout", "2.5", "status"], ["status", "--timeout", "2.5"]):
            self.assertEqual(parse_request(argv)[0].timeout, 2.5)
        self.assertEqual(parse_request(["status", "--timeout", "0"])[0].timeout, 0)
        for value in ("-1", "nan", "inf", "-inf"):
            self.invalid(["status", "--timeout=" + value])

    def test_all_simple_actions_have_unique_requests(self):
        ids = set()
        for action in ("status", "new-round", "contact", "retry", "cancel", "skip", "finish"):
            _, request = parse_request([action])
            self.assertEqual(request["action"], action)
            self.assertEqual(set(request), {"request_id", "action"})
            ids.add(request["request_id"])
        self.assertEqual(len(ids), 7)


class TreasureProtocolTest(unittest.TestCase):
    def test_ignore_malformed_and_other_client_acknowledgements(self):
        valid = {"request_id": "ours", "success": True, "message": "ok", "status": {}}
        self.assertEqual(matching_ack(json.dumps(valid), "ours"), valid)
        self.assertIsNone(matching_ack(json.dumps(valid), "theirs"))
        for invalid in (
            "{", "[]", "null", "{}",
            json.dumps({**valid, "success": "true"}),
            json.dumps({**valid, "success": 1}),
            json.dumps({**valid, "message": None}),
            json.dumps({**valid, "status": []}),
        ):
            self.assertIsNone(matching_ack(invalid, "ours"))

    def test_discovery_precedes_publish_and_retransmits_same_id(self):
        client = FakeClient(ready_at=0.5, acknowledge_after=2)
        request = {"request_id": "same-id", "action": "new-round"}
        response = run_request(client, request, 5, clock=lambda: client.now)
        self.assertTrue(response["success"])
        self.assertEqual(len(client.sent), 2)
        self.assertGreaterEqual(client.sent[0][0], 0.5)
        self.assertGreaterEqual(client.sent[1][0] - client.sent[0][0], 1.0)
        self.assertEqual([item[1] for item in client.sent], [request, request])

    def test_missing_node_is_bounded_and_never_sends(self):
        client = FakeClient(ready_at=100)
        with self.assertRaisesRegex(TimeoutError, "no command was sent"):
            run_request(client, {"request_id": "id", "action": "contact"}, 0.3,
                        clock=lambda: client.now)
        self.assertEqual(client.sent, [])
        self.assertAlmostEqual(client.now, 0.3)

    def test_lost_ack_reports_unknown_outcome_instead_of_failure(self):
        client = FakeClient()
        with self.assertRaisesRegex(TimeoutError, "outcome is unknown"):
            run_request(client, {"request_id": "id", "action": "confirm"}, 0.3,
                        clock=lambda: client.now)
        self.assertEqual(len(client.sent), 1)

    def test_status_is_read_only(self):
        client = FakeClient(status={"state": "IDLE", "round_id": "r1"})
        response = run_request(client, {"request_id": "id", "action": "status"}, 2,
                               clock=lambda: client.now)
        self.assertEqual(response["round_id"], "r1")
        self.assertEqual(client.sent, [])

    def test_zero_timeout_does_not_send_state_changing_request(self):
        client = FakeClient()
        with self.assertRaises(TimeoutError):
            run_request(client, {"request_id": "id", "action": "contact"}, 0,
                        clock=lambda: client.now)
        self.assertEqual(client.sent, [])

    def test_shutdown_does_not_retry(self):
        client = FakeClient()
        client.running = False
        with self.assertRaises(InterruptedError):
            run_request(client, {"request_id": "id", "action": "contact"}, 2,
                        clock=lambda: client.now)
        self.assertEqual(client.sent, [])


if __name__ == "__main__":
    unittest.main()
