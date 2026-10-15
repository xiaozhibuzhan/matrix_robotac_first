"""Human-operated treasure commands for ROS 2 Humble / Python 3.10.

Run from the project root: python3 -m navigation.treasure_cli --help.
ROS imports are deferred so the argument/protocol tests need no ROS install.
"""
from __future__ import annotations

import argparse
import json
import math
import sys
import time
import uuid
from typing import Any, Callable, Protocol, Sequence

COMMAND_TOPIC = "/task2/treasure/command"
ACK_TOPIC = "/task2/treasure/ack"
STATUS_TOPIC = "/task2/treasure/status"
DEFAULT_TIMEOUT = 15.0
RETRY_INTERVAL = 1.0


def finite_number(value: str) -> float:
    try:
        number = float(value)
    except ValueError as exc:
        raise argparse.ArgumentTypeError("must be a finite number") from exc
    if not math.isfinite(number):
        raise argparse.ArgumentTypeError("must be a finite number")
    return number


def nonnegative_timeout(value: str) -> float:
    number = finite_number(value)
    if number < 0:
        raise argparse.ArgumentTypeError("timeout must be nonnegative")
    return number


def nonempty_id(value: str) -> str:
    if not value.strip():
        raise argparse.ArgumentTypeError("identifier must not be empty")
    return value.strip()


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Operate the external treasure controller; no official programs are launched."
    )
    parser.add_argument(
        "--timeout", type=nonnegative_timeout, default=DEFAULT_TIMEOUT,
        help="total discovery/response timeout in seconds (default: 15)",
    )
    commands = parser.add_subparsers(dest="action", required=True)
    helps = {
        "status": "read current durable status without changing it",
        "new-round": "start a new round after resetting the official simulation",
        "resume": "resume a saved round by its round ID",
        "select": "select a box, then click its center and approach point in RViz",
        "contact": "start bounded contact after arriving and stopping at the approach point",
        "confirm": "record a box only after personally observing its disappearance",
        "retry": "retry the current box according to the controller state",
        "cancel": "stop and cancel the current action",
        "skip": "skip the current box without recording a successful trigger",
        "finish": "finish the current round and stop",
    }
    for action, help_text in helps.items():
        command = commands.add_parser(action, help=help_text, description=help_text)
        # A suppressed default preserves --timeout supplied before the action.
        command.add_argument(
            "--timeout", type=nonnegative_timeout, default=argparse.SUPPRESS,
            help="total discovery/response timeout in seconds",
        )
        if action == "resume":
            command.add_argument("round_id", type=nonempty_id)
        elif action == "select":
            command.add_argument("box_id", type=nonempty_id)
            command.add_argument(
                "--radius", type=finite_number, required=True,
                help="observed box footprint radius in metres; controller validates limits",
            )
        elif action == "confirm":
            command.add_argument(
                "--disappeared", action="store_true", required=True,
                help="explicitly confirm you saw this box disappear in the simulator",
            )
    return parser


def parse_request(
    argv: Sequence[str] | None = None, *, request_id: str | None = None,
) -> tuple[argparse.Namespace, dict[str, Any]]:
    args = build_parser().parse_args(argv)
    request: dict[str, Any] = {
        "request_id": request_id or str(uuid.uuid4()), "action": args.action,
    }
    if args.action == "resume":
        request["round_id"] = args.round_id
    elif args.action == "select":
        request.update(box_id=args.box_id, radius=args.radius)
    elif args.action == "confirm":
        request["disappeared"] = True
    return args, request


def json_object(data: str) -> dict[str, Any] | None:
    try:
        value = json.loads(data)
    except (ValueError, TypeError):
        return None
    return value if isinstance(value, dict) and value else None


def matching_ack(data: str, request_id: str) -> dict[str, Any] | None:
    value = json_object(data)
    if (
        value is None
        or value.get("request_id") != request_id
        or type(value.get("success")) is not bool
        or not isinstance(value.get("message"), str)
        or ("status" in value and not isinstance(value["status"], dict))
    ):
        return None
    return value


class Client(Protocol):
    ack: dict[str, Any] | None
    status: dict[str, Any] | None

    def is_ok(self) -> bool: ...
    def command_ready(self) -> bool: ...
    def publish_request(self, request: dict[str, Any]) -> None: ...
    def spin_once(self, timeout_sec: float) -> None: ...


def run_request(
    client: Client, request: dict[str, Any], timeout: float,
    *, clock: Callable[[], float] = time.monotonic,
) -> dict[str, Any]:
    """Use one discovery/response budget; retransmit the same ID for deduping.

    Status only reads durable state. The controller owns command idempotency.
    """
    if not math.isfinite(timeout) or timeout < 0:
        raise ValueError("timeout must be finite and nonnegative")
    deadline = clock() + timeout
    next_send = clock()
    sent = False
    reading_status = request["action"] == "status"
    while client.is_ok():
        if reading_status and client.status is not None:
            return client.status
        if not reading_status and client.ack is not None:
            return client.ack
        now = clock()
        if now >= deadline:
            break
        if not reading_status and now >= next_send and client.command_ready():
            client.publish_request(request)
            sent = True
            next_send = now + RETRY_INTERVAL
        client.spin_once(timeout_sec=min(0.1, max(0.0, deadline - clock())))
    if not client.is_ok():
        raise InterruptedError("ROS shutdown interrupted the treasure command")
    if sent:
        raise TimeoutError(
            "No acknowledgement before timeout; command outcome is unknown. "
            "Read status before issuing another state-changing command. "
            f"request_id={request['request_id']}"
        )
    if reading_status:
        raise TimeoutError(
            "No valid treasure status received before timeout. Start navigation "
            "with --treasure and use the same ROS_DOMAIN_ID and ROS environment."
        )
    raise TimeoutError(
        "No ready treasure controller discovered before timeout; no command was sent. "
        "Start navigation with --treasure and use the same ROS_DOMAIN_ID and ROS environment."
    )


class RosClient:
    """Small rclpy adapter; never starts navigation, SDK, or the simulator."""

    def __init__(self, request_id: str) -> None:
        import rclpy
        from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
        from std_msgs.msg import String

        self.rclpy = rclpy
        self.string_type = String
        self.request_id = request_id
        self.ack: dict[str, Any] | None = None
        self.status: dict[str, Any] | None = None
        self.node = None
        rclpy.init(args=[])
        try:
            self.node = rclpy.create_node("task2_treasure_cli_" + uuid.uuid4().hex[:12])
            reliable = QoSProfile(
                depth=10, reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.VOLATILE,
            )
            durable = QoSProfile(
                depth=1, reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            )
            # Subscribe first, then wait for both ends before sending a command.
            self.ack_subscription = self.node.create_subscription(
                String, ACK_TOPIC, self._on_ack, reliable,
            )
            self.status_subscription = self.node.create_subscription(
                String, STATUS_TOPIC, self._on_status, durable,
            )
            self.publisher = self.node.create_publisher(String, COMMAND_TOPIC, reliable)
        except Exception:
            self.close()
            raise

    def _on_ack(self, message: Any) -> None:
        acknowledgement = matching_ack(message.data, self.request_id)
        if acknowledgement is not None:
            self.ack = acknowledgement

    def _on_status(self, message: Any) -> None:
        status = json_object(message.data)
        if status is not None:
            self.status = status

    def is_ok(self) -> bool:
        return bool(self.rclpy.ok())

    def command_ready(self) -> bool:
        return (
            self.publisher.get_subscription_count() > 0
            # Humble Subscription has no get_publisher_count(). Query the
            # node graph for the ACK topic while keeping both discovery gates.
            and self.node.count_publishers(ACK_TOPIC) > 0
        )

    def publish_request(self, request: dict[str, Any]) -> None:
        message = self.string_type()
        message.data = json.dumps(request, ensure_ascii=False, allow_nan=False)
        self.publisher.publish(message)

    def spin_once(self, timeout_sec: float) -> None:
        self.rclpy.spin_once(self.node, timeout_sec=timeout_sec)

    def close(self) -> None:
        if self.node is not None:
            self.node.destroy_node()
            self.node = None
        if self.rclpy.ok():
            self.rclpy.shutdown()


def main(argv: Sequence[str] | None = None) -> int:
    args, request = parse_request(argv)
    client = None
    try:
        client = RosClient(request["request_id"])
        response = run_request(client, request, args.timeout)
        print(json.dumps(response, ensure_ascii=False, indent=2, allow_nan=False))
        return 0 if args.action == "status" or response["success"] else 3
    except ImportError as exc:
        print(
            "ROS 2 Python modules are unavailable. On Ubuntu, run "
            "'source /opt/ros/humble/setup.bash' and use system python3. "
            f"Details: {exc}", file=sys.stderr,
        )
        return 2
    except TimeoutError as exc:
        print(str(exc), file=sys.stderr)
        return 4
    except (KeyboardInterrupt, InterruptedError):
        print(
            "Treasure CLI interrupted. This does not cancel an accepted robot action; "
            "use the cancel command or the navigation emergency-stop procedure.",
            file=sys.stderr,
        )
        return 130
    except Exception as exc:
        print(f"Treasure command failed: {exc}", file=sys.stderr)
        return 1
    finally:
        if client is not None:
            client.close()


if __name__ == "__main__":
    raise SystemExit(main())
