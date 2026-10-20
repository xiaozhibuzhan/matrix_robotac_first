"""Opt-in official native physics entry; never falls back to kinematics."""
from __future__ import annotations

import argparse
from datetime import datetime
import json
from pathlib import Path
import sys
import uuid

from .__main__ import PACKAGE, PROJECT, parse_args, write_json


def main(argv=None):
    arguments = list(sys.argv[1:] if argv is None else argv)
    preflight = "--preflight" in arguments
    arguments = [arg for arg in arguments if arg != "--preflight"]
    args = parse_args(["--config", str(PACKAGE/"config/multi_official.json"),
                       "--max-seconds", "240", *arguments])
    config = json.loads(args.config.read_text(encoding="utf-8-sig"))
    if config.get("backend") != "official_dynamics":
        raise ValueError("Official entry requires backend=official_dynamics; kinematic fallback is forbidden")
    if not preflight:
        from .multi import run_multi
        return run_multi(args)[0]
    # Preflight compiles a task-owned scene, but never starts physics or SDK.
    from .multi_scene import build_multi_scene
    from .official_scene import prepare_scene
    from .official_runtime import OfficialRuntime
    output = (args.output or PACKAGE/"runs"/f"official_preflight_{datetime.now():%Y%m%d_%H%M%S}_{uuid.uuid4().hex[:8]}").resolve()
    if output.exists() and any(output.iterdir()):
        raise ValueError("Preflight output must be new or empty")
    output.mkdir(parents=True, exist_ok=True)
    build_multi_scene(config, args.model_root, output/"generated")
    _, scene = prepare_scene(output/"generated/scene.xml", config, args.model_root)
    settings = config.get("official_dynamics", {})
    runtime = OfficialRuntime(PROJECT, output/"official_runtime", scene,
                              sdk_port=int(settings.get("sdk_port", 44888)))
    report = runtime.preflight(PACKAGE/settings.get("bridge_executable", "local/native/task3_official_state_bridge"))
    report["scope"] = "dependency/configuration preflight only; no official gait or SDK commands executed"
    write_json(output/"preflight.json", report)
    print(json.dumps(report, ensure_ascii=False, indent=2))
    print(f"TASK3_OFFICIAL_PREFLIGHT report={output/'preflight.json'}")
    return 0 if report["ready"] else 2


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except Exception as exc:
        print(f"Task3 official startup error: {exc}", file=sys.stderr)
        raise SystemExit(2)
