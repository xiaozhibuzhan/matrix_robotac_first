"""Exercise real SIGINT delivery without changing native crash exit codes."""

from pathlib import Path
import os
import shutil
import signal
import subprocess
import tempfile
import unittest

from task3_sim.__main__ import defer_cleanup_sigint


class CleanupInterruptTests(unittest.TestCase):
    def test_repeated_sigint_does_not_abandon_cleanup_and_handler_is_restored(self):
        previous = signal.getsignal(signal.SIGINT)
        completed = []
        with defer_cleanup_sigint() as received:
            signal.raise_signal(signal.SIGINT)
            completed.append("viewer")
            signal.raise_signal(signal.SIGINT)
            completed.append("renderer")
        self.assertEqual(completed, ["viewer", "renderer"])
        self.assertEqual(received, [signal.SIGINT, signal.SIGINT])
        self.assertEqual(signal.getsignal(signal.SIGINT), previous)

    def test_non_interrupt_failures_and_system_exit_are_not_suppressed(self):
        previous = signal.getsignal(signal.SIGINT)
        for failure in (RuntimeError("cleanup failure"), SystemExit(17)):
            with self.subTest(failure=failure):
                with self.assertRaises(type(failure)):
                    with defer_cleanup_sigint():
                        signal.raise_signal(signal.SIGINT)
                        raise failure
                self.assertEqual(signal.getsignal(signal.SIGINT), previous)


@unittest.skipUnless(os.name == "posix" and shutil.which("bash"),
                     "foreground process-group SIGINT requires POSIX bash")
class LauncherSignalTests(unittest.TestCase):
    """Bash/tee integration; the child stands in for the task's signal policy.

    This targets the logging pipeline, not MuJoCo. POSIX setsid/killpg gives
    the same SIGINT to bash, the task child, and tee as a terminal Ctrl+C.
    """

    def run_stub(self, final_code, interrupt=False):
        launcher = Path(__file__).resolve().parents[1] / "run_task3.sh"
        with tempfile.TemporaryDirectory(prefix="task3 signals ") as temporary:
            root = Path(temporary)
            package = root / "task3_sim"
            python = package / ".venv/bin/python"
            python.parent.mkdir(parents=True)
            shutil.copyfile(launcher, package / "run_task3.sh")
            python.write_text(
                "#!/usr/bin/env bash\n"
                "trap 'printf \"FINAL_AFTER_INTERRUPT\\n\"; exit \"$TASK3_STUB_CODE\"' INT\n"
                "printf 'STUB_READY\\n'\n"
                "if [[ $TASK3_STUB_INTERRUPT == 1 ]]; then\n"
                "  while :; do sleep 0.02; done\n"
                "fi\n"
                "exit \"$TASK3_STUB_CODE\"\n", encoding="utf-8")
            python.chmod(0o755)
            output = root / "result with spaces"
            env = dict(os.environ, TASK3_STUB_CODE=str(final_code),
                       TASK3_STUB_INTERRUPT="1" if interrupt else "0")
            process = subprocess.Popen(
                ["bash", str(package / "run_task3.sh"), "--output", str(output)],
                stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True,
                env=env, start_new_session=True)
            try:
                if interrupt:
                    import time
                    deadline = time.monotonic()+5
                    terminal = output / "terminal.log"
                    while not terminal.exists() or "STUB_READY" not in terminal.read_text():
                        if process.poll() is not None or time.monotonic() >= deadline:
                            self.fail("stub did not reach interrupt-ready state")
                        time.sleep(0.01)
                    os.killpg(process.pid, signal.SIGINT)
                captured, _ = process.communicate(timeout=5)
            finally:
                if process.poll() is None:
                    os.killpg(process.pid, signal.SIGKILL)
                    process.communicate()
            log = (output / "terminal.log").read_text()
            self.assertEqual(process.returncode, final_code, captured)
            self.assertIn(f"TASK3_PROCESS_EXIT code={final_code}", log)
            if interrupt:
                self.assertIn("FINAL_AFTER_INTERRUPT", captured)
                self.assertIn("FINAL_AFTER_INTERRUPT", log)

    def test_completed_result_sigint_still_logs_and_exits_zero(self):
        self.run_stub(0, interrupt=True)

    def test_before_result_sigint_keeps_cancellation_nonzero(self):
        self.run_stub(2, interrupt=True)

    def test_native_abort_and_segmentation_statuses_are_preserved(self):
        for code in (134, 139):
            with self.subTest(code=code):
                self.run_stub(code)


if __name__ == "__main__":
    unittest.main()
