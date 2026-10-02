"""Replay the reported SDK state rejection without importing the official SDK."""
from contextlib import ExitStack,redirect_stderr,redirect_stdout
import io
import signal
import sys
from types import SimpleNamespace
import unittest
from unittest.mock import mock_open,patch

from navigation import sdk_worker
from navigation.configuration import load_config


class FakeClock:
    def __init__(self):
        self.now=100.
        self.on_sleep=None

    def monotonic(self):
        return self.now

    def sleep(self,seconds):
        self.now+=seconds
        if self.on_sleep:
            self.on_sleep()


class StateMachineSDK:
    """Model the documented standUp prerequisite, including zero-speed move."""
    def __init__(self,clock,standing=False,connected=True,stand_error=0,move_error=0,void=False):
        self.clock=clock
        self.standing=standing
        self.connected=connected
        self.stand_error=stand_error
        self.move_error=move_error
        self.void=void
        self.actions=[]

    def initRobot(self,*addresses):
        self.actions.append(('initRobot',self.clock.now,addresses))

    def checkConnect(self):
        return self.connected

    def standUp(self):
        self.actions.append(('standUp',self.clock.now,()))
        if self.stand_error:
            return self.stand_error
        self.standing=True
        return None if self.void else 0

    def move(self,*velocity):
        self.actions.append(('move',self.clock.now,velocity))
        if not self.standing:
            return 0x3007
        return self.move_error or (None if self.void else 0)


class SDKStartupTests(unittest.TestCase):
    def run_worker(self,sdk,stand_up=True,interrupt=False,interrupt_move=False):
        output,error=io.StringIO(),io.StringIO()
        signals={}
        config=load_config()
        argv=['sdk_worker','--config','unused.yaml','--ready-fd','7']
        if stand_up:
            argv.append('--stand-up')
        def stop_during_wait():
            if interrupt:
                signals[signal.SIGTERM](signal.SIGTERM,None)
        original_move=sdk.move
        def move_with_signal(*velocity):
            result=original_move(*velocity)
            if interrupt_move:
                signals[signal.SIGTERM](signal.SIGTERM,None)
            return result
        sdk.clock.on_sleep=stop_during_wait
        with ExitStack() as stack:
            stack.enter_context(patch.object(sdk,'move',side_effect=move_with_signal))
            stack.enter_context(patch.object(sys,'argv',argv))
            stack.enter_context(patch.object(sdk_worker,'load_config',return_value=config))
            stack.enter_context(patch.object(sdk_worker,'load_sdk',return_value=sdk))
            stack.enter_context(patch.object(sdk_worker,'open',mock_open(),create=True))
            stack.enter_context(patch.dict(sys.modules,{'fcntl':SimpleNamespace(
                LOCK_EX=1,LOCK_NB=2,flock=lambda *args:None)}))
            stack.enter_context(patch.object(sdk_worker.os,'name','posix'))
            stack.enter_context(patch.object(sdk_worker.signal,'signal',
                side_effect=lambda sig,handler:signals.__setitem__(sig,handler)))
            stack.enter_context(patch.object(sdk_worker.time,'monotonic',sdk.clock.monotonic))
            stack.enter_context(patch.object(sdk_worker.time,'sleep',sdk.clock.sleep))
            stack.enter_context(patch.object(sys,'stdin',SimpleNamespace(fileno=lambda:0)))
            stack.enter_context(patch.object(sdk_worker.os,'set_blocking'))
            stack.enter_context(patch.object(sdk_worker.select,'select',return_value=([0],[],[])))
            stack.enter_context(patch.object(sdk_worker.os,'read',return_value=b''))
            ready=stack.enter_context(patch.object(sdk_worker.os,'write'))
            stack.enter_context(patch.object(sdk_worker.os,'close'))
            stack.enter_context(redirect_stdout(output))
            stack.enter_context(redirect_stderr(error))
            code=sdk_worker.main()
        # Startup/EOF/cleanup must never send translational or turning motion.
        self.assertTrue(all(args==(0.,0.,0.) for name,_,args in sdk.actions if name=='move'))
        return code,ready,output.getvalue(),error.getvalue()

    def test_reported_state_rejection_is_avoided_before_ready(self):
        sdk=StateMachineSDK(FakeClock())
        code,ready,output,error=self.run_worker(sdk)
        self.assertEqual(code,0,error)
        self.assertEqual([a[0] for a in sdk.actions[:3]],['initRobot','standUp','move'])
        self.assertGreaterEqual(sdk.actions[2][1]-sdk.actions[1][1],4.)
        ready.assert_called_once_with(7,b'READY')
        self.assertIn('TASK2_READY',output)

    def test_already_standing_without_flag_does_not_request_standup(self):
        sdk=StateMachineSDK(FakeClock(),standing=True)
        code,ready,_,error=self.run_worker(sdk,stand_up=False)
        self.assertEqual(code,0,error)
        self.assertNotIn('standUp',[a[0] for a in sdk.actions])
        ready.assert_called_once_with(7,b'READY')

    def test_missing_standup_flag_rejects_startup_with_action_and_error_code(self):
        sdk=StateMachineSDK(FakeClock())
        code,ready,_,error=self.run_worker(sdk,stand_up=False)
        self.assertEqual(code,2)
        self.assertNotIn('standUp',[a[0] for a in sdk.actions])
        ready.assert_not_called()
        self.assertIn('move(0,0,0) startup',error)
        self.assertIn('12295 (0x3007)',error)

    def test_rejected_standup_never_announces_ready(self):
        sdk=StateMachineSDK(FakeClock(),stand_error=0x3007)
        code,ready,_,error=self.run_worker(sdk)
        self.assertEqual(code,2)
        self.assertIn('standUp: SDK returned',error)
        ready.assert_not_called()

    def test_rejected_zero_command_after_standup_is_not_ignored(self):
        sdk=StateMachineSDK(FakeClock(),move_error=0x3007)
        code,ready,_,error=self.run_worker(sdk)
        self.assertEqual(code,2)
        self.assertIn('move(0,0,0) startup',error)
        ready.assert_not_called()

    def test_signal_during_standup_wait_never_announces_ready(self):
        sdk=StateMachineSDK(FakeClock())
        code,ready,_,error=self.run_worker(sdk,interrupt=True)
        self.assertEqual(code,0,error)
        ready.assert_not_called()

    def test_connection_timeout_does_not_request_standup(self):
        sdk=StateMachineSDK(FakeClock(),connected=False)
        code,ready,_,error=self.run_worker(sdk)
        self.assertEqual(code,2)
        self.assertIn('SDK connection timeout',error)
        self.assertNotIn('standUp',[a[0] for a in sdk.actions])
        ready.assert_not_called()

    def test_signal_during_initial_zero_command_never_announces_ready(self):
        sdk=StateMachineSDK(FakeClock())
        code,ready,_,error=self.run_worker(sdk,interrupt_move=True)
        self.assertEqual(code,0,error)
        ready.assert_not_called()

    def test_void_sdk_binding_remains_supported(self):
        sdk=StateMachineSDK(FakeClock(),void=True)
        code,ready,_,error=self.run_worker(sdk)
        self.assertEqual(code,0,error)
        ready.assert_called_once_with(7,b'READY')


if __name__=='__main__':
    unittest.main()
