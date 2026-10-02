"""Single, latest-request planning worker; ROS callbacks never wait on A*."""
import threading


class AsyncPlanner:
    def __init__(self, plan):
        self._plan=plan; self._condition=threading.Condition()
        self._generation=0; self._pending=None; self._result=None; self._closed=False
        # The worker owns no ROS/SDK resources. A bounded in-flight A* may finish
        # after close(), but cannot publish a result or delay process shutdown.
        self._thread=threading.Thread(target=self._run,name='task2-planner',daemon=True)
        self._thread.start()

    def submit(self, start, goal):
        with self._condition:
            if self._closed: raise RuntimeError('Planner is closed')
            self._generation+=1; token=self._generation
            # Keep at most the latest waiting request, plus the one executing.
            self._pending=(token,tuple(start),tuple(goal)); self._result=None
            self._condition.notify()
            return token

    def poll(self, token):
        with self._condition:
            if self._closed or token!=self._generation: return None
            result=self._result; self._result=None
            return result

    def discard(self, token):
        with self._condition:
            if token==self._generation:
                self._generation+=1; self._pending=None; self._result=None

    def close(self):
        with self._condition:
            self._closed=True; self._generation+=1
            self._pending=None; self._result=None; self._condition.notify()

    def _run(self):
        while True:
            with self._condition:
                self._condition.wait_for(lambda:self._closed or self._pending is not None)
                if self._closed: return
                token,start,goal=self._pending; self._pending=None
            try:
                result=(self._plan(start,goal),None)
            except Exception as exc:
                result=(None,exc)
            with self._condition:
                if not self._closed and token==self._generation:
                    self._result=result
