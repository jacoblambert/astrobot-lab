"""Service grant-only ROS callbacks independently of simulation-thread I/O.

The injected executor must own only a grant-receiver node. It never steps
physics, issues commands, advances a timestamp, or grants authority itself.
"""

import threading


class LeaseCallbacks:
    def __init__(self, executor, on_failure):
        self.executor = executor
        self.on_failure = on_failure
        self.closed = threading.Event()
        self.thread = threading.Thread(target=self.run, name="capture_lease_receiver", daemon=True)
        self.thread.start()

    def run(self):
        while not self.closed.is_set():
            try:
                self.executor.spin_once(timeout_sec=.01)
            except Exception as exc:
                if not self.closed.is_set():
                    self.on_failure(str(exc))
                return

    def close(self):
        self.closed.set()
        self.executor.wake()
        self.thread.join(timeout=1.)
        if self.thread.is_alive():
            raise RuntimeError("grant callback executor failed to stop")
