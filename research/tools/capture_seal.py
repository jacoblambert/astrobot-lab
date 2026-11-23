"""Post-motion file finalization without blocking real simulation clock ticks.

No motion authority, clock publication, deadline extension or success marker is
provided here. The caller must stop acquisition/commands before starting work.
"""

import threading
import time


class BackgroundSeal:
    def __init__(self, work, *, deadline_s=20.0):
        if not 0 < deadline_s <= 20:
            raise ValueError("bounded post-motion sealing deadline required")
        self.started = time.monotonic()
        self.deadline_s = deadline_s
        self.finished = threading.Event()
        self.error = None
        self.thread = threading.Thread(target=self._run, args=(work,), name="native_archive_seal", daemon=True)
        self.thread.start()

    def _run(self, work):
        try:
            work()
        except BaseException as exc:
            self.error = exc
        finally:
            self.finished.set()

    def poll(self):
        if self.error is not None:
            raise RuntimeError("post-motion archive sealing failed") from self.error
        if time.monotonic() - self.started > self.deadline_s:
            raise TimeoutError("post-motion archive sealing deadline exceeded")
        return self.finished.is_set()
