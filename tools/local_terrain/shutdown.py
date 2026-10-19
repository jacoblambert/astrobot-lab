"""Finish the active callback before honoring a normal launcher SIGINT."""

import signal


class DeferredInterrupt:
    def __init__(self):
        self.requested = False

    def __enter__(self):
        self.previous = signal.signal(signal.SIGINT, self.receive)
        return self

    def receive(self, signum, frame):
        self.requested = True

    def __exit__(self, *exception):
        signal.signal(signal.SIGINT, self.previous)
