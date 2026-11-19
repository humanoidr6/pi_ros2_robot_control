"""Track asynchronous goals so a stop invalidates late goal acceptances."""
import threading


class GoalState:
    def __init__(self):
        self.lock = threading.RLock()
        self.epoch = 0
        self.pending = False
        self.handle = None
        self.stopped = True

    def arm_ticket(self):
        with self.lock:
            return self.epoch

    def armed(self, ticket):
        with self.lock:
            if ticket != self.epoch:
                return False
            self.stopped = False
            return True

    def reserve(self):
        with self.lock:
            if self.stopped:
                raise RuntimeError('Arm navigation before sending a goal')
            if self.pending or self.handle is not None:
                raise RuntimeError('Cancel the current goal before sending another')
            self.pending = True
            return self.epoch

    def accepted(self, ticket, handle):
        with self.lock:
            if ticket != self.epoch or self.stopped:
                return False
            self.pending = False
            self.handle = handle
            return True

    def rejected(self, ticket):
        with self.lock:
            if ticket == self.epoch:
                self.pending = False

    def completed(self, handle):
        with self.lock:
            if handle is not self.handle:
                return False
            self.handle = None
            self.stopped = True
            self.epoch += 1
            return True

    def stop(self):
        with self.lock:
            self.epoch += 1
            self.stopped = True
            self.pending = False
            handle, self.handle = self.handle, None
            return handle
