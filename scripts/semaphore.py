from threading import Lock
from time import sleep

class Semaphore(object):
    def __init__(self):
        self.index = None
        self.lock = Lock()

    def init_index(self, n):
        if self.index is None:
            with self.lock:
                if self.index is None:
                    self.index = n

    def enter(self, n):
        self.init_index(n)
        while (self.index < n):
            sleep(0.001)
        return self.index == n

    def exit(self):
        self.index += 1
