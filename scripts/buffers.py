import numpy as np

# basically just a numpy array
class Buffer(object):
    def __init__(self, dtype):
        self.dtype = dtype
        self.reset()

    def reset(self):
        self.buffer = np.array([], dtype=self.dtype)
        self.start_time = None

    def put(self, data, timestamp):
        if self.start_time is None:
            self.start_time = timestamp
        self.buffer = np.append(self.buffer, data)

    def get(self):
        return self.buffer

# Adds until full, no more
class BlockBuffer(object):
    def __init__(self, size, dtype):
        self.buffer = np.zeros(size, dtype=dtype)
        self.reset()

    def reset(self):
        self.index = 0
        self.start_time = None

    @property
    def is_full(self):
        return self.index == self.size

    @property
    def size(self):
        return self.buffer.size

    def put(self, data, timestamp):
        if self.start_time is None:
            self.start_time = timestamp

        available = self.size - self.index
        if data.size > available:
            self.buffer[self.index:] = data[:available]
            self.index += available
            return available
        else:
            self.buffer[self.index:(self.index + data.size)] = data
            self.index += data.size
        return data.size

    def get(self):
        if self.is_full:
            return self.buffer
        return self.buffer[:self.index]

# accepts past full
# says when it reaches full
class OtherBuffer(object):
    def __init__(self, size, dtype):
        self.size = size
        self.dtype = dtype
        self.hard_reset()

    def reset(self):
        self.index = 0
        self.start_time = None

    def hard_reset(self):
        self.reset()
        self.buffer = np.zeros(self.size, dtype=self.dtype)

    @property
    def is_full(self):
        return self.index >= self.size

    def put(self, data, timestamp):
        if self.start_time is None:
            self.start_time = timestamp

        available = self.size - self.index
        if data.size > available:
            self.buffer = np.append(self.buffer, data)
        else:
            self.buffer[self.index:(self.index + data.size)] = data
        self.index += data.size

    def get(self):
        if self.is_full:
            return self.buffer
        return self.buffer[:self.index]
