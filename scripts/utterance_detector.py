import numpy as np
from buffers import Buffer, BlockBuffer, OtherBuffer

# durations given in milliseconds
class UtteranceDetector(object):
    def __init__(self, sample_rate, num_channels, dtype,
                block_duration, leading_noise_duration, trailing_silence_duration,
                threshold_pct, verbose, callback):

        self.rate = sample_rate
        self.num_channels = num_channels
        self.verbose = verbose
        self.callback = callback

        self.block = BlockBuffer(self.time2length(block_duration / 1000.), dtype)
        self.noise = OtherBuffer(self.time2length(leading_noise_duration / 1000.), dtype)
        self.silence = OtherBuffer(self.time2length(trailing_silence_duration / 1000.), dtype)
        self.utterance = Buffer(dtype)

        if np.issubdtype(dtype, int):
            self.max = np.iinfo(dtype).max
        elif np.issubdtype(dtype, float):
            self.max = np.finfo(dtype).max
        self.threshold = self.max * (threshold_pct / 100.)

        self.in_utterance = False

    @property
    def is_utterance_starting(self):
        return (not self.in_utterance) and self.noise.is_full

    @property
    def is_utterance_ending(self):
        return self.in_utterance and self.silence.is_full

    def is_silent(self, data):
        if self.verbose:
            print round(np.abs(data).max() / float(self.max), 4)
        return np.abs(data).max() < self.threshold

    def length2time(self, length):
        return length / float(self.num_channels) / self.rate

    def time2length(self, time):
        return int(self.rate * time) * self.num_channels

    def put_audio_chunk(self, audio_chunk, timestamp):
        i = 0
        while (i < audio_chunk.size):
            added = self.block.put(audio_chunk[i:], timestamp)
            i += added

            if self.block.is_full:
                self.process_block()
                self.block.reset()

    def process_block(self):
        silent = self.is_silent(self.block.get())
        t = self.block.start_time

        if silent and self.in_utterance:
            self.silence.put(self.block.get(), t)
            self.noise.reset()
        if not silent and not self.in_utterance:
            self.noise.put(self.block.get(), t)
            self.silence.reset()
        if not silent and self.in_utterance:
            self.utterance.put(self.silence.get(), t)
            self.silence.reset()
            self.utterance.put(self.block.get(), t)

        if self.is_utterance_starting:
            self.in_utterance = True
            # This will set the utterance's start time
            self.callback.on_utterance_started(self.noise.start_time)
            self.utterance.put(self.noise.get(), self.noise.start_time)
            self.callback.on_utterance_chunk(self.noise.get(), self.noise.start_time)

        elif self.is_utterance_ending:
            self.in_utterance = False
            duration = self.length2time(self.utterance.get().size)
            self.callback.on_utterance_completed(self.utterance.get(), self.utterance.start_time, duration)

            self.utterance.reset()
            self.silence.hard_reset()

class UtteranceDetectorCallback(object):
    def __init__(self):
        pass

    def on_utterance_started(self, timestamp):
        pass

    def on_utterance_completed(self, utterance, start_time, duration):
        pass

    def on_utterance_chunk(self, chunk, start_time):
        pass
