import numpy as np

from collections import deque

import rospy

# TODO:
#   - stream as constructor argument (or make a new class (generator?))


NORMAL_MAXIMUM = 16384
BUFFER_NP_TYPE = '<i2'  # little endian, signed short


def normalize(snd_data):
    """Average the volume out
    """
    r = snd_data * NORMAL_MAXIMUM / np.abs(snd_data).max()
    return r.astype(snd_data.dtype)


def add_silence(snd_data, rate, seconds):
    """
    Add silence to the start and end of 'snd_data' of length 'seconds' (float)
    This prevents some players from skipping the first few frames.
    """
    zeros = np.zeros((int(seconds * rate),), dtype=snd_data.dtype)
    return np.hstack([zeros, snd_data, zeros])


class SilenceDetector(object):
    """Should implement a threshold property.
    """

    def is_silent(self, snd_data):
        """
        Returns 'True' if all the data is below the 'silent' threshold.
        """
        rospy.loginfo(max(snd_data))
        return snd_data.max() < self.threshold

    def trim(self, start, end, snd_data):
        """
        This function is not used in dynamic thresholding.
        Trim the blank spots at the start and end.
        """
        raise NotImplementedError

    def reset_average(self):
        pass

    def update_average(self, chunk):
        pass


class StaticSilenceDetector(SilenceDetector):

    is_static = True

    def __init__(self, rate, threshold):
        self.rate = rate
        self.threshold = threshold

    def trim(self, start, end, snd_data):
        """
        This function is not used in dynamic thresholding.
        Trim the blank spots at the start and end.
        """
        non_silent = (np.abs(snd_data) <= self.thr).nonzero()[0]
        if len(non_silent) == 0:
            return snd_data[0:0]  # Empty array
        else:
            return snd_data[non_silent[0]:non_silent[-1]]


class DynamicSilenceDetector(SilenceDetector):

    is_static = False

    def __init__(self, rate, dynamic_threshold_percentage=50,
                 min_average_volume=1., n_average=10):
        self.rate = rate
        self.dyn_thr_ratio = dynamic_threshold_percentage / 100.
        self.min_avg = min_average_volume
        self._vol_q = deque([], maxlen=n_average)
        self.reset_average()

    @property
    def average_volume(self):
        if len(self._vol_q) == 0:
            return self.min_avg
        else:
            return max(self.min_avg, sum(self._vol_q) * 1. / len(self._vol_q))

    def reset_average(self):
        self._vol_q.clear()
        # Note: storing the sum and incrementing/decrementing on update
        # would be more efficient but this is simpler for now.

    def update_average(self, chunk):
        self._vol_q.append(max(chunk))

    @property
    def threshold(self):
        """(100+x)% louder than the avg volume."""
        return self.average_volume * (1 + self.dyn_thr_ratio)


class SpeechDetector:
    """
    Dynamic thresholding:
        Before audio is being considered part of a sentence, peak_count
        is used to count how many consecutive frames have been above the
        dynamic threshold volume. Once peak_count is over the specified
        frame number from ros param, we consider the sentence started and
        lock the value of avg volume to maintain the standard thresholding
        standard throughout the sentence. Whenever receiving a frame that
        has volume that is too low, we increase num_silent. When num_silent
        exceeds ten, we consider the sentence finished.

    :param threshold: float
        Static or dynamic threshold. Interpreted as a percentage when dynamic.
    """

    def __init__(self, rate, threshold, dynamic_threshold=False,
                 dynamic_threshold_frame=3, chunk_size=None,
                 min_average_volume=0.):
        if dynamic_threshold:
            self.silence_detect = DynamicSilenceDetector(
                rate, threshold, min_average_volume=min_average_volume)
        else:
            self.silence_detect = StaticSilenceDetector(rate, threshold)
        if chunk_size is None:
            chunk_size = rate // 10
        self.chunk_size = chunk_size
        self.dyn_thr_frame = dynamic_threshold_frame

    def get_next_utter(self, stream, pub_screen):
        """
        Main function for capturing audio.
        Parameters:
            stream: our pyaudio client
            min_avg_volume: helps thresholding in quiet environments
            pub_screen: publishes status messages to baxter screen
        """
        num_silent = 0
        snd_started = False
        stream.start_stream()  # TODO: Why not record during recognition
        chunks = []
        peak_count = 0

        while True:
            """
            main loop for audio capturing
            """
            if snd_started:
                pub_screen.publish("Sentence Started")
            if rospy.is_shutdown():
                return None, None, None
            snd_data = np.frombuffer(
                stream.read(self.chunk_size, exception_on_overflow=False),
                dtype=BUFFER_NP_TYPE)

            # Static thresholding
            if self.silence_detect.is_static:
                chunks.append(snd_data)
                silent = self.silence_detect.is_silent(snd_data)
                if silent and snd_started:
                    num_silent += 1
                elif not silent and not snd_started:
                    rospy.logwarn('collecting audio segment')
                    snd_started = True
                    start_time = rospy.get_rostime()
                    num_silent = 0
                if snd_started and num_silent > 10:
                    rospy.logwarn('audio segment completed')
                    break

            else:  # Dynamic thresholding
                # Calculate an average volume with a queue of ten previous frames
                self.silence_detect.update_average(snd_data)

                rospy.loginfo("[AVG_VOLUME] " +
                              str(self.silence_detect.average_volume))

                silent = self.silence_detect.is_silent(snd_data)

                if silent and snd_started:
                    chunks.append(snd_data)
                    num_silent += 1
                elif not silent and snd_started:
                    chunks.append(snd_data)
                elif silent and not snd_started:
                    peak_count = 0
                    chunks = []
                elif not silent and not snd_started:
                    if peak_count >= self.dyn_thr_frame:
                        rospy.logwarn('collecting audio segment')
                        chunks.append(snd_data)
                        start_time = rospy.get_rostime()
                        snd_started = True
                        num_silent = 0
                    else:
                        peak_count += 1
                        chunks.append(snd_data)
                if snd_started and num_silent > 10:
                    rospy.logwarn('audio segmend completed')
                    chunks.append(snd_data)
                    break

        stream.stop_stream()
        pub_screen.publish("Recognizing")
        end_time = rospy.get_rostime()
        r = normalize(np.hstack(chunks))
        if self.silence_detect.is_static:
            r = self.silence_detect.trim(r)
        r = add_silence(r, self.rate, 0.5)
        return r, start_time, end_time
