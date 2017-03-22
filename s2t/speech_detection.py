import numpy as np

from .py23 import Queue

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
    """

    def __init__(self, rate, threshold, dynamic_threshold=False,
                 dynamic_threshold_percentage=50,
                 dynamic_threshold_frame=3,
                 chunk_size=None, logger=None):
        self.rate = rate
        self.thr = threshold
        self.dyn_thr = dynamic_threshold
        self.dyn_thr_ratio = dynamic_threshold_percentage / 100.
        self.dyn_thr_frame = dynamic_threshold_frame
        if chunk_size is None:
            chunk_size = rate // 10
        self.chunk_size = chunk_size
        if logger is not None:
            self.log = logger
        # TODO improve
        self.avg_volume = None

    def log(self, msg):
        pass

    @property
    def threshold(self):
        """
        Constant threshold for static thresholding.
        For dynamic thresholding, (100+x)% louder than the avg volume.
        """
        if self.dyn_thr:
            return self.avg_volume * (1 + self.dyn_thr_ratio)
        else:
            return self.thr

    def is_silent(self, snd_data):
        """
        Returns 'True' if all the data is below the 'silent' threshold.
        """
        self.log(max(snd_data))
        return snd_data.max() < self.threshold

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

    def get_next_utter(self, stream, min_avg_volume, pub_screen):
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
        avg_volume = 0
        volume_queue = Queue(10)
        volume_sum = 0
        q_size = 0

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
            if not self.dyn_thr:
                chunks.append(snd_data)
                silent = self.is_silent(snd_data)
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
                if volume_queue.full():
                    out = volume_queue.get()
                    volume_sum -= out
                    q_size -= 1
                if not volume_queue.full() and peak_count == 0:
                    volume_queue.put(max(snd_data))
                    volume_sum += max(snd_data)
                    q_size += 1
                avg_volume = max(volume_sum / q_size, min_avg_volume)

                rospy.loginfo("[AVG_VOLUME] " + str(avg_volume))

                self.avg_volume = avg_volume
                silent = self.is_silent(snd_data)

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
                        start_frame = snd_data  # TODO remove
                        start_time = rospy.get_rostime()
                        snd_started = True
                        num_silent = 0
                    else:
                        peak_count += 1
                        chunks.append(snd_data)
                if snd_started and num_silent > 10:
                    rospy.logwarn('audio segmend completed')
                    chunks.append(snd_data)
                    end_frame = snd_data  # TODO remove
                    break

        stream.stop_stream()
        pub_screen.publish("Recognizing")
        end_time = rospy.get_rostime()
        r = normalize(np.hstack(chunks))
        if not self.dyn_thr:
            r = self.trim(r)
        r = add_silence(r, self.rate, 0.5)
        return r, start_time, end_time
