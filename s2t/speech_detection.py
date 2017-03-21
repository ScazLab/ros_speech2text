from sys import byteorder
from array import array
from builtins import range  # For py3 compatibility

from .py23 import Queue

import rospy

# TODO:
#   - abstract rospy
#   - switch from array to numpy
#   - stream as constructor argument (or make a new class (generator?))


NORMAL_MAXIMUM = 16384


def normalize(snd_data):
    """
    Average the volume out
    """
    times = float(NORMAL_MAXIMUM) / max(abs(i) for i in snd_data)

    r = array('h')
    for i in snd_data:
        r.append(int(i * times))
    return r


def add_silence(snd_data, rate, seconds):
    """
    Add silence to the start and end of 'snd_data' of length 'seconds' (float)
    This prevents some players from skipping the first few frames.
    """
    n_zeros = int(seconds * rate)
    r = array('h', [0 for i in range(n_zeros)])
    r.extend(snd_data)
    r.extend([0 for i in range(n_zeros)])
    return r


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
        return max(snd_data) < self.threshold

    def trim(self, start, end, snd_data):
        """
        This function is not used in dynamic thresholding.
        Trim the blank spots at the start and end.
        """
        def _trim(snd_data):
            snd_started = False
            r = array('h')
            for i in snd_data:
                if not snd_started and abs(i) > self.thr:
                    snd_started = True
                    r.append(i)
                elif snd_started:
                    r.append(i)
            return r

        # Trim to the left
        snd_data = _trim(snd_data)
        # Trim to the right
        snd_data.reverse()
        snd_data = _trim(snd_data)
        snd_data.reverse()
        return snd_data

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
        stream.start_stream()  # TODO: should not happen here
        r = array('h')
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
            # little endian, signed short
            snd_data = array('h', stream.read(self.chunk_size))
            if byteorder == 'big':
                snd_data.byteswap()

            # Static thresholding
            if not self.dyn_thr:
                r.extend(snd_data)
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
                    r.extend(snd_data)
                    num_silent += 1
                elif not silent and snd_started:
                    r.extend(snd_data)
                elif silent and not snd_started:
                    peak_count = 0
                    r = array('h')
                elif not silent and not snd_started:
                    if peak_count >= self.dyn_thr_frame:
                        rospy.logwarn('collecting audio segment')
                        r.extend(snd_data)
                        start_frame = snd_data  # TODO remove
                        start_time = rospy.get_rostime()
                        snd_started = True
                        num_silent = 0
                    else:
                        peak_count += 1
                        r.extend(snd_data)
                if snd_started and num_silent > 10:
                    rospy.logwarn('audio segmend completed')
                    r.extend(snd_data)
                    end_frame = snd_data  # TODO remove
                    break

        stream.stop_stream()
        pub_screen.publish("Recognizing")
        end_time = rospy.get_rostime()
        r = normalize(r)
        if not self.dyn_thr:
            r = self.trim(r)
        r = add_silence(r, self.rate, 0.5)
        return r, start_time, end_time
