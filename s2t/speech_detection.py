from sys import byteorder
from array import array
from Queue import Queue

import rospy


RATE = None
THRESHOLD = None
DYNAMIC_THRESHOLD = None
DYNAMIC_THRESHOLD_Percentage = None
DYNAMIC_THRESHOLD_Frame = None
CHUNK_SIZE = None


def is_silent(snd_data):
    """
    This is for static thresholding.
    Returns 'True' if below the 'silent' threshold.
    """
    rospy.loginfo(max(snd_data))
    return max(snd_data) < THRESHOLD


def is_silent_dynamic(avg_volume, snd_data):
    """
    This is for dynamic thresholding.
    Calculates if the volume of the current data frame is (100+x)% louder
    than the avg volume.
    """
    rospy.loginfo(max(snd_data))
    return max(snd_data) < avg_volume * (1 + DYNAMIC_THRESHOLD_Percentage / 100.0)


def normalize(snd_data):
    """
    Average the volume out
    """
    MAXIMUM = 16384
    times = float(MAXIMUM) / max(abs(i) for i in snd_data)

    r = array('h')
    for i in snd_data:
        r.append(int(i * times))
    return r


def trim(start, end, snd_data):
    """
    This function is not used in dynamic thresholding.
    Trim the blank spots at the start and end.
    """
    def _trim(snd_data):
        snd_started = False
        r = array('h')
        for i in snd_data:
            if not snd_started and abs(i) > THRESHOLD:
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


def add_silence(snd_data, seconds):
    """
    Add silence to the start and end of 'snd_data' of length 'seconds' (float)
    This prevents some players from skipping the first few frames.
    """
    r = array('h', [0 for i in xrange(int(seconds * RATE))])
    r.extend(snd_data)
    r.extend([0 for i in xrange(int(seconds * RATE))])
    return r


def get_next_utter(stream, min_avg_volume, pub_screen):
    """
    Main function for capturing audio.
    Parameters:
        stream: our pyaudio client
        min_avg_volume: helps thresholding in quiet environments
        pub_screen: publishes status messages to baxter screen
    """
    num_silent = 0
    snd_started = False
    stream.start_stream()
    r = array('h')
    peak_count = 0
    avg_volume = 0
    volume_queue = Queue(10)
    volume_sum = 0
    q_size = 0

    while 1:
        """
        main loop for audio capturing
        """
        if snd_started:
            pub_screen.publish("Sentence Started")
        if rospy.is_shutdown():
            return None, None, None
        # little endian, signed short
        snd_data = array('h', stream.read(CHUNK_SIZE))
        if byteorder == 'big':
            snd_data.byteswap()

        # Static thresholding
        if not DYNAMIC_THRESHOLD:
            r.extend(snd_data)
            silent = is_silent(snd_data)
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

        """
        Dynamic thresholding
        Before audio is being considered part of a sentence, peak_count
        is used to count how many consecutive frames have been above the
        dynamic threshold volume. Once peak_count is over the specified
        frame number from ros param, we consider the sentence started and
        lock the value of avg volume to maintain the standard thresholding
        standard throughout the sentence. Whenever receiving a frame that
        has volume that is too low, we increase num_silent. When num_silent
        exceeds ten, we consider the sentence finished.
        """
        if DYNAMIC_THRESHOLD:
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
            silent = is_silent_dynamic(avg_volume, snd_data)

            if silent and snd_started:
                r.extend(snd_data)
                num_silent += 1
            elif not silent and snd_started:
                r.extend(snd_data)
            elif silent and not snd_started:
                peak_count = 0
                r = array('h')
            elif not silent and not snd_started:
                if peak_count >= DYNAMIC_THRESHOLD_Frame:
                    rospy.logwarn('collecting audio segment')
                    r.extend(snd_data)
                    start_frame = snd_data
                    start_time = rospy.get_rostime()
                    snd_started = True
                    num_silent = 0
                else:
                    peak_count += 1
                    r.extend(snd_data)
            if snd_started and num_silent > 10:
                rospy.logwarn('audio segmend completed')
                r.extend(snd_data)
                end_frame = snd_data
                break

    stream.stop_stream()
    pub_screen.publish("Recognizing")
    end_time = rospy.get_rostime()
    r = normalize(r)
    if not DYNAMIC_THRESHOLD:
        r = trim(r)
    r = add_silence(r, 0.5)
    return r, start_time, end_time
