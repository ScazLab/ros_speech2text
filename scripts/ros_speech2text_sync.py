#!/usr/bin/env python

from sys import byteorder
from array import array
from struct import pack
from std_msgs.msg import String
from google.cloud import speech
from time import time

import pyaudio
import wave
import io
import os
import rospy
import signal
import sys
import Queue
import csv

# Audio recording parameters
# RATE = 16000
# CHUNK_SIZE = int(RATE / 10)  # 100ms

RATE = None
CHUNK_SIZE = None
SPEECH_HISTORY_DIR = None
THRESHOLD = None
FORMAT = pyaudio.paInt16
run_flag = True
DYNAMIC_THRESHOLD = None
DYNAMIC_THRESHOLD_Percentage = None
DYNAMIC_THRESHOLD_Frame = None

def is_silent(snd_data):
    "Returns 'True' if below the 'silent' threshold"
    rospy.loginfo(max(snd_data))
    return max(snd_data) < THRESHOLD

def is_silent_dynamic(avg_volume, snd_data):
    rospy.loginfo(max(snd_data))
    return max(snd_data) < avg_volume*(1+DYNAMIC_THRESHOLD_Percentage/100.0)

def normalize(snd_data):
    "Average the volume out"
    MAXIMUM = 16384
    times = float(MAXIMUM)/max(abs(i) for i in snd_data)

    r = array('h')
    for i in snd_data:
        r.append(int(i*times))
    return r

def trim(start, end, snd_data):
    "Trim the blank spots at the start and end"
    def _trim(snd_data):
        snd_started = False
        r = array('h')
        for i in snd_data:
            if not snd_started and abs(i)>THRESHOLD:
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
    "Add silence to the start and end of 'snd_data' of length 'seconds' (float)"
    r = array('h', [0 for i in xrange(int(seconds*RATE))])
    r.extend(snd_data)
    r.extend([0 for i in xrange(int(seconds*RATE))])
    return r

def get_next_utter(stream):
    num_silent = 0
    snd_started = False
    stream.start_stream()
    r = array('h')
    peak_count = 0
    avg_volume = 0
    frame_count = 0
    volume_queue = Queue.Queue(10)
    volume_sum = 0
    q_size = 0

    while 1:
        if rospy.is_shutdown():
            return None
        # little endian, signed short
        snd_data = array('h', stream.read(CHUNK_SIZE))
        # avg_volume = max(snd_data)
        if byteorder == 'big':
            snd_data.byteswap()


        if not DYNAMIC_THRESHOLD:
            r.extend(snd_data)
            silent = is_silent(snd_data)
            if silent and snd_started:
                num_silent += 1
            elif not silent and not snd_started:
                rospy.loginfo('collecting audio segment')
                snd_started = True
                num_silent = 0
            if snd_started and num_silent > 10:
                rospy.loginfo('audio segment completed')
                break

        if DYNAMIC_THRESHOLD:
            if volume_queue.full():
                out = volume_queue.get()
                volume_sum -= out
                q_size -= 1
            if not volume_queue.full() and peak_count==0:
                volume_queue.put(max(snd_data))
                volume_sum += max(snd_data)
                q_size += 1
            avg_volume = volume_sum/q_size

            rospy.loginfo("[AVG_VOLUME] "+ str(avg_volume))
            frame_count += 1
            silent = is_silent_dynamic(avg_volume, snd_data)



            if silent and snd_started:
                r.extend(snd_data)
                num_silent += 1
            elif not silent and snd_started:
                r.extend(snd_data)
            elif silent and not snd_started:

                peak_count = 0
                # flush buffer
                r = array('h')
            elif not silent and not snd_started:
                if peak_count>=DYNAMIC_THRESHOLD_Frame:
                    rospy.loginfo('collecting audio segment')
                    r.extend(snd_data)
                    start_frame = snd_data
                    snd_started = True
                    num_silent = 0
                else:
                    peak_count += 1
                    r.extend(snd_data)
            if snd_started and num_silent > 10:
                rospy.loginfo('audio segmend completed')
                r.extend(snd_data)
                end_frame = snd_data
                break

    stream.stop_stream()

    r = normalize(r)
    if not DYNAMIC_THRESHOLD:
        r = trim(r)
    r = add_silence(r, 0.5)
    return r

def recog(speech_client, sn, context):
    file_name = 'sentence' + str(sn) + '.wav'
    file_path = os.path.join(SPEECH_HISTORY_DIR,file_name)
    with io.open(file_path, 'rb') as audio_file:
        content = audio_file.read()
        audio_sample = speech_client.sample(
            content,
            source_uri=None,
            encoding='LINEAR16',
            sample_rate=RATE)

    try:
        alternatives = speech_client.speech_api.sync_recognize(sample = audio_sample, speech_context = context)
        for alternative in alternatives:
            # print('Transcript: {}'.format(alternative.transcript))
            return alternative.transcript
    except ValueError:
        rospy.logwarn('No good result returned')
        return None

def record_to_file(sample_width, data, sn):
    "Records from the microphone and outputs the resulting data to 'path'"
    data = pack('<' + ('h'*len(data)), *data)
    file_name = 'sentence' + str(sn) + '.wav'
    file_path = os.path.join(SPEECH_HISTORY_DIR,file_name)
    wf = wave.open(file_path, 'wb')
    wf.setnchannels(1)
    wf.setsampwidth(sample_width)
    wf.setframerate(RATE)
    wf.writeframes(data)
    wf.close()
    rospy.loginfo('file saved')

# def expand_dir(SPEECH_HISTORY_DIR):
#     if SPEECH_HISTORY_DIR[0]=='~':
#         SPEECH_HISTORY_DIR = os.getenv("HOME") + SPEECH_HISTORY_DIR[1:]
#     if not os.path.isdir(SPEECH_HISTORY_DIR):
#         os.makedirs(SPEECH_HISTORY_DIR)
#     return SPEECH_HISTORY_DIR

def expand_dir(speech_history_dir):
    """
    A function that expands directories so python can find the folder
    """
    speech_history_dir = os.path.expanduser(
        os.path.join(speech_history_dir, str(os.getpid())))
    if not os.path.isdir(speech_history_dir):
        os.makedirs(speech_history_dir)
    return speech_history_dir

def sig_hand(signum, frame):
    global run_flag
    run_flag = False
    print("Stopping Recognition")

def write_record(transcript,writer):
    global SPEECH_HISTORY_DIR
    writer.writerow([rospy.get_rostime(),transcript])


def main():
    global RATE
    global CHUNK_SIZE
    global THRESHOLD
    global SPEECH_HISTORY_DIR
    global FORMAT
    global DYNAMIC_THRESHOLD
    global DYNAMIC_THRESHOLD_Percentage
    global DYNAMIC_THRESHOLD_Frame

    pub = rospy.Publisher('user_input', String, queue_size=10)
    rospy.init_node('speech2text_engine', anonymous=True)
    # default sample rate 16000
    RATE = rospy.get_param('/ros_speech2text/audio_rate',16000)
    THRESHOLD = rospy.get_param('/ros_speech2text/audio_threshold',700)
    SPEECH_HISTORY_DIR = rospy.get_param('/ros_speech2text/speech_history','~/.ros/ros_speech2text/speech_history')
    SPEECH_HISTORY_DIR = expand_dir(SPEECH_HISTORY_DIR)
    input_idx = rospy.get_param('/ros_speech2text/audio_device_idx',None)
    CHUNK_SIZE = int(RATE/10)
    # CHUNK_SIZE = 8192
    DYNAMIC_THRESHOLD = rospy.get_param('/ros_speech2text/enable_dynamic_threshold',False)
    DYNAMIC_THRESHOLD_Percentage = rospy.get_param('/ros_speech2text/audio_dynamic_percentage',None)
    DYNAMIC_THRESHOLD_Frame = rospy.get_param('/ros_speech2text/audio_dynamic_frame',None)

    # get input device ID
    p = pyaudio.PyAudio()
    device_list = [p.get_device_info_by_index(i)['name'] for i in range(p.get_device_count())]
    rospy.set_param('/ros_speech2text/available_audio_device',device_list)

    if input_idx == None:
        input_idx = p.get_default_input_device_info()['index']
    rospy.loginfo("Using device: " + p.get_device_info_by_index(input_idx)['name'])
    stream = p.open(format=FORMAT, channels=1, rate=RATE,input=True, start = False, input_device_index=input_idx, output=False, frames_per_buffer=CHUNK_SIZE)
    sample_width = p.get_sample_size(FORMAT)

    speech_client = speech.Client()
    # signal.signal(signal.SIGINT, sig_hand)
    sn = 0

    csv_file = open(os.path.join(SPEECH_HISTORY_DIR,'transcript'),'wb')
    writer = csv.writer(csv_file,delimiter=' ',)

    while not rospy.is_shutdown():
    # while run_flag:
        aud_data = get_next_utter(stream)
        if aud_data == None:
            rospy.loginfo("Node terminating")
            break
        # if rospy.is_shutdown():
        #     break
        record_to_file(sample_width,aud_data, sn)
        context = rospy.get_param('/ros_speech2text/speech_context',[])
        transcript = recog(speech_client, sn, context)
        sn += 1
        if transcript:
            rospy.loginfo(transcript)
            pub.publish(transcript)
            write_record(transcript,writer)

    stream.close()
    p.terminate()
    csv_file.close()

if __name__ == '__main__':
    main()
