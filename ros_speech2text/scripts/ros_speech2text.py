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

# Audio recording parameters
# RATE = 16000
# CHUNK_SIZE = int(RATE / 10)  # 100ms

RATE = None
CHUNK_SIZE = None
SPEECH_HISTORY_DIR = None
THRESHOLD = None
FORMAT = pyaudio.paInt16
run_flag = True

def is_silent(snd_data):
    "Returns 'True' if below the 'silent' threshold"
    rospy.logdebug(max(snd_data))
    return max(snd_data) < THRESHOLD

def normalize(snd_data):
    "Average the volume out"
    MAXIMUM = 16384
    times = float(MAXIMUM)/max(abs(i) for i in snd_data)

    r = array('h')
    for i in snd_data:
        r.append(int(i*times))
    return r

def trim(snd_data):
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

    while 1:
        if rospy.is_shutdown():
            return None
        # little endian, signed short
        snd_data = array('h', stream.read(CHUNK_SIZE))
        if byteorder == 'big':
            snd_data.byteswap()
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

    stream.stop_stream()

    r = normalize(r)
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
        rospy.loginfo('No good result returned')
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

def expand_dir(SPEECH_HISTORY_DIR):
    if SPEECH_HISTORY_DIR[0]=='~':
        SPEECH_HISTORY_DIR = os.getenv("HOME") + SPEECH_HISTORY_DIR[1:]
    if not os.path.isdir(SPEECH_HISTORY_DIR):
        os.makedirs(SPEECH_HISTORY_DIR)
    return SPEECH_HISTORY_DIR

def sig_hand(signum, frame):
    global run_flag
    run_flag = False
    print("Stopping Recognition")

# def hook():
#     print("stopping")

def main():
    global RATE
    global CHUNK_SIZE
    global THRESHOLD
    global SPEECH_HISTORY_DIR
    global FORMAT
    pub = rospy.Publisher('user_input', String, queue_size=10)
    rospy.init_node('speech2text_engine', anonymous=True)
    # default sample rate 16000
    RATE = rospy.get_param('/ros_speech2text/audio_rate',16000)
    THRESHOLD = rospy.get_param('/ros_speech2text/audio_threshold',700)
    SPEECH_HISTORY_DIR = rospy.get_param('/ros_speech2text/speech_history','~/.ros/ros_speech2text/speech_history')
    SPEECH_HISTORY_DIR = expand_dir(SPEECH_HISTORY_DIR)
    input_idx = rospy.get_param('/ros_speech2text/audio_device_idx',None)
    CHUNK_SIZE = int(RATE/10)

    speech_client = speech.Client()
    # signal.signal(signal.SIGINT, sig_hand)
    sn = 0

    # get input device ID
    p = pyaudio.PyAudio()

    if input_idx == None:
        input_idx = p.get_default_input_device_info()['index']
    rospy.loginfo("Using device: " + p.get_device_info_by_index(input_idx)['name'])
    stream = p.open(format=FORMAT, channels=1, rate=RATE,input=True, start = False, input_device_index=input_idx, output=False, frames_per_buffer=CHUNK_SIZE)
    sample_width = p.get_sample_size(FORMAT)

    while not rospy.is_shutdown():
    # while run_flag:
        aud_data = get_next_utter(stream)
        if aud_data == None:
            rospy.loginfo("Node terminating")
            break
        record_to_file(sample_width,aud_data, sn)
        context = rospy.get_param('/ros_speech2text/speech_context',[])
        transcript = recog(speech_client, sn, context)
        sn += 1
        if transcript:
            rospy.loginfo(transcript)
            pub.publish(transcript)

    stream.close()
    p.terminate()

if __name__ == '__main__':
    main()
