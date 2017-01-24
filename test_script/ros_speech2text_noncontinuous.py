!!WARNING: THIS FILE IS NOT UP-TO-DATE WITH DEVELOPMENT. RUN THE PACKAGE INSTEAD!!

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

THRESHOLD = 700
# RATE = 44100
# RATE = 48000
# CHUNK_SIZE = 1024
FORMAT = pyaudio.paInt16
run_flag = True

def is_silent(snd_data):
    "Returns 'True' if below the 'silent' threshold"
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

def get_next_utter():
    p = pyaudio.PyAudio()
    stream = p.open(format=FORMAT, channels=1, rate=RATE,input=True, output=True, frames_per_buffer=CHUNK_SIZE)
    num_silent = 0
    snd_started = False

    r = array('h')

    while 1:
        # little endian, signed short
        snd_data = array('h', stream.read(CHUNK_SIZE))
        if byteorder == 'big':
            snd_data.byteswap()
        r.extend(snd_data)

        silent = is_silent(snd_data)

        if silent and snd_started:
            num_silent += 1
        elif not silent and not snd_started:
            print('collecting audio segment')
            snd_started = True
            num_silent = 0

        if snd_started and num_silent > 10:
            print('audio segment completed')
            break

    sample_width = p.get_sample_size(FORMAT)
    stream.stop_stream()
    stream.close()
    p.terminate()

    r = normalize(r)
    r = trim(r)
    r = add_silence(r, 0.5)
    return sample_width, r

def recog(speech_client, sn, context):
    file_name = 'sentence' + str(sn) + '.wav'
    file_path = os.path.join(os.path.dirname(__file__),'speech_history',file_name)
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
            print('Transcript: {}'.format(alternative.transcript))
            return alternative.transcript
    except ValueError:
        print('No good result returned')
        return None

def record_to_file(sample_width, data, sn):
    "Records from the microphone and outputs the resulting data to 'path'"
    data = pack('<' + ('h'*len(data)), *data)
    file_name = 'sentence' + str(sn) + '.wav'
    file_path = os.path.join(os.path.dirname(__file__),'speech_history',file_name)
    wf = wave.open(file_path, 'wb')
    wf.setnchannels(1)
    wf.setsampwidth(sample_width)
    wf.setframerate(RATE)
    wf.writeframes(data)
    wf.close()
    print('file saved')

def sig_hand(signum, frame):
    global run_flag
    run_flag = False
    print("Stopping Recognition")

def main():
    global RATE
    global CHUNK_SIZE
    pub = rospy.Publisher('user_input', String, queue_size=10)
    rospy.init_node('speech2text_engine', anonymous=True)
    # default sample rate 16000
    RATE = rospy.get_param('/ros_speech2text/audio_rate',16000)
    print RATE
    CHUNK_SIZE = int(RATE/10)

    speech_client = speech.Client()
    signal.signal(signal.SIGINT, sig_hand)
    sn = 0

    while run_flag:
        sample_width, aud_data = get_next_utter()
        record_to_file(sample_width,aud_data, sn)
        context = rospy.get_param('/ros_speech2text/speech_context',[])
        transcript = recog(speech_client, sn, context)
        sn += 1
        if transcript:
            rospy.loginfo(transcript)
            pub.publish(transcript)

if __name__ == '__main__':
    main()
