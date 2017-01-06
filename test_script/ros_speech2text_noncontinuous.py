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
RATE = 16000
CHUNK_SIZE = int(RATE / 10)  # 100ms

THRESHOLD = 700
# RATE = 44100
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
    stream = p.open(format=FORMAT, channels=1, rate=RATE,
        input=True, output=True,
        frames_per_buffer=CHUNK_SIZE)

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
            snd_started = True
            num_silent = 0

        if snd_started and num_silent > 30:
            break

    sample_width = p.get_sample_size(FORMAT)
    stream.stop_stream()
    stream.close()
    p.terminate()

    r = normalize(r)
    r = trim(r)
    r = add_silence(r, 0.5)
    return r

def recog(speech_client, data):
	audio_sample = speech_client.sample(
            data,
            source_uri=None,
            encoding='LINEAR16',
            sample_rate=RATE)

	alternatives = speech_client.speech_api.sync_recognize(audio_sample)

    for alternative in alternatives:
        print('Transcript: {}'.format(alternative.transcript))
        return alternative.transcript

def sig_hand(signum, frame):
    global run_flag
    run_flag = False
    print("Stopping Recognition")

def main():
	pub = rospy.Publisher('user_input', String, queue_size=10)
    rospy.init_node('speech2text_engine', anonymous=True)
    sub = rospy.Subscriber('context_input', String, add_context)
    speech_client = speech.Client()
    signal.signal(signal.SIGINT, sig_hand)

    while run_flag:
    	aud_data = get_next_utter()
    	transcript = recog(speech_client, aud_data)
    	rospy.loginfo(transcript)
        pub.publish(transcript)

if __name__ == '__main__':
    main()