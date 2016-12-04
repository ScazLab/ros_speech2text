#!/usr/bin/python
# Copyright (C) 2016 Google Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
"""Sample that streams audio to the Google Cloud Speech API via GRPC."""

from __future__ import division

import contextlib
import functools
import re
import signal
import sys
import rospy

from std_msgs.msg import String
from google.cloud import credentials
from google.cloud.grpc.speech.v1beta1 import cloud_speech_pb2 as cloud_speech
from google.rpc import code_pb2
from grpc.beta import implementations
from grpc.framework.interfaces.face import face
import pyaudio
from six.moves import queue
from time import time

# Audio recording parameters
RATE = 16000
CHUNK = int(RATE / 10)  # 100ms

# The Speech API has a streaming limit of 60 seconds of audio*, so keep the
# connection alive for that long, plus some more to give the API time to figure
# out the transcription.
# * https://g.co/cloud/speech/limits#content
DEADLINE_SECS = 60 * 3 + 5
# DEADLINE_SECS = 10 * 3 + 5
SPEECH_SCOPE = 'https://www.googleapis.com/auth/cloud-platform'


def make_channel(host, port):
    """Creates an SSL channel with auth credentials from the environment."""
    # In order to make an https call, use an ssl channel with defaults
    ssl_channel = implementations.ssl_channel_credentials(None, None, None)

    # Grab application default credentials from the environment
    creds = credentials.get_credentials().create_scoped([SPEECH_SCOPE])
    # Add a plugin to inject the creds into the header
    auth_header = (
        'Authorization',
        'Bearer ' + creds.get_access_token().access_token)
    auth_plugin = implementations.metadata_call_credentials(
        lambda _, cb: cb([auth_header], None),
        name='google_creds')

    # compose the two together for both ssl and google auth
    composite_channel = implementations.composite_channel_credentials(
        ssl_channel, auth_plugin)

    return implementations.secure_channel(host, port, composite_channel)


def _audio_data_generator(buff):
    """A generator that yields all available data in the given buffer.
    Args:
        buff - a Queue object, where each element is a chunk of data.
    Yields:
        A chunk of data that is the aggregate of all chunks of data in `buff`.
        The function will block until at least one data chunk is available.
    """
    stop = False
    while not stop:
        # Use a blocking get() to ensure there's at least one chunk of data.
        data = [buff.get()]

        # Now consume whatever other data's still buffered.
        while True:
            try:
                data.append(buff.get(block=False))
            except queue.Empty:
                break

        # `None` in the buffer signals that the audio stream is closed. Yield
        # the final bit of the buffer and exit the loop.
        if None in data:
            stop = True
            data.remove(None)

        yield b''.join(data)


def _fill_buffer(buff, in_data, frame_count, time_info, status_flags):
    """Continuously collect data from the audio stream, into the buffer."""
    buff.put(in_data)
    return None, pyaudio.paContinue


# [START audio_stream]
@contextlib.contextmanager
def record_audio(rate, chunk):
    """Opens a recording stream in a context manager."""
    # Create a thread-safe buffer of audio data
    buff = queue.Queue()

    audio_interface = pyaudio.PyAudio()
    audio_stream = audio_interface.open(
        format=pyaudio.paInt16,
        # The API currently only supports 1-channel (mono) audio
        # https://goo.gl/z757pE
        channels=1, rate=rate,
        input=True, frames_per_buffer=chunk,
        # Run the audio stream asynchronously to fill the buffer object.
        # This is necessary so that the input device's buffer doesn't overflow
        # while the calling thread makes network requests, etc.
        stream_callback=functools.partial(_fill_buffer, buff),
    )

    yield _audio_data_generator(buff)

    audio_stream.stop_stream()
    audio_stream.close()
    # Signal the _audio_data_generator to finish
    buff.put(None)
    audio_interface.terminate()
# [END audio_stream]

def request_stream(data_stream, rate, interim_results=True):
    """Yields `StreamingRecognizeRequest`s constructed from a recording audio
    stream.
    Args:
        data_stream: A generator that yields raw audio data to send.
        rate: The sampling rate in hertz.
        interim_results: Whether to return intermediate results, before the
            transcription is finalized.
    """
    # The initial request must contain metadata about the stream, so the
    # server knows how to interpret it.
    recognition_config = cloud_speech.RecognitionConfig(
        # There are a bunch of config options you can specify. See
        # https://goo.gl/KPZn97 for the full list.
        encoding='LINEAR16',  # raw 16-bit signed LE samples
        sample_rate=rate,  # the rate in hertz
        # See http://g.co/cloud/speech/docs/languages
        # for a list of supported languages.
        language_code='en-US',  # a BCP-47 language tag
        #speech_context=context,
    )
    streaming_config = cloud_speech.StreamingRecognitionConfig(
        interim_results=interim_results,
        config=recognition_config,
    )

    yield cloud_speech.StreamingRecognizeRequest(
        streaming_config=streaming_config)

    for data in data_stream:
        # Subsequent requests can all just have the content
        # print("Request Sent")
        yield cloud_speech.StreamingRecognizeRequest(audio_content=data)


def dump_output(recognize_stream,publisher):
    print("start dumping output")
    t1 = time()
    for resp in recognize_stream:
        # check time out here
        if (time() - t1) > 55:
            print('Exiting..')
            break
        if resp.error.code != code_pb2.OK:
            raise RuntimeError('Server error: ' + resp.error.message)
        if not resp.results:
            continue
        result = resp.results[0]
        transcript = result.alternatives[0].transcript
        confidence = result.alternatives[0].confidence
        if result.is_final:
            print(transcript)
            print(confidence)
            rospy.loginfo(transcript)
            publisher.publish(transcript)

        #should terminate after sixty seconds

def add_context(data):
    print data.data

def create_new_service():
    channel = make_channel('speech.googleapis.com', 443)
    service = cloud_speech.beta_create_Speech_stub(channel)
    return service


def recognize(pub):
    with cloud_speech.beta_create_Speech_stub(
            make_channel('speech.googleapis.com', 443)) as service:
        # For streaming audio from the microphone, there are three threads.
        # First, a thread that collects audio data as it comes in
        # print("entered make_channel")
        with record_audio(RATE, CHUNK) as buffered_audio_data:
            # print("entered record_audio")

            # Second, a thread that sends requests with that data
            # requests is a generator
            # when we have a new_context, we should make a new generator
            requests = request_stream(buffered_audio_data, RATE)
            # Third, a thread that listens for transcription responses
            recognize_stream = service.StreamingRecognize(
                requests, DEADLINE_SECS)

            # Exit things cleanly on interrupt
            # signal.signal(signal.SIGINT, lambda *_: recognize_stream.cancel())

            # Now, put the transcription responses to use.
            try:
                #listen_print_loop(recognize_stream,pub)
                dump_output(recognize_stream,pub)
                recognize_stream.cancel()
            except face.CancellationError:
                # This happens because of the interrupt handler
                pass
            print("end of session")

run_flag = True

def sig_hand(signum, frame):
    global run_flag
    run_flag = False
    print("Stopping Recognition")

def main():
    # Code for creating a ROS node

    pub = rospy.Publisher('user_input', String, queue_size=10)
    rospy.init_node('speech2text_engine', anonymous=True)
    sub = rospy.Subscriber('context_input', String, add_context)

    signal.signal(signal.SIGINT, sig_hand)
    while run_flag:
        recognize(pub)

    #it stays in recognize
    print("after recognize")



if __name__ == '__main__':
    main()