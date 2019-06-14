#!/usr/bin/env python

import numpy as np
from time import sleep
import rospy

from buffers import OtherBuffer
from utterance_detector import UtteranceDetector, UtteranceDetectorCallback
from semaphore import Semaphore
from utilities import dtype_from_width

from std_msgs.msg import String, Time
from ros_speech2text.srv import AudioConfig
from ros_speech2text.msg import AudioChunk, UtteranceChunk, Utterance, StartUtterance, EndUtterance

def subscriber_callback(msg, cb_args):
    (semaphore, dtype, utterance_detector) = cb_args
    if not semaphore.enter(msg.index):
        # You missed your turn!
        # This message will be ignored.
        return
    try:
        audio_chunk = np.frombuffer(str(msg.chunk), dtype=dtype)
        utterance_detector.put_audio_chunk(audio_chunk, msg.time)
    finally:
        # Don't forget to free up the semaphore for the next message
        semaphore.exit()

class Callback(UtteranceDetectorCallback):
    def __init__(self, audio_config, min_output_chunk_size, dtype, output_stream):
        self.buffer = OtherBuffer(min_output_chunk_size, dtype)
        self.chunk_index = 0
        self.utterance_index = 0
        self.audio_config = audio_config
        self.output_stream = output_stream

        self.pub_started = rospy.Publisher(output_stream + '/started', StartUtterance).publish
        self.pub_ended = rospy.Publisher(output_stream + '/ended', EndUtterance).publish
        self.pub_complete = rospy.Publisher(output_stream + '/complete', Utterance).publish
        self.pub_chunk = rospy.Publisher(output_stream + '/chunk', UtteranceChunk).publish

    def length2time(self, length):
        return length / float(self.audio_config.num_channels) / self.audio_config.sample_rate

    def on_utterance_started(self, timestamp):
        rospy.loginfo('Utterance by ' + self.output_stream + ' at ' + str(timestamp))
        self.pub_started(timestamp, self.utterance_index)
        self.buffer.start_time = timestamp

    def on_utterance_completed(self, utterance, start_time):
        duration = self.length2time(utterance.size)
        rospy.loginfo('Utterance completed. Start time: %s, Duration: %s.' % (start_time, duration))

        # Process what's left in the buffer, send is_end=True flag
        self.process_buffer(True)

        self.pub_ended(start_time,
                          duration,
                          self.utterance_index)

        chk = AudioChunk(utterance.tostring(), start_time, 0)
        self.pub_complete(chk,
                          self.audio_config,
                          start_time,
                          duration,
                          self.utterance_index)

    def on_utterance_chunk(self, chunk, start_time):
        self.buffer.put(chunk, start_time)
        if self.buffer.is_full:
            self.process_buffer(False)

    def process_buffer(self, is_end):
        duration = self.length2time(self.buffer.get().size)
        chk = AudioChunk(self.buffer.get().tostring(), self.buffer.start_time, self.chunk_index)
        self.pub_chunk(chk,
                       self.audio_config,
                       self.buffer.start_time,
                       duration,
                       self.utterance_index,
                       self.chunk_index,
                       is_end)
        self.chunk_index += 1
        self.buffer.reset()
        if is_end:
            self.utterance_index += 1
            self.chunk_index = 0

if __name__ == '__main__':
    rospy.init_node('utterance_detect', anonymous = True)
    node_name = rospy.get_name()

    input_stream = rospy.get_param(node_name + '/input_stream')
    output_stream = rospy.get_param(node_name + '/output_stream')
    calibrate = rospy.get_param(node_name + '/calibrate', False)

    # Get the information about the audio stream
    rospy.loginfo('Waiting for connection to input stream service...')
    rospy.wait_for_service(input_stream + '/config')
    config = rospy.ServiceProxy(input_stream + '/config', AudioConfig)()
    rospy.loginfo('Input stream configuration received.')

    rospy.Service(output_stream + '/config', AudioConfig, lambda req: config)

    # Get utterance parameters
    sd_block_duration = rospy.get_param(
        node_name + '/sd_block_duration', 50)
    leading_noise_duration = rospy.get_param(
        node_name + '/leading_noise_duration', 500)
    trailing_silence_duration = rospy.get_param(
        node_name + '/trailing_silence_duration', 500)
    threshold_pct = rospy.get_param(
        node_name + '/threshold_pct', 10)

    # Find the appropriate numpy data type
    dtype = dtype_from_width(config.sample_width)
    if dtype is None:
        print Exception('Sample width ' + str(sample_width) + ' cannot be handled.')

    # Convert size in bytes to array length
    min_output_chunk_size = rospy.get_param(
        node_name + '/min_output_chunk_size', None)
    if min_output_chunk_size is None:
        size = 0
    else:
        size = min_output_chunk_size / config.sample_width
        size = int(size / config.num_channels) * config.num_channels

    cb = Callback(config, int(size), dtype, output_stream)
    utterance_detector = UtteranceDetector(config.sample_rate, config.num_channels, dtype,
                        sd_block_duration, leading_noise_duration, trailing_silence_duration,
                        threshold_pct, calibrate, cb)

    semaphore = Semaphore()

    rospy.Subscriber(input_stream + '/chunk', AudioChunk, subscriber_callback,
        callback_args=(semaphore, dtype, utterance_detector))
    rospy.spin()
