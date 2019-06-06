#!/usr/bin/env python

import rospy
import wave
from os import path
import datetime

from ros_speech2text.msg import Utterance
from utilities import switch_endianness

# Initialize ROS node
rospy.init_node('utterance_save', anonymous = True)
node_name = rospy.get_name()

input_stream = rospy.get_param(node_name + '/input_stream')
prefix = rospy.get_param(node_name + '/file_prefix', 'utt')
dir = rospy.get_param(node_name + '/output_directory', '~')

def callback(msg):
    chunk = msg.audio_chunk.chunk
    config = msg.audio_config
    if config.endianness == 'big':
        chunk = switch_endianness(chunk, config.sample_width)

    filename = path.join(dir, prefix + str(datetime.datetime.now()) + '.wav')
    wf = wave.open(filename, 'wb')
    wf.setnchannels(config.num_channels)
    wf.setframerate(config.sample_rate)
    wf.setsampwidth(config.sample_width)
    wf.writeframes(chunk)
    wf.close()

rospy.Subscriber(input_stream + '/complete', Utterance, callback)
rospy.spin()
