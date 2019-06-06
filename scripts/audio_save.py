#!/usr/bin/env python

import rospy
import wave
from os import path
import datetime

from ros_speech2text.srv import AudioConfig
from ros_speech2text.msg import AudioChunk
from utilities import switch_endianness

# Initialize ROS node
rospy.init_node('audio_save', anonymous = True)
node_name = rospy.get_name()

input_stream = rospy.get_param(node_name + '/input_stream')
prefix = rospy.get_param(node_name + '/file_prefix', 'out')
dir = rospy.get_param(node_name + '/output_directory', '~')

# Get the information about the audio stream
rospy.loginfo('Waiting for connection to input stream...')
rospy.wait_for_service(input_stream + '/config')
config = rospy.ServiceProxy(input_stream + '/config', AudioConfig)()
rospy.loginfo('Connected to input stream')

# Use audio stream info to initialize the wave file
filename = path.join(dir, prefix + str(datetime.datetime.now()) + '.wav')
wf = wave.open(path.join(dir, prefix + str(datetime.datetime.now()) +'.wav'), 'wb')
wf.setnchannels(config.num_channels)
wf.setframerate(config.sample_rate)
wf.setsampwidth(config.sample_width)

def callback(msg):
    chunk = msg.chunk
    if config.endianness == 'big':
        chunk = switch_endianness(chunk, config.sample_width)
    wf.writeframes(str(chunk))

rospy.Subscriber(input_stream + '/chunk', AudioChunk, callback)

rospy.spin()

print "Closing wave file"
wf.close()
