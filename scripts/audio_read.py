#!/usr/bin/env python

import rospy
import pyaudio
import wave
from time import sleep
from std_msgs.msg import String, Header
from ros_speech2text.srv import AudioConfig
from ros_speech2text.msg import AudioChunk

# Initialize ROS node
rospy.init_node('audio_read', anonymous = True)
node_name = rospy.get_name()

input_file = rospy.get_param(node_name + '/input_file')
output_stream = rospy.get_param(node_name + '/output_stream')
frames_per_buffer = rospy.get_param(node_name + '/frames_per_buffer')

wf = wave.open(input_file, 'rb')
num_channels = wf.getnchannels()
sample_width = wf.getsampwidth()
sample_rate = wf.getframerate()

# Make configuration info available by a service
config = {
	"num_channels": num_channels,
	"sample_width": sample_width,
	"sample_rate": sample_rate,
	"endianness": 'little'
}
rospy.Service(output_stream + '/config', AudioConfig, lambda req: config)

pub_chunk = rospy.Publisher(output_stream + '/chunk', AudioChunk)
rospy.loginfo('Publishing audio chunks to /{}/chunk'.format(output_stream))

rospy.loginfo('Waiting for subscriber to ' + output_stream + '/chunk')
while pub_chunk.get_num_connections() == 0:
	sleep(0.001)
rospy.loginfo('At least 1 subscriber found! Reading audio file...')

data = wf.readframes(frames_per_buffer)
chunk_index = 0
while len(data) > 0:
	pub_chunk.publish(data, rospy.get_rostime(), chunk_index)
	chunk_index += 1
	data = wf.readframes(frames_per_buffer)

wf.close()
rospy.loginfo('Done reading audio file')
