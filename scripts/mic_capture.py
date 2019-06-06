#!/usr/bin/env python

import rospy
import pyaudio
from std_msgs.msg import String, Header
from ros_speech2text.srv import AudioConfig
from ros_speech2text.msg import AudioChunk
from sys import byteorder
from utilities import switch_endianness

class InvalidDevice(Exception):
	pass

def print_devices(p):
	print 'Available devices:'
	for i in range(p.get_device_count()):
		print '\tDevice #%d: %s\n' % (i, p.get_device_info_by_index(i)['name'])

# Initialize ROS node
rospy.init_node('mic_capture', anonymous = True)
node_name = rospy.get_name()

p = pyaudio.PyAudio()
print_devices(p)

def device_index_by_name(p, name):
	name = name.lower()
	for i in range(p.get_device_count()):
	 	n = p.get_device_info_by_index(i)['name']
		if name in n.lower():
			return i
	return None

# Get audio configuration
sample_rate = rospy.get_param(node_name + '/sample_rate', None)
num_channels = rospy.get_param(node_name + '/num_channels', 1)
sample_width = rospy.get_param(node_name + 'sample_width', 2)
format = p.get_format_from_width(sample_width)

frames_per_buffer = rospy.get_param(node_name + '/frames_per_buffer', 4096)
device_index = rospy.get_param(node_name + '/device_index', None)
device_name = rospy.get_param(node_name + '/device_name', None)
output_stream = rospy.get_param(node_name + '/output_stream')

# Find device index
if device_index is None:
	# Use default device
	if device_name is None:
		device_index = p.get_default_input_device_info()['index']
	# Look-up device by name
	else:
		device_index = device_index_by_name(p, device_name)
		if device_index is None:
			p.terminate()
			raise InvalidDevice('Invalid device name: %s.' % device_name)

rospy.loginfo("{} using device: {}".format(
    node_name,
    p.get_device_info_by_index(device_index)['name'])
)

# Set sample rate
if sample_rate is None:
	sample_rate = p.get_device_info_by_index(device_index)['defaultSampleRate']

# Make configuration info available by a service
config = {
	"num_channels": num_channels,
	"sample_width": sample_width,
	"sample_rate": sample_rate,
	"endianness": byteorder
}
rospy.Service(output_stream + '/config', AudioConfig, lambda req: config)

pub_chunk = rospy.Publisher(output_stream + '/chunk', AudioChunk)
rospy.loginfo('Publishing audio chunks to {}'.format(output_stream + '/'))

# Open pyaudio stream
stream = p.open(
		input=True,
		output=False,
		format=format,
		channels=num_channels,
		rate=sample_rate,
		frames_per_buffer=frames_per_buffer,
		input_device_index=device_index)

# Read and publish chunks from stream
chunk_index = 0
while not rospy.is_shutdown():
    chunk = stream.read(frames_per_buffer, exception_on_overflow=False)
    if byteorder == 'big':
        chunk = switch_endianness(chunk, sample_width)
    pub_chunk.publish(chunk, rospy.get_rostime(), chunk_index)
    chunk_index += 1

stream.stop_stream()
stream.close()
p.terminate()
