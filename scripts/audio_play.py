#!/usr/bin/env python

import rospy
import pyaudio
from ros_speech2text.srv import AudioConfig
from ros_speech2text.msg import AudioChunk

def callback(msg, cb_args):
    (stream, config) = cb_args
    chunk = msg.chunk
    if config.endianness == 'big':
        chunk = switch_endianness(chunk, config.sample_width)
    stream.write(chunk)

if __name__ == '__main__':
    # Initialize ROS node
    rospy.init_node('audio_save', anonymous = True)
    node_name = rospy.get_name()

    p = pyaudio.PyAudio()

    input_stream = rospy.get_param(node_name + '/input_stream')

    # Get the information about the audio stream
    rospy.loginfo('Waiting for connection to input stream service...')
    rospy.wait_for_service(input_stream + '/config')
    config = rospy.ServiceProxy(input_stream + '/config', AudioConfig)()
    rospy.loginfo('Input stream configuration received.')

    # Open output stream
    stream = p.open(
        format=p.get_format_from_width(config.sample_width),
        channels=config.num_channels,
        rate=config.sample_rate,
        output=True
    )

    rospy.Subscriber(input_stream + '/chunk', AudioChunk, callback, callback_args=(stream, config))

    rospy.spin()

    stream.stop_stream()
    stream.close()
    p.terminate()
