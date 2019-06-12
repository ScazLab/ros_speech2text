#!/usr/bin/env python

import rospy

from google.cloud import speech_v1p1beta1 as speech
from google.cloud.speech_v1p1beta1 import enums
from google.cloud.speech_v1p1beta1 import types

from std_msgs.msg import String, Header, Time
from ros_speech2text.msg import Utterance, Transcript

def callback(msg, cb_args):
    (speech_client, pub_transcript, output_stream) = cb_args

    sample_rate = msg.audio_config.sample_rate
    chunk = msg.audio_chunk.chunk

    if msg.audio_config.sample_width != 2:
        raise Exception('Width ' + str(sample_width) + ' cannot be handled.')

    config = types.RecognitionConfig(
        encoding='LINEAR16',
        sample_rate_hertz=sample_rate,
        language_code='en-US',
        enable_automatic_punctuation=True)

    audio = types.RecognitionAudio(content=chunk)
    response = speech_client.recognize(config, audio)
    if response.results:
        transcript = response.results[0].alternatives[0].transcript
        confidence = response.results[0].alternatives[0].confidence
        pub_transcript.publish(transcript, msg.start_time, msg.duration, msg.index, confidence)
        rospy.loginfo('From: ' + output_stream)
        rospy.loginfo('Transcript: ' + transcript)
    else:
        rospy.loginfo('Speech not recognized.')

if __name__ == '__main__':
    rospy.init_node('speech_recognize', anonymous = True)
    node_name = rospy.get_name()

    input_stream = rospy.get_param(node_name + '/input_stream')

    output_stream = rospy.get_param(node_name + '/output_stream')

    pub_transcript = rospy.Publisher(output_stream + '/transcript', Transcript)
    rospy.loginfo('Publishing transcripts to {}'.format(output_stream + '/transcript'))

    client = speech.SpeechClient()

    rospy.Subscriber(input_stream + '/complete', Utterance, callback, (client, pub_transcript, output_stream))
    rospy.spin()
