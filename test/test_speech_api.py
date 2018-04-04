#!/usr/bin/env python
PKG = 'ros_speech2text'
import os
import unittest
from unittest import TestCase

import rospkg

import rospy
from ros_speech2text.msg import transcript
from std_msgs.msg import String


class TestSpeechAPI(TestCase):
    def setUp(self):

        p = rospkg.RosPack()
        package_path = p.get_path('ros_speech2text')
        self.audio_path = os.path.join(package_path, 'test', 'test_audio')
        self.result = None

    # def tearDown(self):
    #     self.pub.unregister()

    def callback1(self, recog_result):
        print "results received"
        self.result = recog_result.transcript

    def test_sentence1(self):
        self.pub = rospy.Publisher('/test_input', String, queue_size=10)
        file_path = os.path.join(self.audio_path, 'sentence0.wav')
        print file_path
        self.pub.publish(file_path)
        rospy.Subscriber('ros_speech2text/user_output', transcript,
                         self.callback1)
        while self.result == None:
            print "waiting for result"
            rospy.sleep(1)
        print self.result
        self.assertEqual(self.result,
                         'good morning Baxter how are you doing today')


if __name__ == '__main__':
    import rostest
    rospy.init_node('test_ros_speech2text')
    rostest.rosrun(PKG, 'test_speech_detection', TestSpeechAPI)
