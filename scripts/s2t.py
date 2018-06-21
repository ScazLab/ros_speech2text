#!/usr/bin/env python

import sys

import rospy

from ros_speech2text.test_speech import SpeechRecognizer


if __name__ == '__main__':
    rospy.init_node('speech2text_engine', anonymous=True)
    try:
        recog = SpeechRecognizer()
    except SpeechRecognizer.InvalidDevice as e:
        rospy.logerr(e.message)
        sys.exit(1)
