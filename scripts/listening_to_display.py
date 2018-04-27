#!/usr/bin/env python

from threading import Lock

import rospy
from std_msgs.msg import String

from ros_speech2text.msg import event


class ListeningToDisplay(object):

    PERIOD = .1
    MESSAGE = 'Listening'
    # To set message duration on the display node
    DURATION = 1
    DURATION_PARAM = 'baxter_display/onscreen_duration'

    def __init__(self, display_topic):
        rospy.init_node('listening_to_display')
        rospy.sleep(5)
        self.sub = rospy.Subscriber('/speech_to_text/log',
                                    event, self._event_cb)
        self.pub = rospy.Publisher(display_topic, String, queue_size=10)
        self._listening = False
        self._lock = Lock()
        rospy.loginfo('Ready')

    def run(self):
        self.running = True
        self.last_msg = rospy.Time.now()
        while self.running and not rospy.is_shutdown():
            if (rospy.Time.now() - self.last_msg) > rospy.Duration(self.DURATION):
                if self.listening:
                    self.pub.publish(self.MESSAGE)
                else:
                    self.pub.publish(' ')
                self.last_msg = rospy.Time.now()
            rospy.sleep(self.PERIOD)

    @property
    def listening(self):
        with self._lock:
            return self._listening

    @listening.setter
    def listening(self, b):
        with self._lock:
            self._listening = b

    def _event_cb(self, msg):
        if msg.event == event.STARTED:
            rospy.loginfo('starting')
            self.listening = True
        elif msg.event == event.STOPPED:
            rospy.loginfo('stopping')
            self.listening = False


if __name__ == '__main__':
    ListeningToDisplay('/svox_tts/speech_output').run()
