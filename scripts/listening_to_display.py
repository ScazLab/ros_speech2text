from threading import Lock

import rospy
from std_msgs.msg import String

from s2t.speech_recognition.msg import event


class ListeningToDisplay(object):

    PERIOD = .1
    MESSAGE = 'Listening...'
    # To set message duration on the display node
    DURATION = 1
    DURATION_PARAM = 'baxter_display/speech_duration'

    def __init__(self, display_topic):
        rospy.init_node('Display listening')
        rospy.set_param(self.DURATION_PARAM, self.DURATION)
        self.sub = rospy.Subscriber('/speech_to_text/log',
                                    event, self._event_cb)
        self.pub = rospy.Publisher(display_topic, String, queue_size=2)
        self._listening = False
        self._lock = Lock()

    def run(self):
        self.running = True
        self.last_msg = rospy.Time.now()
        while self.running and not rospy.is_shutdown():
            if self.listening and (
                    rospy.Time.now() - self.last_msg) > self.DURATION:
                self.pub.publish(self.MESSAGE)

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
            self.listening = True
        elif msg.event == event.STOPPED:
            self.listening = False


if __name__ == '__main__':
    ListeningToDisplay('/svox_tts/speech_output').run()
