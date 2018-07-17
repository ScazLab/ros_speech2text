#!/usr/bin/env python

import rospy
import rospkg
import os
import os.path
import sys
import random
import time
import datetime
import collections
import socket
import pdb
import json
import time
from google.cloud import language
from google.oauth2 import service_account
from ros_speech2text.msg import transcript, start_utterance
# from nltk.corpus import cmudict


class TabletSession:

    def __init__(self, host, port):
        self.host = host
        self.port = port
        self.conn = None


    # sends a single ! as the transcript to indicate that speech has begun
    def callback_start_utterance(self, data):
        # send as one big string emulating the syntax of a JSON object. It should later be parsed from
        # a string into a JSON object in the Android app.
        # Using a JSON object makes it easy to parse through the data
        start_indicator = "{" \
                        + "\"start_time\": " + "0" + "," \
                        + "\"end_time\": " + "0" + "," \
                        + "\"speech_duration\": " + "0" + "," \
                        + "\"received_time\": " + "0" + "," \
                        + "\"transcript\": " + "\"" + "!" + "\"" + "," \
                        + "\"confidence\": " + "0" + "," \
                        + "\"pid\": " + str(int(data.pid)) + ","\
                        + "\"sentiment\": " + "0" + "," \
                        + "\"recv-end\": " + "0" \
                        + "}"
        print "Sending start_utterance: %s" % start_indicator
        # need the explicit "\r\n" because python's conn.send doesn't do any of that for you
        # this is required only if the machine that is coding the Android app is a Windows machine
        # don't need the \r for Linux or Mac, but shouldn't cause any
        # problems if it is present for Linux or Mac machines
        self.conn.send(start_indicator + "\r\n")

    # sends all relevant information to the TCP client
    def callback_speech_transcript(self, data):
        global language_client

        # format the data so Google can parse through it for sentiment
        document = language.types.Document(
            content = str(data.transcript),
            language = 'en',
            type = 'PLAIN_TEXT'
        )
        response = language_client.analyze_sentiment(
            document = document
        )

        transcript_data = "{" \
                        + "\"start_time\": " + str(data.start_time.to_sec()) + "," \
                        + "\"end_time\": " + str(data.end_time.to_sec()) + "," \
                        + "\"speech_duration\": " + str(data.speech_duration.to_sec()) + "," \
                        + "\"received_time\": " + str(data.received_time.to_sec()) + "," \
                        + "\"transcript\": " + "\"" + "" + str(data.transcript) + "\"" + "," \
                        + "\"confidence\": " + str(float(data.confidence)) + "," \
                        + "\"pid\": " + str(int(data.pid)) + ","\
                        + "\"sentiment\": " + str(response.document_sentiment.score) + "," \
                        + "\"recv-end\": " + str((data.received_time.to_sec() - data.end_time.to_sec())) \
                        + "}"
        print "Sending: %s" % transcript_data
        self.conn.send(transcript_data + "\r\n")

    def run(self):
        BUFFER_SIZE = 1024  
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        s.bind((self.host, self.port))
        print s.getsockname()
        s.listen(1)
        print 'Waiting for client connection...'

        try:
            self.conn, addr = s.accept()
            self.conn.settimeout(None)
            rospy.Subscriber("speech_to_text/start_utterance", start_utterance, self.callback_start_utterance)
            rospy.Subscriber("/speech_to_text/transcript", transcript, self.callback_speech_transcript)
            print "Finished subscribing"
            print 'Connection address:', addr
            msg = self.conn.recv(BUFFER_SIZE)
            print msg
        except KeyboardInterrupt:
            sys.exit()



def establish_tablet_connection():
    name = socket.gethostname()
    TCP_IP = socket.gethostbyname(str(name))
    print str(TCP_IP)
    if str(TCP_IP).startswith('127'):
        #need this for getting IP on linux machine
        TCP_IP = socket.gethostbyname(str(name)+".local")
    TCP_PORT = 9090

    session = TabletSession(TCP_IP, TCP_PORT)
    session.run() 

# For local testing without the need for a client to connect
# These callback functions perform in the exact same way as the other callbacks
# except these don't have a conn.send at the end of them

def callback_speech_transcript(data):
    global language_client

    document = language.types.Document(
        content = str(data.transcript),
        language = 'en',
        type = 'PLAIN_TEXT'
    )
    response = language_client.analyze_sentiment(
        document = document
    )

    transcript_data = "{" \
                        + "\"start_time\": " + str(data.start_time.to_sec()) + "," \
                        + "\"end_time\": " + str(data.end_time.to_sec()) + "," \
                        + "\"speech_duration\": " + str(data.speech_duration.to_sec()) + "," \
                        + "\"received_time\": " + str(data.received_time.to_sec()) + "," \
                        + "\"transcript\": " + "\"" + "" + str(data.transcript) + "\"" + "," \
                        + "\"confidence\": " + str(float(data.confidence)) + "," \
                        + "\"pid\": " + str(int(data.pid)) + ","\
                        + "\"sentiment\": " + str(response.document_sentiment.score) + "," \
                        + "\"recv-end\": " + str((data.received_time.to_sec() - data.end_time.to_sec())) \
                        + "}"
    print "Sending: %s" % transcript_data

def callback_start_utterance(data):
    start_indicator = "{" \
                    + "\"start_time\": " + "0" + "," \
                    + "\"end_time\": " + "0" + "," \
                    + "\"speech_duration\": " + "0" + "," \
                    + "\"received_time\": " + "0" + "," \
                    + "\"transcript\": " + "\"" + "!" + "\"" + "," \
                    + "\"confidence\": " + "0" + "," \
                    + "\"pid\": " + str(int(data.pid)) + ","\
                    + "\"sentiment\": " + "0" + "," \
                    + "\"recv-end\": " + "0" \
                    + "}"
    print "Sending start_utterance: %s" % start_indicator

# depending on which launch file you used, the code will be run either
# locally on the machine without the need for a client to connect or
# it will run with the server client connection
if __name__ == '__main__':
    language_client = language.LanguageServiceClient()

    rospy.init_node('stt_message_sender', anonymous = True)
    # which launch file did you use
    node_name = rospy.get_name()
    # the default will be local but this should never matter because the type is dependent upon
    # which launch file you used (and those shouldn't really ever change)
    launch_type = str(rospy.get_param(node_name + '/type', "local"))
    try:
        if launch_type == "local":
            rospy.Subscriber("speech_to_text/start_utterance", start_utterance, callback_start_utterance)
            rospy.Subscriber("/speech_to_text/transcript", transcript, callback_speech_transcript)
        elif launch_type == "tablet":
            establish_tablet_connection()
        else:
            print "Please check to make sure launch file has the right spelling for launch type"
    except Exception as e:
        print "Please check launch file for launch type"
    rospy.spin()