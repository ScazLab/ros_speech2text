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
from google.cloud import language
from google.oauth2 import service_account
from ros_speech2text.msg import transcript, start_utterance
# from nltk.corpus import cmudict


class TabletSession:

	def __init__(self, host, port):
		self.host = host
		self.port = port
		self.conn = None

	def callback_start_utterance(self, data):
		start_indicator = "{" \
						+ "\"start_time\": " + "0" + "," \
						+ "\"end_time\": " + "0" + "," \
						+ "\"speech_duration\": " + "0" + "," \
						+ "\"received_time\": " + "0" + "," \
						+ "\"transcript\": " + "\"" + "!" + "\"" + "," \
						+ "\"confidence\": " + "0" + "," \
						+ "\"pid\": " + str(int(data.pid)) + ","\
						+ "\"sentiment\": " + "0" + "," \
						+ "\"nouns\": " + "\"" + "\"" + "," \
						+ "\"verbs\": " + "\"" + "\"" + "," \
						+ "\"recv-end\": " + "0" \
						+ "}"
		print "Sending start_utterance: %s" % start_indicator
		self.conn.send(start_indicator + "\r\n")

	def callback_speech_transcript(self, data):
		global language_client

		document = language.types.Document(
			content = str(data.transcript),
			language = 'en',
			type = 'PLAIN_TEXT'
		)
		response = language_client.analyze_sentiment(
			document = document
			# encoding_type = 'UTF32'
		)

		# testing the Google natural language syntax analysis
		# https://cloud.google.com/natural-language/docs/analyzing-syntax
		client = language.LanguageServiceClient()
		tokens = client.analyze_syntax(document).tokens
		nouns = ""
		verbs = ""
		pos_tag = ('UNKNOWN', 'ADJ', 'ADP', 'ADV', 'CONJ', 'DET', 'NOUN', 'NUM',
               'PRON', 'PRT', 'PUNCT', 'VERB', 'X', 'AFFIX')
		for token in tokens:
			# print(u'{}: {}'.format(pos_tag[token.part_of_speech.tag],
   #      		token.text.content))
   			if pos_tag[token.part_of_speech.tag] == 'NOUN':
   				nouns += token.text.content + ", "
   			if pos_tag[token.part_of_speech.tag] == 'VERB':
   				verbs += token.text.content + ", "
   		nouns = nouns[0:-2]
   		verbs = verbs[0:-2]

		transcript_data = "{" \
						+ "\"start_time\": " + str(data.start_time.to_sec()) + "," \
						+ "\"end_time\": " + str(data.end_time.to_sec()) + "," \
						+ "\"speech_duration\": " + str(data.speech_duration.to_sec()) + "," \
						+ "\"received_time\": " + str(data.received_time.to_sec()) + "," \
						+ "\"transcript\": " + "\"" + "" + str(data.transcript) + "\"" + "," \
						+ "\"confidence\": " + str(float(data.confidence)) + "," \
						+ "\"pid\": " + str(int(data.pid)) + ","\
						+ "\"sentiment\": " + str(response.document_sentiment.score) + "," \
						+ "\"nouns\": " + "\"" + str(nouns) + "\"" + "," \
						+ "\"verbs\": " + "\"" + str(verbs) + "\"" + "," \
						+ "\"recv-end\": " + str((data.received_time.to_sec() - data.end_time.to_sec())) \
						+ "}"
		# print "Sending: %s" % json.dumps(json_transcript_data)
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

def callback_speech_transcript(data):
	global language_client

	document = language.types.Document(
		content = str(data.transcript),
		language = 'en',
		# language = 'ko',
		type = 'PLAIN_TEXT'
	)
	response = language_client.analyze_sentiment(
		document = document
		# encoding_type = 'UTF32'
	)

	# testing the Google natural language syntax analysis
	# https://cloud.google.com/natural-language/docs/analyzing-syntax
	client = language.LanguageServiceClient()
	tokens = client.analyze_syntax(document).tokens
	pos_tag = ('UNKNOWN', 'ADJ', 'ADP', 'ADV', 'CONJ', 'DET', 'NOUN', 'NUM',
		'PRON', 'PRT', 'PUNCT', 'VERB', 'X', 'AFFIX')
	nouns = ""
	verbs = ""
	for token in tokens:
   			if pos_tag[token.part_of_speech.tag] == 'NOUN':
   				nouns += token.text.content + ", "
   			if pos_tag[token.part_of_speech.tag] == 'VERB':
   				verbs += token.text.content + ", "
   	nouns = nouns[0:-2]
   	verbs = verbs[0:-2]

 	# nouns = []
 	# for token in tokens:
  #  		if pos_tag[token.part_of_speech.tag] == 'NOUN':
  #  			nouns.extend([token.text.content])

  #  	d = cmudict.dict()
  #  	def nsyl(word):
  #  		return [len(list(y for y in x if y[-1].isdigit())) for x in d[word.lower()]]

  #  	syllabs = ""
  #  	for syl in nouns:
  #  		if nsyl(syl) >= 2:
  #  			syllabs += syl + ", "
  #  	syllabs = syllabs[0:-2]

	json_transcript_data = {
		"start_time": data.start_time.to_sec(),
		"end_time": data.end_time.to_sec(),
		"speech_duration": data.speech_duration.to_sec(),
		"received_time": data.received_time.to_sec(),
		"transcript": str(data.transcript),
		"confidence": float(data.confidence),
		"pid": int(data.pid),
		"sentiment": response.document_sentiment.score,
		"nouns": nouns,
		"verbs": verbs,
		"recv-end": data.received_time.to_sec() - data.end_time.to_sec()
	}

	# json_transcript_data = {
	# 	"start_time": data.start_time.to_sec(),
	# 	"end_time": data.end_time.to_sec(),
	# 	"speech_duration": data.speech_duration.to_sec(),
	# 	"received_time": data.received_time.to_sec(),
	# 	"transcript": str(data.transcript),
	# 	"confidence": float(data.confidence),
	# 	"pid": int(data.pid),
	# 	"sentiment": response.document_sentiment.score,
	# 	"important nouns": syllabs,
	# 	"rec-end": data.received_time.to_sec() - data.end_time.to_sec()
	# }
	print "Sending: %s" % json.dumps(json_transcript_data)

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
					+ "\"nouns\": " + "\"" + "\"" + "," \
					+ "\"verbs\": " + "\"" + "\"" + "," \
					+ "\"recv-end\": " + "0" \
					+ "}"
	print "Sending start_utterance: %s" % start_indicator

if __name__ == '__main__':
	# credentials = service_account.Credentials.from_service_account_file('/home/scazlab/Documents/team_meeting_project_2018/catkin_ws/src/team_meeting_project/ros-speech2text-google-stt-cred')
	# language_client = language.LanguageServiceClient(credentials=credentials)

	language_client = language.LanguageServiceClient()

	rospy.init_node('stt_message_sender', anonymous = True)
	# rospy.Subscriber("speech_to_text/start_utterance", start_utterance, callback_start_utterance)
	# rospy.Subscriber("/speech_to_text/transcript", transcript, callback_speech_transcript)
	establish_tablet_connection()
	rospy.spin()