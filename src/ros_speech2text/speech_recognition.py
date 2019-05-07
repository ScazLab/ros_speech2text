import os
import io
import csv
import shutil
from datetime import datetime
from struct import pack
from threading import Thread
import time

import wave
import pyaudio
# in order to use the enable_automatic_punctuation, use the beta import
# seems to be ever so slightly worse at correctly recognizing speech than the non beta version
from google.cloud import speech_v1p1beta1 as speech
from google.cloud.speech_v1p1beta1 import enums
from google.cloud.speech_v1p1beta1 import types
# from google.cloud import speech as speech
# from google.cloud.speech import enums
# from google.cloud.speech import types
from google.gax.errors import RetryError



import rospy
from std_msgs.msg import String, Header
from ros_speech2text.msg import transcript, event, start_utterance

from .speech_detection import SpeechDetector


FORMAT = pyaudio.paInt16

def list_audio_devices(pyaudio_handler):
    device_list = [pyaudio_handler.get_device_info_by_index(i)['name']
                   for i in range(pyaudio_handler.get_device_count())]
    rospy.logdebug('Available devices:' + ''.join(
        ['\n  - [%d]: %s' % d for d in enumerate(device_list)]))
    rospy.set_param('/ros_speech2text/available_audio_device', device_list)
    return device_list


class SpeechRecognizer(object):

    # This is for the rospy Publisher and Subscriber
    # The beginning of every message has the "prefix" '/speech_to_text'
    TOPIC_BASE = '/speech_to_text'

    class InvalidDevice(ValueError):
        pass

    def __init__(self):
        self.node_name = rospy.get_name()
        self.pid = rospy.get_param(self.node_name + '/pid', -1)
        self._init_history_directory()
        self.print_level = rospy.get_param(self.node_name + '/print_level', 0)
        # ROS message for the speech transcript
        self.pub_transcript = rospy.Publisher(
            self.TOPIC_BASE + '/transcript', transcript, queue_size=10)
        # ROS message for speech utterance started
        self.pub_start = rospy.Publisher(
            self.TOPIC_BASE + '/start_utterance', start_utterance, queue_size=10)
        self.pub_text = rospy.Publisher(
            self.TOPIC_BASE + '/text', String, queue_size=10)
        self.pub_event = rospy.Publisher(
            self.TOPIC_BASE + '/log', event, queue_size=10)
        self.sample_rate = rospy.get_param(self.node_name + '/audio_rate', 16000)
        self.async = rospy.get_param(self.node_name + '/async_mode', False)
        dynamic_thresholding = rospy.get_param(
            self.node_name + '/enable_dynamic_threshold', False)
        if not dynamic_thresholding:
            threshold = rospy.get_param(self.node_name + '/audio_threshold', 575)
        else:
            threshold = rospy.get_param(
                self.node_name + '/audio_dynamic_percentage', 50)
        # decide whether to use the start_utterance messages
        self.enable_start_utterance = rospy.get_param(
                self.node_name + '/enable_start_utterance', False)
        # new instance of the SpeechDetector class found in speech_detection.py
        self.speech_detector = SpeechDetector(
            self.sample_rate,
            threshold,
            dynamic_threshold=dynamic_thresholding,
            dynamic_threshold_frame=rospy.get_param(
                self.node_name + '/audio_dynamic_frame', 3),
            min_average_volume=rospy.get_param(
                self.node_name + '/audio_min_avg', 100),
            num_silent=rospy.get_param(
                self.node_name + '/n_silent_chunks', 10),
        )
        # sig_non_silence is mainly to keep track of whether or not the
        # start utterance message should be sent
        self.sig_non_silence = False
        if self.print_level > 0:
            rospy.loginfo('Sample Rate: {}'.format(self.sample_rate))

        self._init_stream()
        self._init_csv()
        self.speech_client = speech.SpeechClient()
        self.run()

    def _init_history_directory(self):
        param = rospy.get_param(self.node_name + '/speech_history',
                                '~/.ros/ros_speech2text/speech_history')
        date_and_time = datetime.now().isoformat()
        datetime_and_pid_folder_name = date_and_time + "_pid_" + str(self.pid)
        self.history_dir = os.path.expanduser(os.path.join(param, datetime_and_pid_folder_name))
        if not os.path.isdir(self.history_dir):
            os.makedirs(self.history_dir)

    def _init_stream(self):
        self.pa_handler = pyaudio.PyAudio()
        device_list = list_audio_devices(self.pa_handler)
        input_idx = rospy.get_param(self.node_name + '/audio_device_idx', None)
        input_name = rospy.get_param(self.node_name + '/audio_device_name', None)
        if input_idx is None:
            input_idx = self.pa_handler.get_default_input_device_info()['index']
            if input_name is not None:
                try:
                    # use first found for name
                    input_idx = [input_name.lower() in d.lower()
                                 for d in device_list
                                 ].index(True)
                except ValueError:
                    self.terminate()
                    raise self.InvalidDevice(
                        "No device found for name '%s'." % input_name)
        try:
            rospy.loginfo("{} using device: {}".format(
                self.node_name,
                self.pa_handler.get_device_info_by_index(input_idx)['name'])
            )
            self.stream = self.pa_handler.open(
                format=FORMAT, channels=1, rate=self.sample_rate, input=True,
                start=False, input_device_index=input_idx, output=False,
                frames_per_buffer=self.speech_detector.chunk_size)
        except IOError:
            self.terminate()
            raise self.InvalidDevice(
                'Invalid device ID: {}. Available devices listed in rosparam '
                '/ros_speech2text/available_audio_device'.format(input_idx))
        self.sample_width = self.pa_handler.get_sample_size(FORMAT)

    def _init_csv(self):
        self.csv_file = open(os.path.join(self.history_dir, 'transcript'), 'wb')
        self.csv_writer = csv.writer(self.csv_file, delimiter=' ',)
        self.csv_writer.writerow(['utterance_id', 'start', 'end', 'duration', 'transcript', 'confidence'])

    def run(self):
        # sn is only really used for saving the audio files under different names
        sn = 0
        # do we want to use the start_utterance messages
        if (self.enable_start_utterance):
            # don't keep checking if you should send start_utterance, only when you need to
            start_speech = True
            # declare aud_data up here so you can keep track of the recording from before the start utterance message is sent
            aud_data = None
            while not rospy.is_shutdown():
                aud_data, start_time, end_time, sig_non_silence = self.speech_detector.get_next_utter(
                    aud_data, self.stream, start_speech, *self.get_utterance_start_end_callbacks(sn))
                if aud_data is None:
                    rospy.loginfo("No more data, exiting...")
                    break
                # if it's the first significant period of sound, send the start utterance message
                if sig_non_silence & start_speech:
                    start_speech = False
                    try:
                        self.get_utterance_started()
                    except Exception as e:
                        rospy.logerr("Error in starting utterance: {}".format(e))
                # not the first significant period of sound, send the whole message
                else:
                    start_speech = True
                    self.record_to_file(aud_data, sn)
                    aud_data = None
                    # print(sn)
                    try:
                        transc, confidence = self.recog(sn)
                        self.utterance_decoded(sn, transc, confidence, start_time, end_time)
                    except Exception as e:
                        rospy.logerr("Error in recognition: {}".format(e))
                sn += 1
        else:
            while not rospy.is_shutdown():
                aud_data = None
                start_speech = False
                aud_data, start_time, end_time, sig_non_silence = self.speech_detector.get_next_utter(
                    aud_data, self.stream, start_speech, *self.get_utterance_start_end_callbacks(sn))
                if aud_data is None:
                    rospy.loginfo("No more data, exiting...")
                    break
                else:
                    self.record_to_file(aud_data, sn)
                    aud_data = None
                    # print(sn)
                    try:
                        transc, confidence = self.recog(sn)
                        self.utterance_decoded(sn, transc, confidence, start_time, end_time)
                    except Exception as e:
                        rospy.logerr("Error in recognition: {}".format(e))
                sn += 1
        self.terminate()

    def terminate(self):
        if hasattr(self, "stream"):
            self.stream.close()
        if hasattr(self, "pa_handler"):
            self.pa_handler.terminate()
        if hasattr(self, "csv_file"):
            self.csv_file.close()
        if (hasattr(self, "history_dir") and
                rospy.get_param(rospy.get_name() + '/cleanup', True)):
            shutil.rmtree(self.history_dir)

    def utterance_start(self, utterance_id):
        if self.print_level > 1:
            rospy.loginfo('Utterance started')
        self.pub_event.publish(
            self.get_event_base_message(event.STARTED, utterance_id))

    def utterance_end(self, utterance_id):
        if self.print_level > 1:
            rospy.loginfo('Utterance completed')
        self.pub_event.publish(
            self.get_event_base_message(event.STOPPED, utterance_id))

    def get_utterance_start_end_callbacks(self, utterance_id):
        def start(): #The streaming_recognize() method converts speech data to possible text alternatives on the fly.
            self.utterance_start(utterance_id)

        def end():
            self.utterance_end(utterance_id)

        return start, end

    def utterance_decoded(self, utterance_id, transcription, confidence,
                          start_time, end_time):
        transcript_msg = self.get_transcript_message(transcription, confidence,
                                                     start_time, end_time)
        event_msg = self.get_event_base_message(event.DECODED, utterance_id)
        event_msg.transcript = transcript_msg
        if self.print_level > 0:
            rospy.loginfo("{} [confidence: {}]".format(transcription, confidence))
        self.pub_transcript.publish(transcript_msg)
        self.pub_text.publish(transcription)
        self.pub_event.publish(event_msg)
        self.csv_writer.writerow([
            utterance_id, start_time, end_time, transcript_msg.speech_duration,
            transcription, confidence])

    # Send message through ROS that utterance has started
    def get_utterance_started(self):
        utterance_start_pid = self.get_start_utterance()
        self.pub_start.publish(utterance_start_pid)

    def get_transcript_message(self, transcription, confidence, start_time,
                               end_time):
        msg = transcript()
        msg.start_time = start_time
        msg.end_time = end_time
        msg.speech_duration = end_time - start_time
        msg.received_time = rospy.get_rostime()
        msg.transcript = transcription
        msg.confidence = confidence
        msg.pid = self.pid
        return msg

    def get_start_utterance(self):
        msg = start_utterance()
        msg.pid = self.pid
        return msg

    def get_event_base_message(self, evt, utterance_id):
        msg = event()
        msg.header = Header()
        msg.header.stamp = rospy.Time.now()
        msg.event = evt
        msg.utterance_id = utterance_id
        msg.audio_path = self.utterance_file(utterance_id)
        return msg

    def utterance_file(self, utterance_id):
        file_name = 'utterance_{}.wav'.format(utterance_id)
        return os.path.join(self.history_dir, file_name)

    def record_to_file(self, data, utterance_id):
        """Saves audio data to a file"""
        data = pack('<' + ('h' * len(data)), *data)
        path = self.utterance_file(utterance_id)
        wf = wave.open(path, 'wb')
        wf.setnchannels(1)
        wf.setsampwidth(self.sample_width)
        wf.setframerate(self.sample_rate)
        wf.writeframes(data)
        wf.close()
        rospy.logdebug('File saved to {}'.format(path))

    def recog(self, utterance_id):
        context = rospy.get_param(self.node_name + '/speech_context', [])
        path = self.utterance_file(utterance_id)

        with io.open(path, 'rb') as audio_file:
            content = audio_file.read()

        audio = types.RecognitionAudio(content=content)
        config = types.RecognitionConfig(
            encoding='LINEAR16',
            # make sure the sampling rate specified in the microphone node
            sample_rate_hertz=self.sample_rate,
            language_code='en-US',
            # the automatic punctuation is only available for the beta version
            enable_automatic_punctuation=True)

        if self.async:
            try:
               operation = self.speech_client.long_running_recognize(config, audio)
               op_result = operation.result()
               for result in op_result.results:
                  for alternative in result.alternatives:
                     print(self.pid)
                     print(alternative)
                     return alternative.transcript, alternative.confidence
            except Exception as e:
                rospy.logerr("Error in asynchronous recognition: {}".format(e))
        else:
            try:
                response = self.speech_client.recognize(config, audio)
                if response.results is not None:
                    for result in response.results:
                        if result.alternatives[0].transcript is not None:
                            print(self.pid)
                            print(result.alternatives[0])
                            return result.alternatives[0].transcript, result.alternatives[0].confidence
            except Exception as e:
                rospy.logerr("Error in synchronous recognition: {}".format(e))