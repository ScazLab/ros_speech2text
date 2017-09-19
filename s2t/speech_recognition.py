import os
import io
import csv
import shutil
from struct import pack
from threading import Thread

import wave
import pyaudio
from google.cloud import speech
from google.gax.errors import RetryError

import rospy
from std_msgs.msg import String
from ros_speech2text.msg import transcript

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

    TOPIC_BASE = '/speech_to_text'

    class InvalidDeviceID(ValueError):
        pass

    def __init__(self):
        self._init_history_directory()
        self.node_name = rospy.get_name()
        self.pub_transcript = rospy.Publisher(self.TOPIC_BASE + '/transcript',
                                              transcript, queue_size=10)
        self.pub_text = rospy.Publisher(self.TOPIC_BASE + '/text', String,
                                        queue_size=10)
        self.sample_rate = rospy.get_param(self.node_name + '/audio_rate', 16000)
        self.async = rospy.get_param(self.node_name + '/async_mode', True)
        dynamic_thresholding = rospy.get_param(
            self.node_name + '/enable_dynamic_threshold', True)
        if not dynamic_thresholding:
            threshold = rospy.get_param(self.node_name + '/audio_threshold', 700)
        else:
            threshold = rospy.get_param(self.node_name + '/audio_dynamic_percentage', 50)
        self.speech_detector = SpeechDetector(
            self.sample_rate,
            threshold,
            dynamic_threshold=dynamic_thresholding,
            dynamic_threshold_frame=rospy.get_param(
                self.node_name + '/audio_dynamic_frame', 3),
            min_average_volume=rospy.get_param(
                self.node_name + '/audio_min_avg', 100),
            verbose=rospy.get_param(self.node_name + '/verbosity', True)
        )
        self._init_stream()
        self._init_csv()
        self.speech_client = speech.Client()
        self.run()

    def _init_history_directory(self):
        param = rospy.get_param('/ros_speech2text/speech_history',
                                '~/.ros/ros_speech2text/speech_history')
        self.history_dir = os.path.expanduser(os.path.join(param, str(os.getpid())))
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
                    rospy.logwarn(
                        "No device found for name '%s', falling back to default."
                        % input_name)
        try:
            rospy.loginfo("Using device: {}".format(
                self.pa_handler.get_device_info_by_index(input_idx)['name']))
            self.stream = self.pa_handler.open(
                format=FORMAT, channels=1, rate=self.sample_rate, input=True,
                start=False, input_device_index=input_idx, output=False,
                frames_per_buffer=self.speech_detector.chunk_size)
        except IOError:
            self.terminate()
            raise self.InvalidDeviceID(
                'Invalid device ID: {}. Available devices listed in rosparam '
                '/ros_speech2text/available_audio_device'.format(input_idx))
        self.sample_width = self.pa_handler.get_sample_size(FORMAT)

    def _init_csv(self):
        self.csv_file = open(os.path.join(self.history_dir, 'transcript'), 'wb')
        self.csv_writer = csv.writer(self.csv_file, delimiter=' ',)
        self.csv_writer.writerow(['start', 'end', 'duration', 'transcript', 'confidence'])

    def run(self):
        sn = 0
        if self.async:
            self.operation_queue = []
            thread = Thread(target=self.check_operation)
            thread.start()
        while not rospy.is_shutdown():
            aud_data, start_time, end_time = self.speech_detector.get_next_utter(
                self.stream, self.utterance_start, self.utterance_end)
            if aud_data is None:
                rospy.loginfo("Node terminating")
                break
            self.record_to_file(aud_data, sn)
            if self.async:
                operation = self.recog(sn)
                self.operation_queue.append([operation, start_time, end_time])
            else:
                transc, confidence = self.recog(sn)
                self.utterance_decoded(transc, confidence, start_time, end_time)
            sn += 1

    def terminate(self, exitcode=0):
        self.stream.close()
        self.pa_handler.terminate()
        self.csv_file.close()
        if rospy.get_param(rospy.get_name() + '/cleanup', True):
            shutil.rmtree(self.history_dir)

    def utterance_start(self):
        self.pub_text.publish("Sentence Started")

    def utterance_end(self):
        rospy.loginfo('audio segment completed')
        self.pub_text.publish("Recognizing")

    def utterance_decoded(self, transcription, confidence, start_time, end_time):
        msg = transcript()
        msg.start_time = start_time
        msg.end_time = end_time
        msg.speech_duration = end_time - start_time
        msg.received_time = rospy.get_rostime()
        msg.transcript = transcription
        msg.confidence = confidence
        rospy.logwarn("{} [confidence: {}]".format(transcription, confidence))
        self.pub_transcript.publish(msg)
        self.pub_text.publish(transcription)
        self.csv_writer.writerow([msg.start_time, msg.end_time, msg.speech_duration,
                                  msg.transcript, msg.confidence])

    def sentence_file(self, sentence_id):
        file_name = 'sentence_{}.wav'.format(sentence_id)
        return os.path.join(self.history_dir, file_name)

    def record_to_file(self, data, sentence_id):
        """Saves audio data to a file"""
        data = pack('<' + ('h' * len(data)), *data)
        path = self.sentence_file(sentence_id)
        wf = wave.open(path, 'wb')
        wf.setnchannels(1)
        wf.setsampwidth(self.sample_width)
        wf.setframerate(self.sample_rate)
        wf.writeframes(data)
        wf.close()
        rospy.logdebug('File saved to {}'.format(path))

    def recog(self, sn):
        """
        Constructs a recog operation with the audio file specified by sn
        The operation is an asynchronous api call
        """
        context = rospy.get_param(self.node_name + '/speech_context', [])
        path = self.sentence_file(sn)

        with io.open(path, 'rb') as audio_file:
            content = audio_file.read()
            audio_sample = self.speech_client.sample(
                content,
                source_uri=None,
                encoding='LINEAR16',
                sample_rate=self.sample_rate)

        if self.async:
            try:
                operation = self.speech_client.speech_api.async_recognize(
                    sample=audio_sample, speech_context=context)
            except (ValueError, RetryError):
                rospy.logerr("Audio Segment too long. Unable to recognize")
            return operation
        else:
            alternatives = self.speech_client.speech_api.sync_recognize(
                sample=audio_sample, speech_context=context)
            for alternative in alternatives:
                return alternative.transcript, alternative.confidence

    def check_operation(self):
        """
        This function is intended to be run as a seperate thread that repeatedly
        checks if any recog operation has finished.
        The transcript returned is then published on screen of baxter and sent
        to the ros topic with the custom message type 'transcript'.
        """
        while not rospy.is_shutdown():
            try:
                for op in self.operation_queue[:]:
                    if op[0].complete:
                        for result in op[0].results:
                            self.utterance_decoded(
                                result.transcript, result.confidence, op[1], op[2])
                        self.operation_queue.remove(op)
                    else:
                        try:
                            op[0].poll()
                        except ValueError:
                            rospy.logerr("No good results returned!")
                            self.operation_queue.remove(op)
            except Exception as e:
                rospy.logerr("Error in speech recognition thread: {}".format(e))
                self.operation_queue = []
            rospy.sleep(1)
