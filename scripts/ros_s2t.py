#!/usr/bin/env python


from struct import pack
from std_msgs.msg import String
from google.cloud import speech
from ros_speech2text.msg import transcript
from s2t.speech_detection import SpeechDetector

import pyaudio
import wave
import io
import os
import sys
import rospy
import thread
import csv

SPEECH_HISTORY_DIR = None
FORMAT = pyaudio.paInt16
run_flag = True
OPERATION_QUEUE = []
pub_screen = None


def recog(async_mode, speech_client, sn, context, rate):
    """
    Constructs a recog operation with the audio file specified by sn
    The operation is an asynchronous api call
    """
    file_name = 'sentence' + str(sn) + '.wav'
    file_path = os.path.join(SPEECH_HISTORY_DIR, file_name)
    with io.open(file_path, 'rb') as audio_file:
        content = audio_file.read()
        audio_sample = speech_client.sample(
            content,
            source_uri=None,
            encoding='LINEAR16',
            sample_rate=rate)

    if async_mode:
        operation = speech_client.speech_api.async_recognize(sample=audio_sample,
                                                         speech_context=context)
        return operation
    else:
        alternatives = speech_client.speech_api.sync_recognize(sample = audio_sample,
            speech_context = context)
        for alternative in alternatives:
            return alternative.transcript,alternative.confidence
    


def record_to_file(sample_width, data, sn, rate):
    """
    Saves the audio content in data into a file with sn as a suffix of file name
    """
    data = pack('<' + ('h' * len(data)), *data)
    file_name = 'sentence' + str(sn) + '.wav'
    file_path = os.path.join(SPEECH_HISTORY_DIR, file_name)
    wf = wave.open(file_path, 'wb')
    wf.setnchannels(1)
    wf.setsampwidth(sample_width)
    wf.setframerate(rate)
    wf.writeframes(data)
    wf.close()
    rospy.loginfo('file saved')


def expand_dir(speech_history_dir):
    """
    A function that expands directories so python can find the folder
    """
    speech_history_dir = os.path.expanduser(
        os.path.join(speech_history_dir, str(os.getpid())))
    if not os.path.isdir(speech_history_dir):
        os.makedirs(speech_history_dir)
    return speech_history_dir

def generate_msg(text,confidence,start_time,end_time,pub_text,pub_screen,writer):
    msg = transcript()
    msg.start_time = start_time
    msg.end_time = end_time
    msg.speech_duration = start_time-end_time
    msg.received_time = rospy.get_rostime()
    msg.transcript = text
    msg.confidence = confidence
    rospy.logwarn("%s,confidence:%f" % (text, confidence))
    pub_text.publish(msg)
    pub_screen.publish(text)
    writer.writerow([msg.start_time,msg.end_time,msg.speech_duration,
        msg.transcript,msg.confidence])
    return msg

def check_operation(pub_text, pub_screen, writer):
    """
    This function is intended to be run as a seperate thread that repeatedly
    checks if any recog operation has finished.
    The transcript returned is then published on screen of baxter and sent
    to the ros topic with the custom message type 'transcript'.
    """
    global OPERATION_QUEUE
    while not rospy.is_shutdown():
        for op in OPERATION_QUEUE[:]:
            if op[0].complete:
                for result in op[0].results:
                    msg = generate_msg(result.transcript,result.confidence,op[1],op[2],pub_text,pub_screen,writer)
                OPERATION_QUEUE.remove(op)
            else:
                try:
                    op[0].poll()
                except ValueError:
                    rospy.logerr("No good results returned!")
                    OPERATION_QUEUE.remove(op)
        rospy.sleep(1)


def cleanup():
    """
    Cleans up speech history directory after session ends
    """
    cleanup=rospy.get_param(rospy.get_name()+'/cleanup', True)
    if not cleanup:
        return
    speech_directory = SPEECH_HISTORY_DIR
    for file in os.listdir(speech_directory):
        file_path = os.path.join(speech_directory, file)
        try:
            os.remove(file_path)
        except Exception as e:
            rospy.logerr(e)
    os.rmdir(speech_directory)

def utterance_start():
    # pass
    # rospy.loginfo('utterance_start')
    pub_screen.publish("Sentence Started")

def utterance_end():
    # pass
    rospy.logwarn('audio segment completed')
    pub_screen.publish("Recognizing")


def main():
    global SPEECH_HISTORY_DIR
    global FORMAT
    global OPERATION_QUEUE
    global pub_screen

    # Setting up ros params
    rospy.init_node('speech2text_engine', anonymous=True)
    node_name = rospy.get_name()
    pub_text = rospy.Publisher(node_name+'/user_output', transcript, queue_size=10)
    pub_screen = rospy.Publisher('/svox_tts/speech_output', String, queue_size=10)
    async_mode = rospy.get_param(node_name+'/async_mode', True)
    rate = rospy.get_param(node_name+'/audio_rate', 16000)
    dynamic_thresholding = rospy.get_param(node_name+'/enable_dynamic_threshold', True)
    if not dynamic_thresholding:
        threshold = rospy.get_param(node_name+'/audio_threshold', 700)
    else:
        threshold = rospy.get_param(node_name+'/audio_dynamic_percentage', 50)

    SPEECH_HISTORY_DIR = rospy.get_param('/ros_speech2text/speech_history', '~/.ros/ros_speech2text/speech_history')
    SPEECH_HISTORY_DIR = expand_dir(SPEECH_HISTORY_DIR)
    input_idx = rospy.get_param(node_name+'/audio_device_idx', None)

    speech_detector = SpeechDetector(
        rate,
        threshold,
        dynamic_threshold=dynamic_thresholding,
        dynamic_threshold_frame=rospy.get_param(node_name+'/audio_dynamic_frame', 3),
        min_average_volume=rospy.get_param(node_name+'/audio_min_avg', 100),
        verbose=rospy.get_param(node_name+'/verbosity', True)
    )

    """
    Set up PyAudio client, and fetch all available devices
    Get input device ID from ros param, and attempt to use that device as audio source
    """
    p = pyaudio.PyAudio()
    device_list = [p.get_device_info_by_index(i)['name'] for i in range(p.get_device_count())]
    rospy.set_param('/ros_speech2text/available_audio_device', device_list)

    if input_idx is None:
        input_idx = p.get_default_input_device_info()['index']

    try:
        rospy.loginfo("Using device: " + p.get_device_info_by_index(input_idx)['name'])
        stream = p.open(format=FORMAT, channels=1, rate=rate,
                        input=True, start=False, input_device_index=input_idx,
                        output=False,
                        frames_per_buffer=speech_detector.chunk_size)
    except IOError:
        rospy.logerr("Invalid device ID. Available devices listed in rosparam /ros_speech2text/available_audio_device")
        p.terminate()
        sys.exit(1)
    sample_width = p.get_sample_size(FORMAT)

    speech_client = speech.Client()
    sn = 0

    csv_file = open(os.path.join(SPEECH_HISTORY_DIR,'transcript'),'wb')
    writer = csv.writer(csv_file,delimiter=' ',)
    writer.writerow(['start','end','duration','transcript','confidence'])

    """
    Start thread for checking operation results.
    Operations are stored in the global variable OPERATION_QUEUE
    """
    thread.start_new_thread(check_operation, (pub_text, pub_screen, writer))

    """
    Main loop for fetching audio and making operation requests.
    """
    while not rospy.is_shutdown():
        aud_data, start_time, end_time = speech_detector.get_next_utter(
            stream, utterance_start,utterance_end)
        if aud_data is None:
            rospy.loginfo("Node terminating")
            break
        record_to_file(sample_width, aud_data, sn, speech_detector.rate)
        context = rospy.get_param(node_name+'/speech_context', [])
        if async_mode:
            operation = recog(async_mode, speech_client, sn, context, speech_detector.rate)
            OPERATION_QUEUE.append([operation, start_time, end_time])
        else:
            transc,confidence = recog(async_mode, speech_client, sn, context, speech_detector.rate)
            generate_msg(transc,confidence,start_time,end_time,pub_text,pub_screen,writer)
        sn += 1

    stream.close()
    p.terminate()
    csv_file.close()
    cleanup()


if __name__ == '__main__':
    main()
