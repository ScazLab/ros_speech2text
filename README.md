# Ros Speech2Text [![Build Status](https://travis-ci.org/ScazLab/ros_speech2text.svg?branch=master)](https://travis-ci.org/ScazLab/ros_speech2text) [![Issues](https://img.shields.io/github/issues/ScazLab/ros_speech2text.svg?label=Issues)](https://github.com/ScazLab/ros_speech2text/issues) [![Codacy Badge](https://api.codacy.com/project/badge/Grade/be514e5db92f4f96876c5b3afbffcd1f)](https://www.codacy.com/app/Baxter-collaboration/ros_speech2text?utm_source=github.com&amp;utm_medium=referral&amp;utm_content=ScazLab/ros_speech2text&amp;utm_campaign=Badge_Grade)

A speech2text engine for ROS, using the Google Cloud Speech API.

![Setup](https://cloud.githubusercontent.com/assets/7922534/23566041/32243298-001e-11e7-860e-6a187fd416e6.png)

## Prerequisites

 * `ros`: Any version newer than ROS Indigo should work.
 * `google cloud speech 0.22.0`: Available [here](https://pypi.python.org/pypi/google-cloud). Further instructions on API authentication can be found below.
 * `PyAudio>=0.2.9`: Python package for audio source fetching.
 * `svox_tts` : it's a SVOX-PICO based wrapper for text-to-speech. It's not necessary, but if you wish to see status messages on the screen of your robot, svox_tts is required. Available [here](https://github.com/ScazLab/svox_tts).

## Installation

Installation of the package follows the standard building procedure of ROS packages. The following instructions are for `catkin build`, even though the repository can be used and compile with the old `catkin_make` without issues.

 1. Compile the repo: `catkin build ros_speech2text`
 2. To test if the package is working, run `roslaunch ros_speech2text ros_speech2text_async.launch`.


### Note for pyAudio on Ubuntu 14.04

The packaged version of pyAudio on Trusty is `0.2.7`. Newer versions can be installed via `pip install pyaudio`. It however requires portaudio to be installed on the system which can be found [here](http://www.portaudio.com/download.html).


### Authentication Instructions

Authentication of the `Google Cloud Speech API` is done by setting an environmental variable. For instructions on obtaing an API credential, check [here](https://cloud.google.com/speech/docs/getting-started). The path of the API credential should be supplied in the launch file, see below for more instructions.

## Execution

### Initial steps (mainly for Scazlab students)

 0. Turn on the robot. Wait for the robot to finish its start-up phase.
 1. Be sure that the system you're running the code has access to the Baxter robot. This is usually done by running the `baxter.sh` script that should be provided in your Baxter installation. See [here](http://sdk.rethinkrobotics.com/wiki/Hello_Baxter#Source_ROS_Environment_Setup_Script) for more info. **@ScazLab students** → for what concerns the Baxter robot on the ScazLab, this means that every time you have to run some ROS software to be used on the robot you should open a new terminal, and do the following: ` cd ros_devel_ws && ./baxter.sh `. A change in the terminal prompt should acknowledge that you now have access to `baxter.local`. __Please be aware of this issue when you operate the robot__.
 2. Untuck the robot. **@ScazLab students** → we have an alias for this, so you just have to type `untuck`

### Launch file parameters

#### Public ROS params
* `/ros_speech2text/speech_history`: location of the speech history for the session
* `GOOGLE_APPLICATION_CREDENTIALS`: sets environment variable for Google Cloud APIs to work

#### Private ROS params
* `audio_device_idx`: device ID of audio source.
* `audio_rate`: rate for your audio capturing device
* `audio_threshold`: volume threshold for static thresholding
* `enable_dynamic_threshold`: param for dynamic thresholding
* `audio_dynamic_percentage`: activate audio recording when volume is this percentage higher than average
* `audio_dynamic_frame`: for x consecutive frames all louder than the percentage we specified, activate recording
* `audio_min_avg`: min value of average volume to prevent system from being too sensitive in case of constantly quiet environments
* `speech_context`: list of context clues for speech recognition

### Recognition modes
#### Synchronous Recognition
The synchronous recognition mode can be launched by `roslaunch ros_speech2text ros_speech2text_sync.launch`. In the synchronous mode, after a sentence input is completed, the system makes a blocking API call, and all audio input is halted until the recognition results are returned from the server.

#### Asynchronous Recognition
The synchronous recognition mode can be launched by `roslaunch ros_speech2text ros_speech2text_async.launch`. A separate thread in this mode polls the results of the async API calls repeatedly, while the main thread keeps on capturing audio and recording sentence.

### Misc
The results of recognition is published to the topic `/ros_speech2text/user_output` with the custom message type `transcript`.

## Troubleshooting
1. What if after `catkin build`, it seems like the ROS package still cannot be found?

   Run `catkin clean` and `rospack profile`, and try to build the package again.
2. What if I don't know the device ID of my audio source?

   Run the node once, and use rosparam to get the param `/ros_speech2text/available_audio_device`. The devices are sorted by device ID starting from zero.
3. Can I have multiple instances running at the same time?

   Yes, the private parameters can help you configure different audio sources for different nodes.

