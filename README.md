# Ros Speech2Text [![Issues](https://img.shields.io/github/issues/ScazLab/ros_speech2text.svg?label=Issues)](https://github.com/ScazLab/ros_speech2text/issues)

A speech2text engine for ROS, using the Google Cloud Speech API.

![Setup](https://cloud.githubusercontent.com/assets/7922534/23566041/32243298-001e-11e7-860e-6a187fd416e6.png)

## Prerequisites

 * `ros`: Any version newer than ROS Indigo should work.
 * `google cloud speech 0.22.0`: Available [here](https://pypi.python.org/pypi/google-cloud). Further instructions on API authentication can be found below.
 * `PyAudio 0.2.7`: Python package for audio source fetching. 
 * `svox_tts` : it's a SVOX-PICO based wrapper for text-to-speech. It's not necessary, but if you wish to see status messages on the screen of your robot, svox_tts is required. Available [here](https://github.com/ScazLab/svox_tts).

## Installation

Installation of the package follows the standard building procedure of ROS packages. The following instructions are for `catkin build`, even though the repository can be used and compile with the old `catkin_make` without issues.

 1. Compile the repo: `catkin build ros_speech2text`
 2. To test if the package is working, run `roslaunch ros_speech2text ros_speech2text_async.launch`.
 
## Execution

### Initial steps (mainly for Scazlab students)

 0. Turn on the robot. Wait for the robot to finish its start-up phase.
 1. Be sure that the system you're running the code has access to the Baxter robot. This is usually done by running the `baxter.sh` script that should be provided in your Baxter installation. See [here](http://sdk.rethinkrobotics.com/wiki/Hello_Baxter#Source_ROS_Environment_Setup_Script) for more info. **@ScazLab students** → for what concerns the Baxter robot on the ScazLab, this means that every time you have to run some ROS software to be used on the robot you should open a new terminal, and do the following: ` cd ros_devel_ws && ./baxter.sh `. A change in the terminal prompt should acknowledge that you now have access to `baxter.local`. __Please be aware of this issue when you operate the robot__.
 2. Untuck the robot. **@ScazLab students** → we have an alias for this, so you just have to type `untuck`

### Authentication Instructions

Authentication of the `Google Cloud Speech API` is done by setting an environmental variable. For instructions on obtaing an API credential, check [here](https://cloud.google.com/speech/docs/getting-started). The path of the API credential should be supplied in the launch file, see below for more instructions.

### Launch file parameters

### Recognition modes

### Misc

## Troubleshooting
1. What if after `catkin build`, it seems like the ROS package still cannot be found?
   Run `catkin clean` and `rospack profile`, and try to build the package again.
