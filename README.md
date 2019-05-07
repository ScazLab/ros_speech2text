# Ros Speech2Text with Ubuntu 16.04 and updated Google Speech Client [![Build Status](https://travis-ci.org/ScazLab/ros_speech2text.svg?branch=master)](https://travis-ci.org/ScazLab/ros_speech2text) [![Issues](https://img.shields.io/github/issues/ScazLab/ros_speech2text.svg?label=Issues)](https://github.com/ScazLab/ros_speech2text/issues) [![Codacy Badge](https://api.codacy.com/project/badge/Grade/be514e5db92f4f96876c5b3afbffcd1f)](https://www.codacy.com/app/Baxter-collaboration/ros_speech2text?utm_source=github.com&amp;utm_medium=referral&amp;utm_content=ScazLab/ros_speech2text&amp;utm_campaign=Badge_Grade)

This code was updated and modified by Nick Chang and Sarah Sebo for use in the Jibo survival studies run in 2018 and 2019 under the supervision of Sarah Sebo. All information is current as of 2019-5-07.

A speech2text engine for ROS __(WARNING! This version on this branch uses Ubuntu 16.04 with ROS Kinetic and NOT Indigo)__, using the updated Google Cloud Speech API.

# Setup and Depdencies

## ROS (Kinetic Kame)

Follow the [ROS installation instructions for Kinetic Kame](http://wiki.ros.org/kinetic).

## Setup a catkin workspace with this (ros_speech2text) ROS package

Clone this ROS package in `catkin_ws/src` and remember to `catkin_make` and source your `devel/setup.bash` file. 

Once you have made sure that you have built and sourced your new catkin_ws, make sure to make your scripts executable. In terminal, go to the location of your python scripts. For this specific project, these are located in `scripts` folders. Once there, run `chmod +x [name of file]` and open a new terminal and you should be able to run using roslaunch.

## Pip

Make sure you have pip installed, for Python2 the command is: `sudo apt install python-pip`

You may also run into some problems with `pip` if you are starting from scratch (i.e. fresh installation of Ubuntu 16.04). One error I (Nick) got a bunch was `AttributeError: 'module' object has no attribute 'SSL_ST_INIT'`. To fix this, just run `sudo python -m easy_install --upgrade pyOpenSSL` and it should be fine.

For most all other warnings and errors, you should be able to just do something like `sudo pip install [name] --upgrade`.

## Pyaudio

You must install pyaudio and its dependencies: 
```
sudo apt-get install libasound-dev portaudio19-dev libportaudio2 libportaudiocpp0
sudo pip install pyaudio
```

## Google Cloud Speech + Google Cloud Credentials

You'll need to install [google cloud speech](https://cloud.google.com/speech-to-text/docs/reference/libraries#client-libraries-install-python): `pip install --upgrade google-cloud-speech`. 

You'll also need to get a JSON google application credentials key from your google cloud account. You'll put this file in your `ros_speech2text` package and link to it in the launch files. 

# Notes on some of the microphones we used

__NOTE:__ "SAMSON STAGE PXD1" microphones were used in our project, and if you want to use them again, open up the back and use the screwdriver to set the gain to in between the fourth and fifth tick from the bottom. This sets the sensitivity to a place that easily detects the wearer's voice but not other sounds/voices. But test this yourself because depending on the surroundings, you may need higher or lower sensitivity. Also, make sure to have the mics directly facing your mouth and not to the side of your mouth, or you may get unpleasant results.

When creating individual nodes for the SAMSON mics, use the option to set the mics based on their names and not their numerical id. The numerical id often changes and sometimes just straight up doesn't work. So using the name "hw:__#__,0" works much better. If in the case that you run the launch file and keep getting an error something along the lines of unable to find the mic, then just close all your open terminals and restart roscore and relaunch the files. You shouldn't need to do this, but also this should always work (given that all your code is correct)

# Running in terminal:
In terminal, make sure you run `roscore` first before trying to run the other files

Once you have `roscore` up and running, open another terminal window or tab run `roslaunch ros_speech2text ros_speech2text.launch` to run with only one mic or `roslaunch ros_speech2text ros_speech2text_[2, 3, or 4]mics.launch` depending on how many mics you want to run with. For example if I want to run with 3 mics, I would run `roslaunch ros_speech2text ros_speech2text_3mics.launch`

__Warning:__
In the case that the mics become, for whatever reason, out of order (i.e. mic 1 is no longer associated with pid 1), then you can either plug in the mics again in the right oder or change the names that each node in the launch file is looking for. The names could also reset, and you'll need to run `rosparam get /ros_speech2text/available_audio_device/` in order to determine the names of the mics that are plugged into the computer. 

If you want to control whether or not to use the start_utterance messages, look in the `ros_speech2text` launch files and find the parameter `enable_start_utterance`. This message had the contents `"!"` and was used to communicate when someone starts speaking. 

# Using the updated Google-cloud speech-to-text API (Useful links from Nick Chang)
Take a look at these pages (navigating Google's documentation can be kind of annoying sometimes):

[first place to look](https://cloud.google.com/speech-to-text/docs/basics)

[second place to look](https://google-cloud-python.readthedocs.io/en/latest/speech/index.html)

[migration from old Google API](https://cloud.google.com/speech-to-text/docs/python-client-migration)

[stable version of speech client](https://google-cloud-python.readthedocs.io/en/latest/speech/gapic/v1/api.html)

[documentation on methods](https://cloud.google.com/speech-to-text/docs/reference/rpc/google.cloud.speech.v1)

[beta version of client with added functionality like auto punctuation](https://google-cloud-python.readthedocs.io/en/latest/speech/gapic/v1p1beta1/api.html)

[auto punctuation documentation](https://cloud.google.com/speech-to-text/docs/automatic-punctuation)
(I don't really notice a difference in terms of recognition speed, so it could be cool to keep testing this out)

For information on how to analyze the transcript for things like getting the sentiment of the sentence or grabbing the nouns and verbs of the sentence, look here:

[first place to look](https://cloud.google.com/natural-language/docs/basics)

[second place to look](https://google-cloud-python.readthedocs.io/en/latest/language/usage.html)

[for analyzing syntax](https://cloud.google.com/natural-language/docs/analyzing-syntax)
