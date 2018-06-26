# Ros Speech2Text with Ubuntu 16.04 and updated Google Speech Client [![Build Status](https://travis-ci.org/ScazLab/ros_speech2text.svg?branch=master)](https://travis-ci.org/ScazLab/ros_speech2text) [![Issues](https://img.shields.io/github/issues/ScazLab/ros_speech2text.svg?label=Issues)](https://github.com/ScazLab/ros_speech2text/issues) [![Codacy Badge](https://api.codacy.com/project/badge/Grade/be514e5db92f4f96876c5b3afbffcd1f)](https://www.codacy.com/app/Baxter-collaboration/ros_speech2text?utm_source=github.com&amp;utm_medium=referral&amp;utm_content=ScazLab/ros_speech2text&amp;utm_campaign=Badge_Grade)

This was used with the "Team Meeting Project" using the Jibo robots. All information is current as of 2018-6-22.

A speech2text engine for ROS __(WARNING! This version on this branch uses Ubuntu 16.04 with ROS Kinetic and NOT Indigo)__, using the updated Google Cloud Speech API.

For setting up ROS and all that fun stuff look [here](https://alecive.github.io/ros_installation.html).
Just make sure to replace any instance of the word "indigo" with "kinetic" because kinetic is the version of ROS for Ubuntu 16.04
You should also cross reference [this](http://wiki.ros.org/kinetic/Installation/Ubuntu) as you go just to make sure anything kinetic specific is executed properly.

Just make sure that before trying to do `pip install pyaudio` you run this line first `sudo apt-get install libasound-dev portaudio19-dev libportaudio2 libportaudiocpp0`

__NOTE:__ "SAMSON STAGE PXD1" microphones were used in our project, and if you want to use them again, open up the back and use the screwdriver to set the gain to in between the fourth and fifth tick from the bottom. This sets the sensitivity to a place that easily detects the wearer's voice but not other sounds/voices. But test this yourself because depending on the surroundings, you may need higher or lower sensitivity. Also, make sure to have the mics directly facing your mouth and not to the side of your mouth, or you may get unpleasant results.

For using the updated Google-cloud speech-to-text API, take a look at these pages (navigating Google's documentation can be kind of annoying sometimes):

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
