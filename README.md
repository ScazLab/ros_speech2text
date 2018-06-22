# Ros Speech2Text with Ubuntu 16.04 and updated Google Speech Client [![Build Status](https://travis-ci.org/ScazLab/ros_speech2text.svg?branch=master)](https://travis-ci.org/ScazLab/ros_speech2text) [![Issues](https://img.shields.io/github/issues/ScazLab/ros_speech2text.svg?label=Issues)](https://github.com/ScazLab/ros_speech2text/issues) [![Codacy Badge](https://api.codacy.com/project/badge/Grade/be514e5db92f4f96876c5b3afbffcd1f)](https://www.codacy.com/app/Baxter-collaboration/ros_speech2text?utm_source=github.com&amp;utm_medium=referral&amp;utm_content=ScazLab/ros_speech2text&amp;utm_campaign=Badge_Grade)

A speech2text engine for ROS __(WARNING! This version on this branch uses Ubuntu 16.04 with Ros Kinetic and NOT Indigo)__, using the updated Google Cloud Speech API.

__NOTE:__ When using the "SAMSON STAGE PXD1" microphones, open up the back and use the screwdriver to set the gain to in between the third and fourth tick from the bottom. This sets the sensitivity to a place that easily detects the wearer's voice but not other sounds/voices.

For using the updated Google-cloud speech-to-text API, take a look at these pages (navigating Google's documentation can be kind of annoying sometimes):
https://google-cloud-python.readthedocs.io/en/latest/speech/index.html
https://cloud.google.com/speech-to-text/docs/python-client-migration
https://google-cloud-python.readthedocs.io/en/latest/speech/gapic/v1/api.html
https://google-cloud-python.readthedocs.io/en/latest/speech/gapic/v1p1beta1/api.html
https://cloud.google.com/speech-to-text/docs/reference/rpc/google.cloud.speech.v1#google.cloud.speech.v1.LongRunningRecognizeResponse
https://cloud.google.com/speech-to-text/docs/basics

For information on how to analyze the transcript for things like getting the sentiment of the sentence or grabbing the nouns and verbs of the sentence, look here:
https://cloud.google.com/natural-language/docs/basics
https://cloud.google.com/natural-language/docs/analyzing-syntax
https://google-cloud-python.readthedocs.io/en/latest/language/usage.html
