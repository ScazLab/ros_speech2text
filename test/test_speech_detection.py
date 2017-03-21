from unittest import TestCase

from array import array
from s2t.speech_detection import SpeechDetector, normalize, NORMAL_MAXIMUM


class TestNormalize(TestCase):

    def test_normalize_ones(self):
        a = array('h', [1, 1, 1, 1, 1])
        b = array('h', [NORMAL_MAXIMUM] * 5)
        self.assertEqual(normalize(a), b)

    def test_normalize_twos(self):
        a = array('h', [2, 2, 2, 2, 2])
        b = array('h', [NORMAL_MAXIMUM] * 5)
        self.assertEqual(normalize(a), b)

    def test_normalize(self):
        a = array('h', [10, 1, 0])
        b = array('h', [NORMAL_MAXIMUM, NORMAL_MAXIMUM // 10, 0])
        self.assertEqual(normalize(a), b)

    def test_normalize_already_normal(self):
        a = array('h', [NORMAL_MAXIMUM, NORMAL_MAXIMUM // 10, 0])
        self.assertEqual(normalize(a), a)


class TestAddSilence(TestCase):

    pass


class TestSpeechDetector(TestCase):

    def test_silent_is_silent(self):
        sd = SpeechDetector(100, 1.)
        self.assertTrue(sd.is_silent(array('h', [0, 0, 0, 0, 0])))

    def test_silent_is_silent_dynamic(self):
        sd = SpeechDetector(100, 1., dynamic_threshold=True)
        sd.avg_volume = 1.  # TODO change
        self.assertTrue(sd.is_silent(array('h', [0, 0, 0, 0, 0])))

    def test_above_static(self):
        sd = SpeechDetector(100, 1.)
        self.assertFalse(sd.is_silent(array('h', [0, 0, 2, 0, 0])))

    def test_above_static_but_not_dynamic(self):
        sd = SpeechDetector(100, 1., dynamic_threshold=True,
                            dynamic_threshold_percentage=50.)
        sd.avg_volume = 1.5
        self.assertTrue(sd.is_silent(array('h', [0, 0, 2, 0, 0])))

    def test_above_dynamic_but_not_static(self):
        sd = SpeechDetector(100, 3., dynamic_threshold=True,
                            dynamic_threshold_percentage=50.)
        sd.avg_volume = 1.
        self.assertFalse(sd.is_silent(array('h', [0, 0, 2, 0, 0])))

    # TODO continue...
