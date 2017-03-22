from unittest import TestCase

import numpy as np

from s2t.speech_detection import (StaticSilenceDetector,
                                  DynamicSilenceDetector,
                                  normalize, NORMAL_MAXIMUM)


class TestNormalize(TestCase):

    def test_normalize_ones(self):
        a = np.ones((5,))
        b = NORMAL_MAXIMUM * a
        np.testing.assert_array_equal(normalize(a), b)

    def test_normalize_twos(self):
        a = 2 * np.ones((5,))
        b = np.ones((5,)) * NORMAL_MAXIMUM
        np.testing.assert_array_equal(normalize(a), b)

    def test_normalize(self):
        a = np.array([10, 1, 0])
        b = np.array([NORMAL_MAXIMUM, NORMAL_MAXIMUM // 10, 0])
        np.testing.assert_array_equal(normalize(a), b)

    def test_normalize_already_normal(self):
        a = np.array([NORMAL_MAXIMUM, NORMAL_MAXIMUM // 10, 0])
        np.testing.assert_array_equal(normalize(a), a)

    def test_normalize_result_is_int(self):
        a = (100 * np.random.random((5,))).astype(np.int16)
        self.assertEqual(normalize(a).dtype, np.int16)


class TestAddSilence(TestCase):

    pass


class TestSilenceDetector(TestCase):

    def test_silent_is_silent(self):
        sd = StaticSilenceDetector(100, 1.)
        a = np.array([0, 0, 0, 0, 0], dtype=np.int16)
        self.assertTrue(sd.is_silent(a))

    def test_silent_is_silent_dynamic(self):
        sd = DynamicSilenceDetector(100)
        sd.avg_volume = 1.  # TODO change
        a = np.array([0, 0, 0, 0, 0], dtype=np.int16)
        self.assertTrue(sd.is_silent(a))

    def test_above_static(self):
        sd = StaticSilenceDetector(100, 1.)
        a = np.array([0, 0, 2, 0, 0], dtype=np.int16)
        self.assertFalse(sd.is_silent(a))

    def test_above_static_but_not_dynamic(self):
        sd = DynamicSilenceDetector(100, 50.)
        sd.avg_volume = 1.5
        a = np.array([0, 0, 2, 0, 0], dtype=np.int16)
        self.assertTrue(sd.is_silent(a))

    def test_above_dynamic_but_not_static(self):
        sd = DynamicSilenceDetector(100, 50.)
        sd.avg_volume = 1.
        a = np.array([0, 0, 2, 0, 0], dtype=np.int16)
        self.assertFalse(sd.is_silent(a))
