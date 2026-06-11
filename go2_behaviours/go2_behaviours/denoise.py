"""
Noise reduction for Go2 audio frames.

Applies a high-pass filter to cut lidar motor noise which sits below 200 Hz.
Applied immediately to every frame with no learning phase.
"""

import numpy as np
from scipy.signal import butter, sosfilt, sosfilt_zi

SAMPLE_RATE = 48000
HIGHPASS_CUTOFF_HZ = 200


class NoiseReducer:
    def __init__(self):
        self._sos = butter(6, HIGHPASS_CUTOFF_HZ / (SAMPLE_RATE / 2), btype='high', output='sos')
        self._zi  = sosfilt_zi(self._sos) * 0.0

    @property
    def learning(self):
        return False

    def process(self, mono):
        filtered, self._zi = sosfilt(self._sos, mono.astype(np.float32), zi=self._zi)
        return np.clip(filtered, -32768, 32767).astype(np.int16)
