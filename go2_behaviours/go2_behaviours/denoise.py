"""
Noise reduction for Go2 audio frames.

Learns the noise profile from the first ~2 seconds of audio,
then applies per-frame stationary noise reduction to remove
the Go2's motor/lidar noise (dominant below 300 Hz).
"""

import numpy as np
from scipy.signal import butter, filtfilt

SAMPLE_RATE = 48000
FRAME_SIZE = 960
NOISE_LEARN_SECONDS = 3.0
NOISE_LEARN_FRAMES = int(NOISE_LEARN_SECONDS * SAMPLE_RATE / FRAME_SIZE)
HIGHPASS_CUTOFF_HZ = 300  # lidar motor noise sits below this


class NoiseReducer:
    def __init__(self):
        import noisereduce  # noqa: F401 — verify installed

        self._noise_frames = []
        self._noise_clip = None
        self._learning = True

        nyq = SAMPLE_RATE / 2
        b, a = butter(4, HIGHPASS_CUTOFF_HZ / nyq, btype='high')
        self._hp_b = b
        self._hp_a = a

    def _learn_noise(self, mono):
        self._noise_frames.append(mono.copy())
        if len(self._noise_frames) >= NOISE_LEARN_FRAMES:
            self._noise_clip = np.concatenate(self._noise_frames).astype(np.float32)
            self._learning = False
            self._noise_frames = []

    @property
    def learning(self):
        return self._learning

    def process(self, mono):
        import noisereduce as nr

        if self._learning:
            self._learn_noise(mono)
            return mono

        # high-pass filter to cut lidar motor hum below 300 Hz
        filtered = filtfilt(self._hp_b, self._hp_a, mono.astype(np.float32))

        # spectral noise gating on top of the filtered signal
        reduced = nr.reduce_noise(
            y=filtered,
            sr=SAMPLE_RATE,
            y_noise=self._noise_clip,
            stationary=True,
            prop_decrease=0.9,
            n_fft=min(len(mono), 1024),
        )
        return np.clip(reduced, -32768, 32767).astype(np.int16)
