"""
Noise reduction for Go2 audio frames.

Two-stage approach:
    1. High-pass filter at 200 Hz to cut lidar motor hum
    2. Stationary spectral noise gating via noisereduce
"""

import numpy as np
from scipy.signal import butter, sosfilt, sosfilt_zi

SAMPLE_RATE = 48000
HIGHPASS_CUTOFF_HZ = 200
NOISE_LEARN_SECONDS = 3.0
NOISE_LEARN_FRAMES = int(NOISE_LEARN_SECONDS * SAMPLE_RATE / 960)


class NoiseReducer:
    def __init__(self):
        import noisereduce  # verify installed

        self._sos = butter(6, HIGHPASS_CUTOFF_HZ / (SAMPLE_RATE / 2), btype='high', output='sos')
        self._zi  = sosfilt_zi(self._sos) * 0.0

        self._noise_frames = []
        self._noise_clip   = None
        self._learning     = True

    @property
    def learning(self):
        return self._learning

    def process(self, mono):
        import noisereduce as nr

        filtered, self._zi = sosfilt(self._sos, mono.astype(np.float32), zi=self._zi)

        if self._learning:
            self._noise_frames.append(filtered.copy())
            if len(self._noise_frames) >= NOISE_LEARN_FRAMES:
                self._noise_clip = np.concatenate(self._noise_frames).astype(np.float32)
                self._noise_frames = []
                self._learning = False
            return np.clip(filtered, -32768, 32767).astype(np.int16)

        reduced = nr.reduce_noise(
            y=filtered,
            sr=SAMPLE_RATE,
            y_noise=self._noise_clip,
            stationary=True,
            prop_decrease=0.85,
            n_fft=512,
        )

        # ensure output length always matches input length
        out = np.zeros(len(mono), dtype=np.float32)
        copy_len = min(len(reduced), len(mono))
        out[:copy_len] = reduced[:copy_len]

        return np.clip(out, -32768, 32767).astype(np.int16)
