"""
Noise reduction for Go2 audio frames.

Buffers audio into 2-second chunks then applies stationary spectral noise
gating via noisereduce. The first chunk is used as the noise profile (keep
quiet for the first 2 seconds). Subsequent chunks are denoised and flushed.
"""

import numpy as np

SAMPLE_RATE      = 48000
CHUNK_SAMPLES    = SAMPLE_RATE * 2   # 2 seconds per chunk
NOISE_CHUNKS     = 1                 # first chunk = noise profile


class NoiseReducer:
    def __init__(self):
        import noisereduce  # verify installed

        self._buffer      = []
        self._noise_clip  = None
        self._chunks_seen = 0

    @property
    def learning(self):
        return self._noise_clip is None

    def process_frame(self, mono):
        """Buffer one 960-sample frame. Returns denoised chunk when ready, else None."""
        self._buffer.append(mono.copy())

        total = sum(len(f) for f in self._buffer)
        if total < CHUNK_SAMPLES:
            return None

        chunk = np.concatenate(self._buffer).astype(np.float32)
        self._buffer = []
        self._chunks_seen += 1

        if self._chunks_seen <= NOISE_CHUNKS:
            self._noise_clip = chunk
            return None  # discard first chunk — it's the noise profile

        import noisereduce as nr
        reduced = nr.reduce_noise(
            y=chunk,
            sr=SAMPLE_RATE,
            y_noise=self._noise_clip,
            stationary=True,
            prop_decrease=0.6,
            n_fft=2048,
            freq_mask_smooth_hz=500,
        )
        return np.clip(reduced, -32768, 32767).astype(np.int16)
