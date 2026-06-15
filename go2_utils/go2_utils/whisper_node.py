#!/usr/bin/env python3

"""
Decode the Go2 audio stream, segment it into utterances with energy-based
VAD endpointing, denoise each utterance with DeepFilterNet, and transcribe
with faster-whisper.

Waits for you to speak and pause, then transcribes the whole utterance once.
Transcription runs on a separate thread so listening is never blocked
"""

import audioop
import queue
import threading
from collections import deque

import numpy as np
import opuslib
import rclpy
import torch
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String
from unitree_go.msg import AudioData


class DeepFilterDenoiser:
    """DeepFilterNet wrapper. Cleans mono 48 kHz audio.

    atten_lim_db caps the maximum attenuation so consonants survive
    (12 dB worked well against the Go2 lidar noise).
    """

    def __init__(self, atten_lim_db=None):
        from df.enhance import enhance, init_df

        self._enhance = enhance
        self.model, self.df_state, _ = init_df()
        self.sample_rate = self.df_state.sr()  # 48000
        self.atten_lim_db = atten_lim_db

    def enhance(self, mono_i16):
        audio = torch.from_numpy(mono_i16.astype(np.float32) / 32768.0).unsqueeze(0)
        cleaned = self._enhance(
            self.model, self.df_state, audio, atten_lim_db=self.atten_lim_db
        )
        return cleaned.squeeze(0).cpu().numpy()


class Go2WhisperNode(Node):
    def __init__(self):
        super().__init__("go2_whisper_node")

        self.declare_parameter("audio_topic", "/audiosender")
        self.declare_parameter("text_topic", "/go2/whisper/text")
        self.declare_parameter("model_name", "base.en")
        self.declare_parameter("language", "en")

        # Energy VAD endpointing. min_rms must sit ABOVE the lidar floor
        # (~600), or every frame reads as speech and the pause is never found.
        self.declare_parameter("min_rms", 800)
        self.declare_parameter("endpoint_silence", 0.6)   # trailing silence (s)
        self.declare_parameter("min_utterance", 0.3)      # ignore shorter (s)
        self.declare_parameter("max_utterance", 4.0)      # force-flush cap (s)
        self.declare_parameter("min_text_length", 2)
        # Require this many consecutive loud frames (20 ms each) to start an
        # utterance, so a single noise blip doesn't trigger it.
        self.declare_parameter("speech_start_frames", 2)
        # Audio kept before the trigger so soft word onsets aren't clipped.
        self.declare_parameter("pre_roll_ms", 300)
        # Peak-normalize the denoised utterance to lift quiet speech.
        self.declare_parameter("normalize", True)
        # Drop an utterance if Whisper's no_speech_prob exceeds this.
        self.declare_parameter("max_no_speech", 0.7)

        # faster-whisper engine.
        self.declare_parameter("device", "")              # "" = auto
        self.declare_parameter("compute_type", "")        # "" = auto
        self.declare_parameter("beam_size", 2)
        self.declare_parameter("no_repeat_ngram_size", 3)
        self.declare_parameter("vad_filter", False)       # faster internal VAD

        # DeepFilterNet noise reduction.
        self.declare_parameter("enable_denoise", True)
        self.declare_parameter("atten_lim_db", 12.0)      # 0 = no limit

        # Empty by default: a command list as initial_prompt makes Whisper
        # parrot those words on noise.
        self.declare_parameter("initial_prompt", "")
        self.declare_parameter("debug", True)

        self.audio_topic = self.get_parameter("audio_topic").value
        self.text_topic = self.get_parameter("text_topic").value
        self.model_name = self.get_parameter("model_name").value
        self.language = self.get_parameter("language").value

        self.min_rms = int(self.get_parameter("min_rms").value)
        self.endpoint_silence = float(self.get_parameter("endpoint_silence").value)
        self.min_utterance = float(self.get_parameter("min_utterance").value)
        self.max_utterance = float(self.get_parameter("max_utterance").value)
        self.min_text_length = int(self.get_parameter("min_text_length").value)
        self.speech_start_frames = int(self.get_parameter("speech_start_frames").value)
        self.pre_roll_ms = int(self.get_parameter("pre_roll_ms").value)
        self.normalize = bool(self.get_parameter("normalize").value)
        self.max_no_speech = float(self.get_parameter("max_no_speech").value)

        self.device_param = self.get_parameter("device").value
        self.compute_type = self.get_parameter("compute_type").value
        self.beam_size = int(self.get_parameter("beam_size").value)
        self.no_repeat_ngram_size = int(
            self.get_parameter("no_repeat_ngram_size").value
        )
        self.vad_filter = bool(self.get_parameter("vad_filter").value)

        self.enable_denoise = bool(self.get_parameter("enable_denoise").value)
        atten = float(self.get_parameter("atten_lim_db").value)
        self.atten_lim_db = atten if atten > 0.0 else None

        self.initial_prompt = self.get_parameter("initial_prompt").value or None
        self.debug = bool(self.get_parameter("debug").value)

        # Verified Go2 stream format.
        self.opus_rate = 48000
        self.opus_channels = 2
        self.opus_frame_size = 960     # 20 ms at 48 kHz
        self.frame_ms = 20
        self.whisper_rate = 16000

        self.decoder = opuslib.Decoder(self.opus_rate, self.opus_channels)

        self.pub = self.create_publisher(String, self.text_topic, 10)
        self.sub = self.create_subscription(
            AudioData,
            self.audio_topic,
            self.audio_callback,
            qos_profile_sensor_data,
        )

        self.audio_queue = queue.Queue()
        # Finished utterances wait here for transcription, so the slow
        # transcribe step never blocks the VAD thread from listening.
        self.utterance_queue = queue.Queue()
        self.running = True

        self.denoiser = None
        if self.enable_denoise:
            self.get_logger().info("Loading DeepFilterNet")
            self.denoiser = DeepFilterDenoiser(atten_lim_db=self.atten_lim_db)

        self.load_asr()

        self.worker = threading.Thread(target=self.worker_loop, daemon=True)
        self.worker.start()
        self.transcriber = threading.Thread(target=self.transcribe_loop, daemon=True)
        self.transcriber.start()

        self.get_logger().info("Listening on %s" % self.audio_topic)
        self.get_logger().info("Publishing text on %s" % self.text_topic)
        self.get_logger().info(
            "min_rms=%d endpoint_silence=%.2f beam_size=%d denoise=%s"
            % (self.min_rms, self.endpoint_silence, self.beam_size, self.enable_denoise)
        )

    def load_asr(self):
        import ctranslate2
        from faster_whisper import WhisperModel

        cuda_ok = ctranslate2.get_cuda_device_count() > 0
        device = self.device_param or ("cuda" if cuda_ok else "cpu")
        compute_type = self.compute_type or ("float16" if device == "cuda" else "int8")
        self.get_logger().info(
            "Loading faster-whisper '%s' on %s (%s)"
            % (self.model_name, device, compute_type)
        )
        self.fw_model = WhisperModel(
            self.model_name, device=device, compute_type=compute_type
        )

    def transcribe(self, audio_f32):
        segments, _ = self.fw_model.transcribe(
            audio_f32,
            language=self.language,
            beam_size=self.beam_size,
            condition_on_previous_text=False,
            no_speech_threshold=0.6,
            no_repeat_ngram_size=self.no_repeat_ngram_size,
            vad_filter=self.vad_filter,
            initial_prompt=self.initial_prompt,
        )
        segs = list(segments)
        text = " ".join(s.text for s in segs).strip()
        nsp = segs[0].no_speech_prob if segs else None
        return text, nsp

    def audio_callback(self, msg: AudioData):
        try:
            pcm48_stereo = self.decoder.decode(
                bytes(msg.data),
                self.opus_frame_size,
                decode_fec=False,
            )
        except Exception as exc:
            self.get_logger().warn("Opus decode failed: %s" % exc)
            return

        # Stereo 48 kHz int16 -> mono 48 kHz int16. Stay at 48 kHz so the
        # denoiser sees the full band; downsample after enhancement.
        pcm48_mono = audioop.tomono(pcm48_stereo, 2, 0.5, 0.5)
        self.audio_queue.put(np.frombuffer(pcm48_mono, dtype=np.int16))

    def resample_to_16k(self, mono_i16):
        out, _ = audioop.ratecv(
            mono_i16.tobytes(),
            2,
            1,
            self.opus_rate,
            self.whisper_rate,
            None,
        )
        return np.frombuffer(out, dtype=np.int16)

    def worker_loop(self):
        endpoint_ms = int(self.endpoint_silence * 1000)
        min_samples = int(self.min_utterance * self.opus_rate)
        max_samples = int(self.max_utterance * self.opus_rate)

        utterance = []
        utt_len = 0
        in_speech = False
        silence_ms = 0
        loud_run = 0
        pre_roll = deque(maxlen=max(1, self.pre_roll_ms // self.frame_ms))

        while self.running:
            try:
                frame = self.audio_queue.get(timeout=0.1)
            except queue.Empty:
                continue

            loud = audioop.rms(frame.tobytes(), 2) >= self.min_rms

            if not in_speech:
                # Wait for sustained loudness before starting, so a single
                # noise burst can't open an utterance. Keep a pre-roll so the
                # word onset isn't clipped once we do start.
                pre_roll.append(frame)
                loud_run = loud_run + 1 if loud else 0
                if loud_run >= self.speech_start_frames:
                    in_speech = True
                    silence_ms = 0
                    utterance = list(pre_roll)
                    utt_len = sum(len(f) for f in utterance)
                    pre_roll.clear()
                continue

            utterance.append(frame)
            utt_len += len(frame)
            silence_ms = 0 if loud else silence_ms + self.frame_ms

            if silence_ms >= endpoint_ms or utt_len >= max_samples:
                if utt_len >= min_samples:
                    # Hand off to the transcribe thread; never block listening.
                    self.utterance_queue.put(np.concatenate(utterance))
                utterance = []
                utt_len = 0
                in_speech = False
                silence_ms = 0
                loud_run = 0
                pre_roll.clear()

    def transcribe_loop(self):
        while self.running:
            try:
                audio48_i16 = self.utterance_queue.get(timeout=0.1)
            except queue.Empty:
                continue
            self.finalize(audio48_i16)

    def finalize(self, audio48_i16):
        try:
            raw_rms = audioop.rms(audio48_i16.tobytes(), 2)

            if self.denoiser is not None:
                cleaned_f32 = self.denoiser.enhance(audio48_i16)
            else:
                cleaned_f32 = audio48_i16.astype(np.float32) / 32768.0

            if self.normalize:
                peak = float(np.max(np.abs(cleaned_f32)))
                if peak > 0.0:
                    cleaned_f32 = cleaned_f32 * (0.95 / peak)

            cleaned48_i16 = np.clip(
                cleaned_f32 * 32768.0, -32768, 32767
            ).astype(np.int16)

            audio16 = self.resample_to_16k(cleaned48_i16)
            clean_rms = audioop.rms(audio16.tobytes(), 2)
            audio_f32 = audio16.astype(np.float32) / 32768.0

            text, nsp = self.transcribe(audio_f32)

            if self.debug:
                self.get_logger().info(
                    "utt %.1fs raw_rms=%d clean_rms=%d nsp=%s -> '%s'"
                    % (
                        len(audio48_i16) / self.opus_rate,
                        raw_rms,
                        clean_rms,
                        ("%.2f" % nsp) if nsp is not None else "na",
                        text,
                    )
                )

            # Reject what Whisper flags as non-speech (lidar noise reads ~0.9)
            # and anything too short to be a command.
            if nsp is not None and nsp > self.max_no_speech:
                return
            if len(text) < self.min_text_length:
                return

            msg = String()
            msg.data = text
            self.pub.publish(msg)

            if not self.debug:
                self.get_logger().info("Heard: %s" % text)

        except Exception as exc:
            self.get_logger().error("Transcription failed: %s" % exc)

    def destroy_node(self):
        self.running = False
        if hasattr(self, "worker") and self.worker.is_alive():
            self.worker.join(timeout=1.0)
        if hasattr(self, "transcriber") and self.transcriber.is_alive():
            self.transcriber.join(timeout=1.0)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = Go2WhisperNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
