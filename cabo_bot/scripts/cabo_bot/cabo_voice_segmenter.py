import pyaudio
import numpy as np
import soundfile as sf
import whisper
import threading
import csv
from datetime import datetime

import os
os.environ['RCUTILS_CONSOLE_OUTPUT_FORMAT'] = '[{severity}] [{name}]: {message}'

class AudioSegmenter:
    def __init__(self):
        # Constants
        self.RATE = 32000  # Sample rate
        self.CHUNK = 1024  # Size of each audio chunk
        self.THRESHOLD_MULTIPLIER = 1.2  # Threshold for detecting command
        self.SEGMENT_WAIT_TIME = 1  # Time in seconds to wait before finalizing a segment
        self.PRE_SEGMENT_TIME = 0.3  # Time in seconds to include before the beginning of the segment
        self.AFT_SEGMENT_TIME = 0.3  # Time in seconds to include after the ending of the segment, exclude the waiting time
        self.MIN_SEGMENT_LENGTH = 1  # Minimum segment length in seconds, including the PRE and AFT time

        self.p = pyaudio.PyAudio()
        self.stream = None
        self.baseline = None
        self.all_audio_frames = []
        self.segment_audio_frames = []
        self.pre_segment_frames = []
        self.segments = []
        self.current_segment_start = None
        self.record_dir = "./voice_temp"
        os.makedirs(self.record_dir, exist_ok=True)

        self.model = whisper.load_model("base.en") 
        self.csv_file_path = os.path.expanduser('~/cabo_ros2/src/test_record/voice.csv')


    def initialize_stream(self):
        self.stream = self.p.open(format=pyaudio.paInt16,
                                  channels=1,
                                  rate=self.RATE,
                                  input=True,
                                  frames_per_buffer=self.CHUNK)
        self.baseline = self.__calculate_baseline(self.stream)

    def __calculate_baseline(self, stream, duration=5):
        print("Calibrating baseline noise level...")
        frames = []
        for _ in range(0, int(self.RATE / self.CHUNK * duration)):
            data = stream.read(self.CHUNK)
            frames.append(np.frombuffer(data, dtype=np.int16))
        baseline = np.mean([np.abs(frame).mean() for frame in frames])
        print(f"Baseline noise level: {baseline}")

        # 保存录音数据
        baseline_audio = np.concatenate(frames)
        baseline_path = os.path.join(self.record_dir, "baseline_audio.wav")
        sf.write(baseline_path, baseline_audio, self.RATE)

        # Whisper 模型识别
        result = self.model.transcribe(baseline_path, language="en")
        print(f"Recognized Text for Baseline: {result['text']}")
        os.remove(baseline_path)

        return baseline * 0.8

    def __is_command(self, audio_frames):
        return np.mean([np.abs(frame).mean() for frame in audio_frames]) > self.baseline * self.THRESHOLD_MULTIPLIER

    def __capture_audio(self):
        volume_history = []

        try:
            while True:
                data = self.stream.read(self.CHUNK)
                audio_frame = np.frombuffer(data, dtype=np.int16)
                self.all_audio_frames.append(audio_frame)

                # Maintain a buffer for pre-segment frames
                self.pre_segment_frames.append(audio_frame)
                if len(self.pre_segment_frames) > int(self.PRE_SEGMENT_TIME * self.RATE / self.CHUNK):
                    self.pre_segment_frames.pop(0)

                volume_history.append(np.abs(audio_frame).mean())
                if len(volume_history) > int(self.SEGMENT_WAIT_TIME * self.RATE / self.CHUNK):
                    volume_history.pop(0)

                avg_volume = np.mean(volume_history)

                # 清除前一行并输出新的噪音值
                print(f"\rCurrent noise level: {avg_volume:.2f}", end="")

                if avg_volume > self.baseline * self.THRESHOLD_MULTIPLIER:
                    if not self.segment_audio_frames:
                        self.current_segment_start = len(self.all_audio_frames) - len(self.pre_segment_frames)  # Start of the segment
                        # Include pre-segment frames
                        self.segment_audio_frames.extend(self.pre_segment_frames)
                    self.segment_audio_frames.append(audio_frame)
                elif self.segment_audio_frames:
                    self.segment_audio_frames.append(audio_frame)
                    if len(volume_history) == int(self.SEGMENT_WAIT_TIME * self.RATE / self.CHUNK):
                        if avg_volume < self.baseline * self.THRESHOLD_MULTIPLIER:
                            segment_end = len(self.all_audio_frames) - int(self.SEGMENT_WAIT_TIME * self.RATE / self.CHUNK)  # End of the segment

                            record_start = max(0, self.current_segment_start - int(self.PRE_SEGMENT_TIME * self.RATE / self.CHUNK))
                            record_end = min(len(self.all_audio_frames), segment_end + int(self.AFT_SEGMENT_TIME * self.RATE / self.CHUNK))
                            segment_length = (record_end - record_start) * self.CHUNK / self.RATE

                            if segment_length > self.MIN_SEGMENT_LENGTH:
                                start_time = datetime.now()
                                self.segments.append((record_start, record_end))

                                # Save the segment immediately
                                segment_audio = np.concatenate(self.all_audio_frames[record_start:record_end])
                                segment_path = os.path.join(self.record_dir, f"segment_{len(self.segments)}.wav")
                                sf.write(segment_path, segment_audio, self.RATE)

                                # 保留识别到的文字输出
                                print(f"\nSegment {len(self.segments)} length: {segment_length:.2f} seconds")
                                result = self.model.transcribe(segment_path, language="en")
                                print(f"Recognized Text for Segment {len(self.segments)}: {result['text']}")


                                command = self.on_text_recognized(result['text'])
                                os.remove(segment_path)


                                end_time = datetime.now()
                                time_diff = (end_time - start_time).total_seconds()

                                os.makedirs(os.path.dirname(self.csv_file_path), exist_ok=True)
                                with open(self.csv_file_path, mode='a', newline='') as csv_file:
                                    csv_writer = csv.writer(csv_file)
                                    csv_writer.writerow([
                                        f"{start_time}",
                                        f"{end_time}",
                                        f"{time_diff:.2f}",
                                        result['text'],
                                        command
                                    ])


                            # Remove already processed frames from all_audio_frames to save memory
                            self.all_audio_frames = self.all_audio_frames[record_end:]
                            self.segment_audio_frames = []  # Reset for the next segment
                            self.current_segment_start = None

                # Maintain only the recent 10 seconds of audio frames
                max_frames = int(self.RATE * 10 / self.CHUNK)
                if len(self.all_audio_frames) > max_frames:
                    self.all_audio_frames = self.all_audio_frames[-max_frames:]

        except KeyboardInterrupt:
            print("\nTerminating...")

    def live(self):
        print("Listening for commands in live mode...")
        self.__capture_audio()

    def start(self):
        threading.Thread(target=self.live).start()

    def on_text_recognized(self, text):
        pass


if __name__ == "__main__":
    segmenter = AudioSegmenter()
    segmenter.live()
