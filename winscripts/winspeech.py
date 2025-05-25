import socket
import json
import os
import time
import pyaudio
from vosk import Model, KaldiRecognizer

model_path = ""
host = ""
port = 65434


class VoiceCommandClient:
    VALID_COMMANDS = {"forward", "backward", "left", "right", "stop","slow","fast","increase speed","decrease speed"}
    

    def __init__(self, model_path, host, port, silence_timeout=5):
        self.model_path = model_path
        self.host = host
        self.port = port
        self.silence_timeout = silence_timeout

        self.model = None
        self.recognizer = None
        self.stream = None
        self.socket = None

    def load_model(self):
        if not os.path.exists(self.model_path):
            raise FileNotFoundError(f"Model not found at: {self.model_path}")
        print("[INFO] Loading Vosk model...")
        valid_words=json.dumps(["forward", "backward", "left", "right", "stop","slow","fast","increase speed","decrease speed"])
        self.model = Model(self.model_path)
        self.recognizer = KaldiRecognizer(self.model, 16000,valid_words)

    def setup_audio_stream(self):
        p = pyaudio.PyAudio()
        self.stream = p.open(
            format=pyaudio.paInt16,
            channels=1,
            rate=16000,
            input_device_index=2, # -------------------------------------------------------------------------------------------------------
            input=True,
            frames_per_buffer=8000
        )
        self.stream.start_stream()
        print("[INFO] Microphone stream started...")

    def connect_to_server(self):
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.socket.connect((self.host, self.port))
        print(f"[INFO] Connected to {self.host}:{self.port}")

    def listen_and_send(self):
        print("[INFO] Listening for commands…")
        last_spoken_time = time.time()
        try:
            while True:
                data = self.stream.read(4000, exception_on_overflow=False)
                if self.recognizer.AcceptWaveform(data):
                    result = json.loads(self.recognizer.Result())
                    command = result.get("text", "").strip()
                    if command:
                        last_spoken_time = time.time()
                        if command in self.VALID_COMMANDS:
                            print(f"[CMD] You said: {command}")
                            self.socket.sendall(command.encode())
                        else:
                            print(f"[WARN] Unrecognized command: '{command}'")

                # Silence fallback
                if time.time() - last_spoken_time > self.silence_timeout:
                    print("[INFO] Waiting for command…")
                    last_spoken_time = time.time()  # reset to throttle messages

        except KeyboardInterrupt:
            print("\n[INFO] Stopped by user")
        finally:
            self.cleanup()

    def cleanup(self):
        if self.stream:
            self.stream.stop_stream()
            self.stream.close()
        if self.socket:
            self.socket.close()
        print("[INFO] Disconnected successfully.")


if __name__ == "__main__":

    client = VoiceCommandClient(model_path, host, port, silence_timeout=5)
    client.load_model()
    client.setup_audio_stream()
    client.connect_to_server()
    client.listen_and_send()