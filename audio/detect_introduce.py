import whisper
import re
import os
import sounddevice as sd
import numpy as np
from scipy.io.wavfile import write
from gtts import gTTS
import time

# Load the Whisper model (replace 'base' with the model size you're using)
model = whisper.load_model("small")

# List of names and drinks to match
names = [
    'Julia',
    'Emma',
    'Ema',
    'Sara',
    'Sarah',
    'Laura',
    'Susan',
    'John',
    'Lucas',
    'William',
    'Kevin',
    'Peter',
    'Robin',
    'Robbin',
    'Jeffery',
    'Jeffrey',
    'Tan',
    'Sarah',
    'Adam'
]
drinks = [
    'Hot Chocolate',
    'Coke',
    'Soy',
    'Water',
    'Lime juice',
    'Tea',
    'Coffee',
    'Soda',
    'Milk',
    'Latte',
    'T'
]


# Function to record audio from the microphone and save it as a wav file
def record_audio(duration=4, sample_rate=44100):
    print("Recording... Please speak.")
    audio_data = sd.rec(int(duration * sample_rate),
                        samplerate=sample_rate,
                        channels=1,
                        dtype='float32')
    sd.wait()  # Wait until the recording is finished
    audio_data = np.squeeze(audio_data)  # Remove single-dimensional entries
    write("recorded_audio.wav", sample_rate, audio_data)  # Save as WAV file
    print("Recording complete. Saved as 'recorded_audio.wav'")
    return "recorded_audio.wav"


# Function to transcribe audio using the local Whisper model
def transcribe_audio(audio_file_path):
    result = model.transcribe(audio_file_path, language='en')
    print(result['text'])
    return result['text']


# Function to announce the question about name using gTTS
def ask_name():
    announcement_text = "What is your name."
    tts = gTTS(text=announcement_text, lang='en', slow=False)
    tts.save("askname.mp3")
    # os.system("start askname.mp3")  # For Windows
    os.system("mpg321 askname.mp3")  # For Linux/macOS


# Function to announce the question about drink using gTTS
def ask_drink():
    announcement_text = "What is your favourite drink."
    tts = gTTS(text=announcement_text, lang='en', slow=False)
    tts.save("askdrink.mp3")
    # os.system("start askdrink.mp3")  # For Windows
    os.system("mpg321 askdrink.mp3")  # For Linux/macOS


# Function to announce detected name using gTTS
def announce_name(detected_name):
    announcement_text = f"Hello {detected_name}."
    tts = gTTS(text=announcement_text, lang='en', slow=False)
    tts.save("name_announcement.mp3")
    # os.system("start name_announcement.mp3")  # For Windows
    os.system("mpg321 name_announcement.mp3")  # For Linux/macOS


# Function to announce detected name using gTTS
def announce_drink(drink_choice, detected_name):
    announcement_text = f"Hello {detected_name} your favourite drink is {drink_choice}."
    tts = gTTS(text=announcement_text, lang='en', slow=False)
    tts.save("drink_announcement.mp3")
    # os.system("start name_announcement.mp3")  # For Windows
    os.system("mpg321 drink_announcement.mp3")  # For Linux/macOS


def annouce_follow():
    announcement_text = f"Follow me I will introduce you to the host"
    tts = gTTS(text=announcement_text, lang='en', slow=False)
    tts.save("final_audio.mp3")
    os.system("mpg321 final_audio.mp3")


# Function to detect names and keep asking until a correct name is matched
def detect_name():
    while True:
        ask_name()
        audio_file = record_audio(duration=5)  # Record 5 seconds of audio
        transcribed_text = transcribe_audio(audio_file)

        # Match any name from the list
        detected_names = [
            name for name in names if re.search(r'\b' + name + r'\b',
                                                transcribed_text,
                                                re.IGNORECASE)
        ]

        if detected_names:
            detected_name = detected_names[0]
            print(f"Detected name: {detected_name}")
            announce_name(detected_name)
            return detected_name
        else:
            print("No valid name detected. Please try again.")


# Function to detect drinks after name is detected
def detect_drink(detected_name):
    while True:
        ask_drink()
        audio_file = record_audio(duration=5)  # Record 5 seconds of audio
        transcribed_text = transcribe_audio(audio_file)

        # Match any drink from the list
        detected_drinks = [
            drink for drink in drinks if re.search(r'\b' + drink + r'\b',
                                                   transcribed_text,
                                                   re.IGNORECASE)
        ]

        if detected_drinks:
            drink_choice = detected_drinks[0]
            announce_drink(drink_choice, detected_name)
            print(f"Serving {drink_choice} to {detected_name}")
            return drink_choice
        else:
            print("No valid drink detected. Please try again.")


print('started')
time.sleep(7)
# Main program flow
detected_name = detect_name(
)  # Keep asking for name until a valid one is detected
detect_drink(detected_name)  # After detecting name, ask for the drink
annouce_follow()
