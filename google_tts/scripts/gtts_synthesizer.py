import sys
from gtts import gTTS
import os

def synthesize(text, output_file):
    try:
        tts = gTTS(text, lang='en')
        tts.save(output_file)
        print(f"Saved synthesized speech to {output_file}")
        return 0
    except Exception as e:
        print(f"Error during speech synthesis: {e}")
        return 1

if __name__ == "__main__":
    if len(sys.argv) != 3:
        print("Usage: python3 gtts_synthesizer.py <text> <output_file>")
        sys.exit(1)

    text_to_speak = sys.argv[1]
    output_file = sys.argv[2]
    sys.exit(synthesize(text_to_speak, output_file))
