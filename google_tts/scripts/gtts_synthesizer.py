import sys
from gtts import gTTS
import os

def synthesize(text, output_file):
    try:
        output_dir = os.path.dirname(output_file)
        print(f"Creating output directory: {output_dir}")
        os.makedirs(output_dir, exist_ok=True)
        
        # Verify directory creation
        if not os.path.exists(output_dir):
            print(f"Error: Failed to create directory {output_dir}")
            return 1
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