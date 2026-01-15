import math
import struct
import os

def create_wav_file(filename, freq):
    sample_rate = 44100
    # Generate a buffer that loops smoothly
    duration_target = 0.5 # Longer duration for better quality
    num_cycles = max(1, int(freq * duration_target))
    num_samples = int((sample_rate * num_cycles) / freq)
    
    audio = bytearray()
    factor = 2.0 * math.pi * freq / sample_rate
    for i in range(num_samples):
        val = int(32767.0 * math.sin(factor * i))
        audio.extend(struct.pack('<h', val))
    
    # WAV Header 
    header = struct.pack('<4sI4s', b'RIFF', 36 + len(audio), b'WAVE')
    header += struct.pack('<4sIHHIIHH', b'fmt ', 16, 1, 1, sample_rate, sample_rate * 2, 2, 16)
    header += struct.pack('<4sI', b'data', len(audio))
    
    with open(filename, 'wb') as f:
        f.write(header + audio)
    print(f"Generated {filename} for {freq}Hz")

def main():
    freqs = {
        '440': 440,
        '540': 540,
        '640': 640,
        '740': 740,
        '840': 840,
    }

    if not os.path.exists("wavs"):
        os.makedirs("wavs")

    for name, freq in freqs.items():
        filename = f"wavs/{freq}.wav"
        create_wav_file(filename, freq)

if __name__ == "__main__":
    main()
