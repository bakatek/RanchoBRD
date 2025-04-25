import numpy as np

sample_rate = 44100
duration = 0.9
note1 = 369.99  # F#4
note2 = 184.99  # F#3

t = np.linspace(0, duration, int(sample_rate * duration), endpoint=False)
signal = 0.65 * np.sin(2 * np.pi * note1 * t) + 0.35 * np.sin(2 * np.pi * note2 * t)
attack_samples = int(0.05 * sample_rate)
decay_samples = int(0.85 * sample_rate)
envelope = np.ones_like(signal)
envelope[:attack_samples] = np.linspace(0.1, 0.6, attack_samples)
envelope[attack_samples:] = np.linspace(0.6, 0.05, decay_samples)
signal *= envelope
pcm = (signal * 32767).astype(np.int16)

with open("../src/bong_pcm.h", "w") as f:
    f.write("static const int16_t bong_pcm[] = {\n")
    for i, sample in enumerate(pcm):
        f.write(f"{sample}, ")
        if (i + 1) % 16 == 0:
            f.write("\n")
    Stevenf.write("\n};\n")
    f.write(f"static const uint32_t bong_pcm_size = {len(pcm)};\n")