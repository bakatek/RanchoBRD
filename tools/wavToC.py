import scipy.io.wavfile
import numpy as np

rate, data = scipy.io.wavfile.read("excited-2-101soundboards.wav")
assert rate == 44100 and data.dtype == np.int16 and len(data.shape) == 1
with open("../src/phares_pcm.h", "wt") as f:
    f.write("#include <stdint.h>\n\n")
    f.write("static const int16_t phares_pcm[] = {\n")
    for i, sample in enumerate(data):
        f.write(f"{sample}, ")
        if (i + 1) % 16 == 0:
            f.write("\n")
    f.write("\n};\n")
    f.write(f"static const uint32_t phares_pcm_size = {len(data)};\n")


rate, data = scipy.io.wavfile.read("1-screaming-101soundboards.wav")
assert rate == 44100 and data.dtype == np.int16 and len(data.shape) == 1
with open("../src/critical_pcm.h", "wt") as f:
    f.write("#include <stdint.h>\n\n")
    f.write("static const int16_t critical_pcm[] = {\n")
    for i, sample in enumerate(data):
        f.write(f"{sample}, ")
        if (i + 1) % 16 == 0:
            f.write("\n")
    f.write("\n};\n")
    f.write(f"static const uint32_t critical_pcm_size = {len(data)};\n")


rate, data = scipy.io.wavfile.read("excited-2-101soundboards.wav")
assert rate == 44100 and data.dtype == np.int16 and len(data.shape) == 1
with open("../src/startup_pcm.h", "wt") as f:
    f.write("#include <stdint.h>\n\n")
    f.write("static const int16_t startup_pcm[] = {\n")
    for i, sample in enumerate(data):
        f.write(f"{sample}, ")
        if (i + 1) % 16 == 0:
            f.write("\n")
    f.write("\n};\n")
    f.write(f"static const uint32_t startup_pcm_size = {len(data)};\n")


rate, data = scipy.io.wavfile.read("worried-101soundboards.wav")
assert rate == 44100 and data.dtype == np.int16 and len(data.shape) == 1
with open("../src/worried_pcm.h", "wt") as f:
    f.write("#include <stdint.h>\n\n")
    f.write("static const int16_t worried_pcm[] = {\n")
    for i, sample in enumerate(data):
        f.write(f"{sample}, ")
        if (i + 1) % 16 == 0:
            f.write("\n")
    f.write("\n};\n")
    f.write(f"static const uint32_t worried_pcm_size = {len(data)};\n")
    
rate, data = scipy.io.wavfile.read("18-101soundboards.wav")
assert rate == 44100 and data.dtype == np.int16 and len(data.shape) == 1
with open("../src/clignotant_pcm.h", "wt") as f:
    f.write("#include <stdint.h>\n\n")
    f.write("static const int16_t clignotant_pcm[] = {\n")
    for i, sample in enumerate(data):
        f.write(f"{sample}, ")
        if (i + 1) % 16 == 0:
            f.write("\n")
    f.write("\n};\n")
    f.write(f"static const uint32_t clignotant_pcm_size = {len(data)};\n")