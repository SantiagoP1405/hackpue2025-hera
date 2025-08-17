import sounddevice as sd
import numpy as np
import torch
from speechbrain.inference.speaker import SpeakerRecognition
import torchaudio
import os

# Cambio: usar la nueva API de SpeechBrain 1.0
model = SpeakerRecognition.from_hparams(source="speechbrain/spkrec-ecapa-voxceleb")

REFERENCIAS_DIR = os.path.expanduser("~/.voz_identifier_refs/")
os.makedirs(REFERENCIAS_DIR, exist_ok=True)

def grabar(nombre, duracion=3, fs=16000):
    print(f"Grabando voz de {nombre} por {duracion} segundos...")
    audio = sd.rec(int(duracion * fs), samplerate=fs, channels=1, dtype='float32')
    sd.wait()
    archivo = os.path.join(REFERENCIAS_DIR, f"{nombre}.wav")
    
    # Cambio: convertir numpy array a tensor PyTorch
    audio_tensor = torch.from_numpy(audio[:, 0])
    
    # Cambio: usar torchaudio para guardar el archivo
    torchaudio.save(archivo, audio_tensor.unsqueeze(0), fs)
    print(f"Guardado en {archivo}")
    return archivo

def main():
    print("=== Grabador de Referencias de Voz ===")
    print("Se grabarán 3 voces de referencia (A, B, C)")
    input("Presiona Enter para comenzar...")
    
    for nombre in ["A", "B", "C"]:
        input(f"Presiona Enter para grabar la voz de la persona {nombre}...")
        grabar(nombre)
        print(f"Voz {nombre} grabada exitosamente!\n")
    
    print("¡Todas las referencias grabadas!")

if __name__ == '__main__':
    main()