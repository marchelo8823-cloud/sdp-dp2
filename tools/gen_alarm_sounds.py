"""Генерация звуков аварийной сигнализации пульта (qml/snd/*.wav).

WARNING — два коротких гудка в цикле 1.5 с (предупреждение);
ALARM — четыре частых двухтональных в цикле 1.2 с (авария).
Чистый синтез, без внешних файлов: 22050 Гц, 16 бит, mono.
"""
import math
import os
import struct
import wave

SR = 22050
OUT = os.path.join(os.path.dirname(__file__), "..", "qml", "snd")


def tone(frames, freq, t0, dur, amp=0.5):
    """Наложить тон с плавными фронтами (10 мс) на буфер."""
    n0 = int(t0 * SR)
    n1 = min(len(frames), n0 + int(dur * SR))
    fade = int(0.010 * SR)
    for n in range(n0, n1):
        k = n - n0
        env = min(1.0, k / fade, (n1 - n) / fade)
        frames[n] += amp * env * math.sin(2 * math.pi * freq * n / SR)


def render(path, length_s, events):
    frames = [0.0] * int(length_s * SR)
    for freq, t0, dur, amp in events:
        tone(frames, freq, t0, dur, amp)
    with wave.open(path, "wb") as w:
        w.setnchannels(1)
        w.setsampwidth(2)
        w.setframerate(SR)
        w.writeframes(b"".join(
            struct.pack("<h", int(max(-1, min(1, f)) * 32767))
            for f in frames))
    print("записан", path)


def main():
    os.makedirs(OUT, exist_ok=True)
    # WARNING: bip-bip, пауза
    render(os.path.join(OUT, "warn.wav"), 1.5, [
        (660, 0.00, 0.15, 0.45),
        (660, 0.30, 0.15, 0.45),
    ])
    # ALARM: четыре частых двухтональных
    render(os.path.join(OUT, "alarm.wav"), 1.2, [
        (880, 0.00, 0.12, 0.55),
        (1046, 0.15, 0.12, 0.55),
        (880, 0.30, 0.12, 0.55),
        (1046, 0.45, 0.12, 0.55),
    ])


if __name__ == "__main__":
    main()
