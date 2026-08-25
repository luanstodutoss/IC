import serial
import numpy as np
import matplotlib.pyplot as plt

PORT = "COM7"
BAUDRATE = 115200
N_BINS = 512

fs_real = 1586  # calibrado

def main():
    try:
        ser = serial.Serial(PORT, BAUDRATE, timeout=2)
        print("Serial conectada!")
    except:
        print("Erro ao abrir porta. Verifique o cabo.")
        return

    plt.ion()
    fig, ax = plt.subplots()

    while True:
        line = ser.readline().decode('utf-8', errors='ignore').strip()

        if line:
            print("Recebido:", line)  # 🔍 DEBUG

        if "FFT_START" in line:  # 🔥 mais robusto
            print("Iniciando leitura FFT...")

            buffer = []

            while True:
                data = ser.readline().decode('utf-8', errors='ignore').strip()

                if data == "FFT_END":
                    print("Fim da FFT")
                    break

                try:
                    buffer.append(float(data))
                except:
                    continue

            print(f"Amostras recebidas: {len(buffer)}")

            if len(buffer) >= N_BINS:
                y = np.array(buffer[:N_BINS])
                y -= np.mean(y)

                mags = np.abs(np.fft.fft(y))[:N_BINS // 2]
                mags = (mags * 2.0) / N_BINS

                freqs = np.fft.fftfreq(N_BINS, d=1/fs_real)[:N_BINS // 2]

                peak_idx = np.argmax(mags)
                peak_freq = freqs[peak_idx]

                print(f"🔥 Pico: {peak_freq:.2f} Hz")

                ax.clear()
                ax.plot(freqs, mags)
                ax.set_title(f"Pico: {peak_freq:.2f} Hz")
                ax.set_xlim(0, fs_real / 2)

                plt.draw()
                plt.pause(0.1)
            else:
                print("❌ Dados insuficientes para FFT")

if __name__ == "__main__":
    main()