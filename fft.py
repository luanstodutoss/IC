import serial
import numpy as np
import matplotlib
matplotlib.use('Agg')  # Sem janela, só salva arquivo
import matplotlib.pyplot as plt
from datetime import datetime
import os

# ─── Configurações ────────────────────────────────────────────
PORT      = "COM4"       # Windows: "COM3" | Linux/Mac: "/dev/ttyUSB0"
BAUDRATE  = 115200
OUTPUT_DIR = "fft_images"
REAL_ODR  = 970.0
N_BINS    = 512
# ──────────────────────────────────────────────────────────────

os.makedirs(OUTPUT_DIR, exist_ok=True)

def save_fft_png(magnitudes, peak_freq, rms, bin_resolution, index):
    freqs = np.arange(N_BINS) * bin_resolution

    fig, ax = plt.subplots(figsize=(14, 6))
    fig.patch.set_facecolor('#0d1117')
    ax.set_facecolor('#0d1117')

    # Espectro
    ax.plot(freqs, magnitudes, color='#00d4ff', linewidth=0.8, alpha=0.9)
    ax.fill_between(freqs, magnitudes, alpha=0.15, color='#00d4ff')

    # Linha do pico
    ax.axvline(x=peak_freq, color='#ff4444', linewidth=1.5,
               linestyle='--', label=f'Pico: {peak_freq:.2f} Hz')

    # Anotação do pico
    peak_mag = magnitudes[int(peak_freq / bin_resolution)]
    ax.annotate(
        f'{peak_freq:.2f} Hz',
        xy=(peak_freq, peak_mag),
        xytext=(peak_freq + 10, peak_mag * 0.85),
        color='#ff4444',
        fontsize=11,
        fontweight='bold',
        arrowprops=dict(arrowstyle='->', color='#ff4444', lw=1.5)
    )

    # Estilo
    ax.set_xlabel('Frequência (Hz)', color='#aaaaaa', fontsize=12)
    ax.set_ylabel('Magnitude',       color='#aaaaaa', fontsize=12)
    ax.set_title(
        f'Espectro FFT — GY45/MMA8452Q  |  RMS: {rms:.4f} g  |  Resolução: {bin_resolution:.3f} Hz/bin',
        color='white', fontsize=13, pad=12
    )
    ax.tick_params(colors='#aaaaaa')
    ax.spines['bottom'].set_color('#333333')
    ax.spines['left'].set_color('#333333')
    ax.spines['top'].set_visible(False)
    ax.spines['right'].set_visible(False)
    ax.grid(True, color='#222222', linewidth=0.5)
    ax.legend(facecolor='#1a1a2e', edgecolor='#333333',
              labelcolor='white', fontsize=11)
    ax.set_xlim(0, freqs[-1])
    ax.set_ylim(bottom=0)

    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    filename  = os.path.join(OUTPUT_DIR, f"fft_{timestamp}_{index:04d}.png")
    plt.savefig(filename, dpi=150, bbox_inches='tight',
                facecolor=fig.get_facecolor())
    plt.close()
    print(f"[{index:04d}] Salvo: {filename}  |  Pico: {peak_freq:.2f} Hz  |  RMS: {rms:.4f} g")

def main():
    print(f"Conectando em {PORT} @ {BAUDRATE}...")
    ser = serial.Serial(PORT, BAUDRATE, timeout=5)
    print("Aguardando dados da FFT...\n")

    image_count = 0
    buffer      = []
    capturing   = False
    peak_freq   = 0.0
    rms         = 0.0
    bin_res     = 0.0

    while True:
        try:
            line = ser.readline().decode('utf-8', errors='ignore').strip()
            if not line:
                continue

            if line.startswith("FFT_START:"):
                # FFT_START:159.23:0.03210:0.4736
                parts     = line.split(":")
                peak_freq = float(parts[1])
                rms       = float(parts[2])
                bin_res   = float(parts[3])
                buffer    = []
                capturing = True

            elif line == "FFT_END" and capturing:
                if len(buffer) == N_BINS:
                    magnitudes = np.array(buffer, dtype=np.float64)
                    image_count += 1
                    save_fft_png(magnitudes, peak_freq, rms, bin_res, image_count)
                else:
                    print(f"[AVISO] Buffer incompleto: {len(buffer)} bins")
                capturing = False
                buffer    = []

            elif line == "STATUS:parado":
                print("Sensor parado — sem vibração detectada.")

            elif capturing:
                buffer.append(float(line))

        except KeyboardInterrupt:
            print("\nEncerrado pelo usuário.")
            ser.close()
            break
        except Exception as e:
            print(f"[ERRO] {e}")

if __name__ == "__main__":
    main()