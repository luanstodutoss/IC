#include <Wire.h>
#include <SparkFun_MMA8452Q.h>
#include <arduinoFFT.h>

MMA8452Q acc;

const uint16_t N_SAMPLES   = 2048;
const float    REAL_ODR    = 970.0f;
const float    NOISE_FLOOR = 0.02f;

double vReal[N_SAMPLES];
double vImag[N_SAMPLES];

ArduinoFFT<double> FFT = ArduinoFFT<double>(vReal, vImag, N_SAMPLES, REAL_ODR);

uint16_t sampleIndex = 0;

void setup() {
  Serial.begin(115200);
  Wire.begin(8, 9);

  if (acc.begin(Wire, 0x1C) == false) {
    Serial.println("ERRO: Sensor não encontrado!");
    while (true);
  }

  acc.setScale(SCALE_4G);
  acc.setDataRate(ODR_800);
}

void removeDC() {
  double sum = 0.0;
  for (uint16_t i = 0; i < N_SAMPLES; i++) sum += vReal[i];
  double mean = sum / N_SAMPLES;
  for (uint16_t i = 0; i < N_SAMPLES; i++) vReal[i] -= mean;
}

void loop() {
  // Coleta amostras
  if (sampleIndex < N_SAMPLES) {
    if (!acc.available()) return;
    vReal[sampleIndex] = acc.getCalculatedZ();
    vImag[sampleIndex] = 0.0;
    sampleIndex++;
    return;
  }

  removeDC();

  // Checa RMS
  double sumSq = 0.0;
  for (uint16_t i = 0; i < N_SAMPLES; i++) sumSq += vReal[i] * vReal[i];
  double rms = sqrt(sumSq / N_SAMPLES);

  if (rms < NOISE_FLOOR) {
    Serial.println("STATUS:parado");
    sampleIndex = 0;
    return;
  }

  // FFT
  FFT.windowing(FFTWindow::Hann, FFTDirection::Forward);
  FFT.compute(FFTDirection::Forward);
  FFT.complexToMagnitude();

  // Pico
  uint16_t peakBin = 2;
  double   peakMag = 0.0;
  for (uint16_t i = 2; i < N_SAMPLES / 2; i++) {
    if (vReal[i] > peakMag) { peakMag = vReal[i]; peakBin = i; }
  }

  // Interpolação parabólica
  float refinedFreq;
  if (peakBin > 2 && peakBin < (N_SAMPLES / 2 - 1)) {
    double y1    = vReal[peakBin - 1];
    double y2    = vReal[peakBin];
    double y3    = vReal[peakBin + 1];
    double delta = 0.5 * (y3 - y1) / (2.0 * y2 - y1 - y3);
    refinedFreq  = (peakBin + delta) * (REAL_ODR / N_SAMPLES);
  } else {
    refinedFreq  = peakBin * (REAL_ODR / N_SAMPLES);
  }

  // --- Envia dados estruturados para o Python ---
  // Formato: FFT_START → bins → FFT_END
  Serial.print("FFT_START:");
  Serial.print(refinedFreq, 2);   // frequência do pico
  Serial.print(":");
  Serial.print(rms, 5);           // RMS
  Serial.print(":");
  Serial.println(REAL_ODR / N_SAMPLES, 4);  // resolução por bin

  // Envia apenas os primeiros 512 bins (0~485 Hz) para não sobrecarregar
  for (uint16_t i = 0; i < 512; i++) {
    Serial.println(vReal[i], 4);
  }

  Serial.println("FFT_END");

  sampleIndex = 0;
}