/* ================================================================
 *  ESP32-S3 + BMI160 (I2C) - Coleta de 20.48s + FFT unica (PSRAM)
 *  Iniciacao Cientifica - Ciclo 3/4
 *
 *  ESTRATEGIA: acumula 32768 amostras/eixo (20.48s a 1600Hz) em
 *  PSRAM. So DEPOIS de completar a coleta, roda uma unica FFT de
 *  32768 pontos por eixo. Durante a coleta, plota ao vivo no Serial
 *  Plotter; ao completar, pausa e imprime o resultado da FFT.
 *
 *  Por que 32768 e nao 30s (48000 amostras): a biblioteca arduinoFFT
 *  usa uint16_t para o numero de amost ras internamente (limite maximo
 *  65535), entao a maior potencia de 2 segura e 32768. Por isso a
 *  janela de analise foi ajustada para exatos 20.48s.
 *
 *  REQUISITO DE HARDWARE: sua placa precisa ter PSRAM habilitada
 *  (Arduino IDE: Tools > PSRAM > OPI/QSPI PSRAM). O firmware verifica
 *  isso no boot e trava com mensagem clara se nao encontrar - nao
 *  tenta se virar sem ela (3 eixos x 32768 x 4 bytes x 2 buffers
 *  [real+imag] = ~512KB, praticamente toda a SRAM interna do chip).
 *
 *  Resolucao em frequencia resultante: 1600 / 32768 = 0.0488 Hz/bin.
 * ================================================================ */

#include <Wire.h>
#include <arduinoFFT.h>

// ---------------------------------------------------------------
// PINAGEM E ENDERECO I2C
// ---------------------------------------------------------------
static const int PIN_SDA  = 8;
static const int PIN_SCL  = 9;
static const int PIN_INT1 = 4;
#define BMI160_I2C_ADDR 0x69

// ---------------------------------------------------------------
// REGISTRADORES BMI160
// ---------------------------------------------------------------
#define REG_CHIP_ID        0x00
#define REG_PMU_STATUS     0x03
#define REG_FIFO_LENGTH_0  0x22
#define REG_FIFO_LENGTH_1  0x23
#define REG_FIFO_DATA      0x24
#define REG_ACC_CONF       0x40
#define REG_ACC_RANGE      0x41
#define REG_FIFO_CONFIG_0  0x46
#define REG_FIFO_CONFIG_1  0x47
#define REG_INT_EN_1       0x51
#define REG_INT_OUT_CTRL   0x53
#define REG_INT_MAP_1      0x56
#define REG_CMD            0x7E

#define CMD_SOFTRESET      0xB6
#define CMD_ACC_SET_NORMAL 0x11
#define CMD_FIFO_FLUSH     0xB0

#define ACC_CONF_VALUE    0x2C     // odr=1600Hz | bwp=normal
#define ACC_RANGE_VALUE   0x05     // +-4g (era +-8g; amplitudes medidas ficam bem abaixo de 4g, dobra a resolucao)
#define ACC_SENSITIVITY   8192.0f  // LSB/g para +-4g (32768/4 = 8192; era 4096 para +-8g)

#define BATCH_SAMPLES     32
#define BATCH_BYTES       (BATCH_SAMPLES * 6)
#define FIFO_WM_UNITS     (BATCH_BYTES / 4)
#define FIFO_READ_BUFFER_BYTES 240
uint8_t fifoBuf[FIFO_READ_BUFFER_BYTES];

// ---------------------------------------------------------------
// COLETA + FFT UNICA
// ---------------------------------------------------------------
#define FFT_SIZE 32768UL   // = 20.48s a 1600Hz, potencia de 2, dentro do limite uint16_t da lib
#define HAMMING_COHERENT_GAIN 0.54f
#define FFT_MAG_NORM (2.0f / (FFT_SIZE * HAMMING_COHERENT_GAIN))

float *vRealX, *vRealY, *vRealZ, *vImag; // alocados em PSRAM no setup()
ArduinoFFT<float> *fftX, *fftY, *fftZ;

enum State { ACQUIRING, ANALYZING, DONE };
State state = ACQUIRING;
uint32_t sampleCount = 0;

volatile bool fifoReadyFlag = false;
void IRAM_ATTR onFifoWatermark() { fifoReadyFlag = true; }

// ---------------------------------------------------------------
// I2C - LEITURA/ESCRITA
// ---------------------------------------------------------------
void bmi160WriteReg(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(BMI160_I2C_ADDR);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
}

uint8_t bmi160ReadReg(uint8_t reg) {
  Wire.beginTransmission(BMI160_I2C_ADDR);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom((int)BMI160_I2C_ADDR, 1);
  return Wire.available() ? Wire.read() : 0xFF;
}

bool bmi160ReadRegs(uint8_t startReg, uint8_t *buffer, uint16_t len) {
  Wire.beginTransmission(BMI160_I2C_ADDR);
  Wire.write(startReg);
  Wire.endTransmission(false);
  uint16_t got = Wire.requestFrom((int)BMI160_I2C_ADDR, (int)len);
  if (got != len) return false;
  for (uint16_t i = 0; i < len; i++) buffer[i] = Wire.read();
  return true;
}

bool bmi160Init() {
  Wire.setBufferSize(256);   // CRITICO - sem isso, leituras grandes falham silenciosamente
  Wire.begin(PIN_SDA, PIN_SCL);
  Wire.setClock(400000);

  bmi160WriteReg(REG_CMD, CMD_SOFTRESET);
  delay(15);

  uint8_t chipId = bmi160ReadReg(REG_CHIP_ID);
  Serial.print("[DIAG] CHIP_ID=0x"); Serial.println(chipId, HEX);
  if (chipId != 0xD1) return false;

  bmi160WriteReg(REG_CMD, CMD_ACC_SET_NORMAL);
  delay(10);
  bmi160WriteReg(REG_ACC_CONF, ACC_CONF_VALUE);
  delay(1);
  bmi160WriteReg(REG_ACC_RANGE, ACC_RANGE_VALUE);
  delay(1);

  // VERIFICACAO DE INTEGRIDADE: rele o registrador para confirmar que
  // a escrita realmente "pegou". Se o I2C tiver uma falha silenciosa
  // aqui, o sensor ficaria preso na faixa padrao de fabrica (+-2g),
  // e TODA leitura de aceleracao sairia sistematicamente errada por
  // um fator fixo - um erro bem mais serio que qualquer coisa mecanica.
  uint8_t rangeReadback = bmi160ReadReg(REG_ACC_RANGE);
  Serial.print("[DIAG] ACC_RANGE escrito=0x"); Serial.print(ACC_RANGE_VALUE, HEX);
  Serial.print(" lido de volta=0x"); Serial.println(rangeReadback, HEX);
  if (rangeReadback != ACC_RANGE_VALUE) {
    Serial.println("[FATAL] ACC_RANGE nao foi aplicado corretamente - todas as leituras de g estariam erradas!");
    return false;
  }

  bmi160WriteReg(REG_CMD, CMD_FIFO_FLUSH);
  delay(1);
  bmi160WriteReg(REG_FIFO_CONFIG_1, 0x40);         // habilita accel no FIFO, modo headerless
  bmi160WriteReg(REG_FIFO_CONFIG_0, FIFO_WM_UNITS); // watermark
  bmi160WriteReg(REG_INT_EN_1, 0x40);               // fwm_en
  bmi160WriteReg(REG_INT_OUT_CTRL, 0x0B);           // push-pull, ativo alto, BORDA (bit0=1)
  bmi160WriteReg(REG_INT_MAP_1, 0x40);              // int1_fwm

  return true;
}

// ---------------------------------------------------------------
// ANALISE FINAL
// ---------------------------------------------------------------
void removeDC(float *buf, uint32_t n) {
  double mean = 0;
  for (uint32_t i = 0; i < n; i++) mean += buf[i];
  mean /= n;
  for (uint32_t i = 0; i < n; i++) buf[i] -= (float)mean;
}

// RMS no dominio do tempo - deve ser calculado sobre o sinal SEM
// gravidade (apos removeDC) e ANTES de qualquer janela ser aplicada,
// senao o resultado sai artificialmente reduzido pela atenuacao que
// a propria janela de Hamming introduz nas bordas do sinal.
double computeRMS(float *buf, uint32_t n) {
  double sumSq = 0;
  for (uint32_t i = 0; i < n; i++) sumSq += (double)buf[i] * (double)buf[i];
  return sqrt(sumSq / n);
}

void printTop5Peaks(float *vReal, char axisLabel) {
  Serial.print("  Eixo "); Serial.print(axisLabel); Serial.println(":");
  // Reaproveita vImag como area de trabalho para copiar/mutilar os
  // picos - nesse ponto a FFT ja terminou, vImag nao e mais necessario.
  float *magCopy = vImag;
  uint32_t half = FFT_SIZE / 2;
  for (uint32_t i = 0; i < half; i++) magCopy[i] = vReal[i] * FFT_MAG_NORM;
  magCopy[0] = 0; // ignora DC

  for (int p = 0; p < 5; p++) {
    uint32_t peakBin = 0;
    float peakVal = 0;
    for (uint32_t i = 1; i < half; i++) {
      if (magCopy[i] > peakVal) { peakVal = magCopy[i]; peakBin = i; }
    }
    float freqHz = peakBin * (1600.0f / FFT_SIZE);
    Serial.print("    "); Serial.print(freqHz, 3);
    Serial.print(" Hz  ("); Serial.print(peakVal, 5); Serial.println(" g)");

    // Exclusao alargada: com resolucao fina (0.0488 Hz/bin), o
    // espalhamento natural de um pico real (mesmo com janela de
    // Hamming) cobre bem mais que 5 bins - sem isso, o mesmo pico
    // fisico aparecia duas vezes na lista (foi o caso dos "160.791"
    // e "160.449" que sao a mesma vibracao, nao dois sinais).
    // +-40 bins = +-1.95 Hz de exclusao, folga confortavel.
    int lo = max((int)1, (int)peakBin - 40);
    int hi = min((int)half - 1, (int)peakBin + 40);
    for (int i = lo; i <= hi; i++) magCopy[i] = 0;
  }
}

void runFinalFFT() {
  Serial.println("=== Iniciando analise FFT (32768 amostras, 20.48s) ===");
  Serial.println("Pode levar alguns segundos - PSRAM e mais lenta que SRAM interna.");
  uint32_t t0 = millis();

  removeDC(vRealX, FFT_SIZE);
  double rmsX = computeRMS(vRealX, FFT_SIZE);
  Serial.print("  RMS eixo X (sem gravidade): "); Serial.print(rmsX, 5); Serial.println(" g");
  for (uint32_t i = 0; i < FFT_SIZE; i++) vImag[i] = 0;
  fftX->windowing(FFTWindow::Hamming, FFTDirection::Forward);
  fftX->compute(FFTDirection::Forward);
  fftX->complexToMagnitude();
  printTop5Peaks(vRealX, 'X');
  yield();

  removeDC(vRealY, FFT_SIZE);
  double rmsY = computeRMS(vRealY, FFT_SIZE);
  Serial.print("  RMS eixo Y (sem gravidade): "); Serial.print(rmsY, 5); Serial.println(" g");
  for (uint32_t i = 0; i < FFT_SIZE; i++) vImag[i] = 0;
  fftY->windowing(FFTWindow::Hamming, FFTDirection::Forward);
  fftY->compute(FFTDirection::Forward);
  fftY->complexToMagnitude();
  printTop5Peaks(vRealY, 'Y');
  yield();

  removeDC(vRealZ, FFT_SIZE);
  double rmsZ = computeRMS(vRealZ, FFT_SIZE);
  Serial.print("  RMS eixo Z (sem gravidade): "); Serial.print(rmsZ, 5); Serial.println(" g");
  for (uint32_t i = 0; i < FFT_SIZE; i++) vImag[i] = 0;
  fftZ->windowing(FFTWindow::Hamming, FFTDirection::Forward);
  fftZ->compute(FFTDirection::Forward);
  fftZ->complexToMagnitude();
  printTop5Peaks(vRealZ, 'Z');

  uint32_t elapsed = millis() - t0;
  Serial.print("=== FFT concluida em "); Serial.print(elapsed); Serial.println(" ms ===");
}

// ---------------------------------------------------------------
// SETUP
// ---------------------------------------------------------------
void setup() {
  Serial.begin(921600);
  while (!Serial) { delay(10); }

  Serial.println("Verificando PSRAM...");
  if (!psramFound()) {
    Serial.println("[FATAL] PSRAM nao encontrada. Verifique:");
    Serial.println("  1) Sua placa tem PSRAM fisicamente (variante N16R8, nao N8)?");
    Serial.println("  2) Arduino IDE: Tools > PSRAM > habilitada (OPI ou QSPI)?");
    while (true) delay(1000);
  }
  Serial.print("[OK] PSRAM detectada: ");
  Serial.print(ESP.getPsramSize() / (1024.0 * 1024.0), 2);
  Serial.println(" MB");

  size_t bufBytes = (size_t)FFT_SIZE * sizeof(float);
  vRealX = (float*) ps_malloc(bufBytes);
  vRealY = (float*) ps_malloc(bufBytes);
  vRealZ = (float*) ps_malloc(bufBytes);
  vImag  = (float*) ps_malloc(bufBytes);

  if (!vRealX || !vRealY || !vRealZ || !vImag) {
    Serial.println("[FATAL] Falha ao alocar buffers em PSRAM.");
    while (true) delay(1000);
  }
  Serial.print("[OK] Buffers alocados: ");
  Serial.print((bufBytes * 4) / (1024.0 * 1024.0), 2);
  Serial.println(" MB no total");

  fftX = new ArduinoFFT<float>(vRealX, vImag, FFT_SIZE, 1600.0f);
  fftY = new ArduinoFFT<float>(vRealY, vImag, FFT_SIZE, 1600.0f);
  fftZ = new ArduinoFFT<float>(vRealZ, vImag, FFT_SIZE, 1600.0f);

  Serial.println("Inicializando BMI160 (I2C)...");
  if (!bmi160Init()) {
    Serial.println("[FATAL] Falha na inicializacao do BMI160.");
    while (true) delay(1000);
  }
  Serial.println("BMI160 pronto. Iniciando coleta de 20.48 segundos (32768 amostras)...");
  Serial.println("ax_g,ay_g,az_g");

  pinMode(PIN_INT1, INPUT);
  attachInterrupt(digitalPinToInterrupt(PIN_INT1), onFifoWatermark, RISING);
}

// ---------------------------------------------------------------
// LOOP
// ---------------------------------------------------------------
uint32_t lastHeartbeatMs = 0;

void loop() {
  // Heartbeat independente do estado - roda sempre, mesmo se a
  // coleta estiver de fato travada, para diferenciar "travado de
  // verdade" de "so sem feedback visual ate completar".
  
  if (state == ACQUIRING && fifoReadyFlag) {
    noInterrupts();
    fifoReadyFlag = false;
    interrupts();

    while (true) {
      uint8_t lenLow  = bmi160ReadReg(REG_FIFO_LENGTH_0);
      uint8_t lenHigh = bmi160ReadReg(REG_FIFO_LENGTH_1);
      uint16_t available = ((uint16_t)(lenHigh & 0x07) << 8) | lenLow;

      uint16_t toRead = min(available, (uint16_t)FIFO_READ_BUFFER_BYTES);
      toRead -= (toRead % 6);
      if (toRead == 0) break;

      bool ok = bmi160ReadRegs(REG_FIFO_DATA, fifoBuf, toRead);
      if (!ok) {
        Serial.println("[WARN] leitura de FIFO falhou, lote descartado");
        break;
      }

      uint16_t samplesInBatch = toRead / 6;
      for (uint16_t i = 0; i < samplesInBatch && sampleCount < FFT_SIZE; i++) {
        uint8_t *s = &fifoBuf[i * 6];
        int16_t rawX = (int16_t)((s[1] << 8) | s[0]);
        int16_t rawY = (int16_t)((s[3] << 8) | s[2]);
        int16_t rawZ = (int16_t)((s[5] << 8) | s[4]);

        float ax_g = rawX / ACC_SENSITIVITY;
        float ay_g = rawY / ACC_SENSITIVITY;
        float az_g = rawZ / ACC_SENSITIVITY;

        Serial.print(ax_g, 5); Serial.print(',');
        Serial.print(ay_g, 5); Serial.print(',');
        Serial.println(az_g, 5);

        vRealX[sampleCount] = ax_g;
        vRealY[sampleCount] = ay_g;
        vRealZ[sampleCount] = az_g;
        sampleCount++;
      }

      if (sampleCount >= FFT_SIZE) {
        state = ANALYZING;
        break;
      }
      if (available <= FIFO_READ_BUFFER_BYTES) break;
      yield();
    }
  }

  if (state == ANALYZING) {
    Serial.println("Coleta concluida (32768 amostras).");
    runFinalFFT();
    state = DONE;
  }

  if (state == DONE) {
    delay(1000); // fica parado - reinicie a placa para rodar de novo
  }
}
