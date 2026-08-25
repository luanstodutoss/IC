/* ================================================================
 *  ESP32-S3 + MMA8452Q (I2C) - Coleta de 20.48s + FFT unica (PSRAM)
 *  Iniciacao Cientifica - port do firmware do BMI160 para o MMA8452Q
 *
 *  DIFERENCAS DE HARDWARE EM RELACAO AO BMI160 (documentar no relatorio):
 *    - ODR maxima: 800 Hz (contra 1600 Hz do BMI160) -> Nyquist = 400 Hz.
 *      Qualquer conteudo de vibracao acima de 400 Hz NAO e capturavel
 *      com este sensor - e uma limitacao fisica, nao de firmware.
 *    - Sem FIFO interno: usa interrupcao de Data Ready por amostra
 *      (mesmo principio do primeiro firmware que fizemos pro BMI160,
 *      antes de migrarmos pra FIFO) - nao ha "lote" para drenar aqui.
 *    - Resolucao: 12 bits (contra 16 bits do BMI160).
 *
 *  Como a ODR e menor, 20.48s a 800Hz da exatamente 16384 amostras
 *  (2^14) - ja e potencia de 2 exata, sem precisar de zero-padding.
 *  Resolucao em frequencia: 800/16384 = 0.0488 Hz/bin (mesma resolucao
 *  fina que tinhamos com o BMI160, por coincidencia da escolha de
 *  20.48s como janela).
 *
 *  Pinagem (I2C, mesmos pinos ja usados com o BMI160):
 *    SCL -> GPIO9
 *    SDA -> GPIO8
 *    INT1 (do sensor) -> GPIO4
 *    VCC -> 3V3, GND -> GND
 *    Endereco I2C: 0x1C (confirmado via firmware de identificacao)
 *
 *  REQUISITO DE HARDWARE: PSRAM habilitada (Tools > PSRAM > OPI/QSPI).
 * ================================================================ */

#include <Wire.h>
#include <arduinoFFT.h>

// ---------------------------------------------------------------
// PINAGEM E ENDERECO I2C
// ---------------------------------------------------------------
static const int PIN_SDA  = 8;
static const int PIN_SCL  = 9;
static const int PIN_INT1 = 4;
#define MMA8452Q_I2C_ADDR 0x1C

// ---------------------------------------------------------------
// REGISTRADORES MMA8452Q (Freescale/NXP)
// ---------------------------------------------------------------
#define REG_STATUS       0x00  // bit3 = ZYXDR (novo dado pronto)
#define REG_OUT_X_MSB    0x01  // burst de 6 bytes: X_MSB,X_LSB,Y_MSB,Y_LSB,Z_MSB,Z_LSB
#define REG_WHO_AM_I     0x0D  // deve retornar 0x2A
#define REG_XYZ_DATA_CFG 0x0E  // bits[1:0] = faixa de escala (FS)
#define REG_CTRL_REG1    0x2A  // bits[5:3]=DR (ODR), bit0=ACTIVE
#define REG_CTRL_REG2    0x2B  // bit6 = RST (reset por software)
#define REG_CTRL_REG3    0x2C  // bit1=IPOL, bit0=PP_OD
#define REG_CTRL_REG4    0x2D  // bit0 = INT_EN_DRDY
#define REG_CTRL_REG5    0x2E  // bit0 = INT_CFG_DRDY (1=INT1, 0=INT2)

#define XYZ_DATA_CFG_4G  0x01     // FS=01 -> +-4g
#define ACC_SENSITIVITY  512.0f   // LSB/g para +-4g, 12 bits (2048 counts / 4g)

#define FFT_SIZE 16384UL   // 20.48s a 800Hz - ja potencia de 2 exata
#define HAMMING_COHERENT_GAIN 0.54f
#define FFT_MAG_NORM (2.0f / (FFT_SIZE * HAMMING_COHERENT_GAIN))

float *vRealX, *vRealY, *vRealZ, *vImag;
ArduinoFFT<float> *fftX, *fftY, *fftZ;

enum State { ACQUIRING, ANALYZING, DONE };
State state = ACQUIRING;
uint32_t sampleCount = 0;

volatile bool dataReadyFlag = false;
void IRAM_ATTR onDataReady() { dataReadyFlag = true; }

// ---------------------------------------------------------------
// I2C - LEITURA/ESCRITA
// ---------------------------------------------------------------
void writeReg(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(MMA8452Q_I2C_ADDR);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
}

uint8_t readReg(uint8_t reg) {
  Wire.beginTransmission(MMA8452Q_I2C_ADDR);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom((int)MMA8452Q_I2C_ADDR, 1);
  return Wire.available() ? Wire.read() : 0xFF;
}

bool readRegs(uint8_t startReg, uint8_t *buffer, uint8_t len) {
  Wire.beginTransmission(MMA8452Q_I2C_ADDR);
  Wire.write(startReg);
  Wire.endTransmission(false);
  uint8_t got = Wire.requestFrom((int)MMA8452Q_I2C_ADDR, (int)len);
  if (got != len) return false;
  for (uint8_t i = 0; i < len; i++) buffer[i] = Wire.read();
  return true;
}

bool mma8452Init() {
  Wire.setBufferSize(64);
  Wire.begin(PIN_SDA, PIN_SCL);
  Wire.setClock(400000);

  // Reset por software (equivalente ao soft reset que fazíamos no BMI160)
  writeReg(REG_CTRL_REG2, 0x40); // RST=1
  delay(10);

  uint8_t whoAmI = readReg(REG_WHO_AM_I);
  Serial.print("[DIAG] WHO_AM_I=0x"); Serial.println(whoAmI, HEX);
  if (whoAmI != 0x2A) return false;

  // A partir daqui, o sensor precisa estar em STANDBY para configurar
  // (ACTIVE=0, que ja e o estado apos o reset).

  writeReg(REG_XYZ_DATA_CFG, XYZ_DATA_CFG_4G); // +-4g

  writeReg(REG_CTRL_REG3, 0x02); // IPOL=1 (ativo alto), PP_OD=0 (push-pull)
  writeReg(REG_CTRL_REG4, 0x01); // INT_EN_DRDY=1
  writeReg(REG_CTRL_REG5, 0x01); // roteia DRDY para o pino INT1

  // CTRL_REG1: DR2:DR1:DR0 = 000 (800Hz, ODR maxima) | ACTIVE=1
  // Isso tambem transiciona standby->active, ligando o sensor de fato.
  writeReg(REG_CTRL_REG1, 0x01);
  delay(5); // tempo de "turn-on" apos ativar

  // Verificacao de integridade: confirma que a faixa realmente foi
  // aplicada (mesmo cuidado que tivemos com o BMI160).
  uint8_t rangeReadback = readReg(REG_XYZ_DATA_CFG) & 0x03;
  Serial.print("[DIAG] XYZ_DATA_CFG lido de volta=0x"); Serial.println(rangeReadback, HEX);
  if (rangeReadback != XYZ_DATA_CFG_4G) {
    Serial.println("[FATAL] Faixa de escala nao foi aplicada corretamente.");
    return false;
  }

  return true;
}

// ---------------------------------------------------------------
// ANALISE (identica em estrutura ao firmware do BMI160)
// ---------------------------------------------------------------
void removeDC(float *buf, uint32_t n) {
  double mean = 0;
  for (uint32_t i = 0; i < n; i++) mean += buf[i];
  mean /= n;
  for (uint32_t i = 0; i < n; i++) buf[i] -= (float)mean;
}

double computeRMS(float *buf, uint32_t n) {
  double sumSq = 0;
  for (uint32_t i = 0; i < n; i++) sumSq += (double)buf[i] * (double)buf[i];
  return sqrt(sumSq / n);
}

void printTop5Peaks(float *vReal, char axisLabel) {
  Serial.print("  Eixo "); Serial.print(axisLabel); Serial.println(":");
  float *magCopy = vImag; // reaproveitado, FFT ja terminou nesse ponto
  uint32_t half = FFT_SIZE / 2;
  for (uint32_t i = 0; i < half; i++) magCopy[i] = vReal[i] * FFT_MAG_NORM;
  magCopy[0] = 0;

  for (int p = 0; p < 5; p++) {
    uint32_t peakBin = 0;
    float peakVal = 0;
    for (uint32_t i = 1; i < half; i++) {
      if (magCopy[i] > peakVal) { peakVal = magCopy[i]; peakBin = i; }
    }
    float freqHz = peakBin * (800.0f / FFT_SIZE); // 800 = ODR deste sensor (nao 1600!)
    Serial.print("    "); Serial.print(freqHz, 3);
    Serial.print(" Hz  ("); Serial.print(peakVal, 5); Serial.println(" g)");

    int lo = max((int)1, (int)peakBin - 40);
    int hi = min((int)half - 1, (int)peakBin + 40);
    for (int i = lo; i <= hi; i++) magCopy[i] = 0;
  }
}

void runFinalFFT() {
  Serial.println("=== Iniciando analise FFT (16384 amostras, 20.48s a 800Hz) ===");
  uint32_t t0 = millis();

  removeDC(vRealX, FFT_SIZE);
  Serial.print("  RMS eixo X (sem gravidade): "); Serial.print(computeRMS(vRealX, FFT_SIZE), 5); Serial.println(" g");
  for (uint32_t i = 0; i < FFT_SIZE; i++) vImag[i] = 0;
  fftX->windowing(FFTWindow::Hamming, FFTDirection::Forward);
  fftX->compute(FFTDirection::Forward);
  fftX->complexToMagnitude();
  printTop5Peaks(vRealX, 'X');
  yield();

  removeDC(vRealY, FFT_SIZE);
  Serial.print("  RMS eixo Y (sem gravidade): "); Serial.print(computeRMS(vRealY, FFT_SIZE), 5); Serial.println(" g");
  for (uint32_t i = 0; i < FFT_SIZE; i++) vImag[i] = 0;
  fftY->windowing(FFTWindow::Hamming, FFTDirection::Forward);
  fftY->compute(FFTDirection::Forward);
  fftY->complexToMagnitude();
  printTop5Peaks(vRealY, 'Y');
  yield();

  removeDC(vRealZ, FFT_SIZE);
  Serial.print("  RMS eixo Z (sem gravidade): "); Serial.print(computeRMS(vRealZ, FFT_SIZE), 5); Serial.println(" g");
  for (uint32_t i = 0; i < FFT_SIZE; i++) vImag[i] = 0;
  fftZ->windowing(FFTWindow::Hamming, FFTDirection::Forward);
  fftZ->compute(FFTDirection::Forward);
  fftZ->complexToMagnitude();
  printTop5Peaks(vRealZ, 'Z');

  Serial.print("=== FFT concluida em "); Serial.print(millis() - t0); Serial.println(" ms ===");
}

// ---------------------------------------------------------------
// SETUP
// ---------------------------------------------------------------
void setup() {
  Serial.begin(921600);
  while (!Serial) { delay(10); }

  Serial.println("Verificando PSRAM...");
  if (!psramFound()) {
    Serial.println("[FATAL] PSRAM nao encontrada. Tools > PSRAM > habilitar.");
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

  fftX = new ArduinoFFT<float>(vRealX, vImag, FFT_SIZE, 800.0f);
  fftY = new ArduinoFFT<float>(vRealY, vImag, FFT_SIZE, 800.0f);
  fftZ = new ArduinoFFT<float>(vRealZ, vImag, FFT_SIZE, 800.0f);

  Serial.println("Inicializando MMA8452Q (I2C)...");

  // CRITICO: registrar o attachInterrupt ANTES de ativar o sensor.
  // O MMA8452Q mantem o pino de interrupcao em NIVEL alto ate o dado
  // ser lido (nao e um pulso curto como no BMI160) - se o sensor ja
  // estiver ativo (gerando dados) antes do attachInterrupt existir,
  // a primeira borda de subida acontece "no vazio" e o pino fica
  // travado em alto, sem gerar mais nenhuma borda detectavel depois.
  pinMode(PIN_INT1, INPUT);
  attachInterrupt(digitalPinToInterrupt(PIN_INT1), onDataReady, RISING);

  if (!mma8452Init()) {
    Serial.println("[FATAL] Falha na inicializacao do MMA8452Q.");
    while (true) delay(1000);
  }
  Serial.println("MMA8452Q pronto. Iniciando coleta de 20.48s (16384 amostras a 800Hz)...");
  Serial.println("ax_g,ay_g,az_g");
}

// ---------------------------------------------------------------
// LOOP
// ---------------------------------------------------------------
uint32_t lastHeartbeatMs = 0;

void loop() {

  if (state == ACQUIRING && dataReadyFlag) {
    noInterrupts();
    dataReadyFlag = false;
    interrupts();

    uint8_t raw[6];
    if (readRegs(REG_OUT_X_MSB, raw, 6)) {
      // Cada eixo: 12 bits, justificado a esquerda em 16 bits (MSB,
      // LSB com os 4 bits baixos sempre zero) - o shift aritmetico
      // de 4 bits para a direita faz a extensao de sinal corretamente.
      int16_t rawX = ((int16_t)((raw[0] << 8) | raw[1])) >> 4;
      int16_t rawY = ((int16_t)((raw[2] << 8) | raw[3])) >> 4;
      int16_t rawZ = ((int16_t)((raw[4] << 8) | raw[5])) >> 4;

      float ax_g = rawX / ACC_SENSITIVITY;
      float ay_g = rawY / ACC_SENSITIVITY;
      float az_g = rawZ / ACC_SENSITIVITY;

      Serial.print(ax_g, 5); Serial.print(',');
      Serial.print(ay_g, 5); Serial.print(',');
      Serial.println(az_g, 5);

      if (sampleCount < FFT_SIZE) {
        vRealX[sampleCount] = ax_g;
        vRealY[sampleCount] = ay_g;
        vRealZ[sampleCount] = az_g;
        sampleCount++;
      }
      if (sampleCount >= FFT_SIZE) {
        state = ANALYZING;
      }
    } else {
      Serial.println("[WARN] leitura I2C falhou, amostra descartada");
    }
  }

  if (state == ANALYZING) {
    Serial.println("Coleta concluida (16384 amostras).");
    runFinalFFT();
    state = DONE;
  }

  if (state == DONE) {
    delay(1000);
  }
}
