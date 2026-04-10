#include <Wire.h>
#include <SparkFun_MMA8452Q.h>

MMA8452Q acc;

void setup() {
  Serial.begin(115200);
  Wire.begin(8, 9);

  if (acc.begin(Wire, 0x1C) == false) {
    Serial.println("Sensor não encontrado!");
    while (true);
  }

  acc.setScale(SCALE_4G);
  acc.setDataRate(ODR_800);
  Serial.println("Sensor iniciado!");
}

void loop() {
  if (acc.available()) {
    Serial.print("X:"); Serial.print(acc.getCalculatedX()); Serial.print(" ");
    //Serial.print("Y:"); Serial.print(acc.getCalculatedY()); Serial.print(" ");
    //Serial.print("Z:"); Serial.print(acc.getCalculatedZ()); Serial.print(" ");
    Serial.print("MIN:"); Serial.print(-4.0); Serial.print(" ");
    Serial.print("MAX:"); Serial.println(4.0);
  }

}