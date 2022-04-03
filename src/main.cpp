#include <Arduino.h>
#include <Wire.h>
#include "SmcG2.h"

#define SDA2 15
#define SCL2 33

TwoWire I2C1 = TwoWire(0); //Setup the first I2C bus for the TFT & IMU
TwoWire I2C2 = TwoWire(1); //Setup the second I2C bus for the motors 

void setup() {
#if defined (ESP32)
  Serial.begin(115200);
  Serial.println("Setup Serial...");
#endif // ESP32

  I2C1.begin();
  I2C1.setClock(400000);
  I2C2.begin(SDA2,SCL2,(uint32_t)400000);

  SmcG2I2C smc1(13, I2C2); //Left Motor Controller
  SmcG2I2C smc2(14, I2C2); //Right Motor Controller

  smc1.exitSafeStart();
  smc2.exitSafeStart();
}

void loop() {

}
