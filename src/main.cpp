#include <Arduino.h>
#include <Wire.h>

#define SDA2 15
#define SCL2 33

TwoWire I2Cone = TwoWire(0);
TwoWire I2Ctwo = TwoWire(1);

int i2c_master_write_register(uint8_t address, uint8_t reg, uint32_t len, const uint8_t *data)
{
  I2Ctwo.beginTransmission(address);
  I2Ctwo.write(reg);
  I2Ctwo.write(data, len);
  I2Ctwo.endTransmission();
  
  return 0;
}

int i2c_master_read_register(uint8_t address, uint8_t reg, uint32_t len, uint8_t *buff)
{
  I2Ctwo.beginTransmission(address);
  I2Ctwo.write(reg);
  I2Ctwo.endTransmission(false); // Send repeated start

  uint32_t offset = 0;
  uint32_t num_received = I2Ctwo.requestFrom(address, len);

  if (num_received == len)
  {
    for (uint8_t i = 0; i < len; i++){
      buff[i] = I2Ctwo.read();
    }
    return 0;
  }
  else
  {
    return -1;
  }
}

void setup() {
#if defined (ESP32)
  Serial.begin(115200);
  Serial.println("Setup Serial...");
#endif // ESP32

  I2Cone.begin();
  I2Cone.setClock(400000);
  I2Ctwo.begin(SDA2,SCL2,(uint32_t)400000);
}

void loop() {


  delay(1);
}