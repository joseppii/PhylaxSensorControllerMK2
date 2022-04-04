#include <Arduino.h>
#include <SBUS2.h>
#include <Wire.h>

#include "SmcG2.h"

#define SDA2 15
#define SCL2 33

#define RXD2 12
#define TXD2 27

TwoWire I2C1 = TwoWire(0); //Setup the first I2C bus for the TFT & IMU
TwoWire I2C2 = TwoWire(1); //Setup the second I2C bus for the motors 

uint8_t FrameErrorRate = 0;
int16_t channel[16] = {0};

void i2cBusScanner(TwoWire& I2C)
{
  byte error, address;
  int nDevices;
  Serial.println("Scanning...");

  nDevices = 0;

  for(address = 1; address < 127; address++ ) {
    I2C.beginTransmission(address);
    error = I2C.endTransmission();
  
    if (error == 0) {
      Serial.print("I2C device found at address 0x");
      
      if (address<16) {
        Serial.print("0");
      }
      Serial.println(address,HEX);
      nDevices++;
    }
    else if (error==4) {
      Serial.print("Unknow error at address 0x");
      if (address<16) {
        Serial.print("0");
      }
      Serial.println(address,HEX);
    }    
  }  
  delay(1000);
}

void setup() {
#if defined (ESP32)
  Serial.begin(115200);
  Serial.println("Setup Serial...");
#endif // ESP32

  SBUS2_Setup(RXD2,TXD2);     // For ESP32 set RX and TX Pin Number
  
  I2C1.begin();
  I2C1.setClock(400000);
  I2C2.begin(SDA2,SCL2,(uint32_t)400000);

  SmcG2I2C smc1(13, I2C2); //Left Motor Controller 0x0D
  SmcG2I2C smc2(14, I2C2); //Right Motor Controller 0x0E

  smc1.exitSafeStart();
  smc2.exitSafeStart();
}

void loop() {
  //i2cBusScanner(I2C2);
if(SBUS_Ready()){                               // SBUS Frames available -> Ready for getting Servo Data
      for(uint8_t i = 0; i<3; i++)
      {
        channel[i] = SBUS2_get_servo_data(i);   // Channel = Servo Value of Channel
      }
      
      FrameErrorRate = SBUS_get_FER();
    }
}
