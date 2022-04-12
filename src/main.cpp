#include <Arduino.h>

#include <SBUS2.h>
#include <Wire.h>
#include <WiFi.h>
#include "SmcG2.h"
#include "encoder.h"
#include "RosImu.h"

#define SDA2 15
#define SCL2 33

#define RXD2 12
#define TXD2 27

#define AD0_VAL 1      // The value of the last bit of the I2C address.                \
                       // On the SparkFun 9DoF IMU breakout the default is 1, and when \
                       // the ADR jumper is closed the value becomes 0

#define LED_PIN 13

#if __has_include("credentials.h")

// For local development (rename credenials-template.h and use your current WiFi credentials
#include "credentials.h"

#else

// WiFi credentials
  char ssid[] = "*****";
  char password[] = "*****";
#endif

TwoWire I2C1 = TwoWire(0); //Setup the first I2C bus for the TFT & IMU
//TwoWire I2C2 = TwoWire(1); //Setup the second I2C bus for the motors 

SmcG2I2C smc1(13, I2C1); //Left Motor Controller 0x0D
SmcG2I2C smc2(14, I2C1); //Right Motor Controller 0x0E

PololuEncoder encoderLeft(34, 48, 0.03);  //Left
PololuEncoder encoderRight(34, 48, 0.03); //Right

ICM_20948_I2C myICM; //Create an ICM_20948_I2C object
RosImu rosImu(myICM);

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

uint32_t encoder1Count;
uint32_t encoder2Count;

void setup() {
#if defined (ESP32)
  Serial.begin(115200);
  Serial.println("Setup Serial...");
#endif // ESP32

  SBUS2_Setup(RXD2,TXD2);     // For ESP32 set RX and TX Pin Number
  
  I2C1.begin();
  I2C1.setClock(400000);
  //I2C2.begin(SDA2,SCL2,(uint32_t)400000);

  encoderLeft.init(19,18);
  encoderRight.init(17,16);

  smc1.exitSafeStart();
  smc2.exitSafeStart();

  rosImu.begin(I2C1, AD0_VAL);
  rosImu.init();
} 

void loop() {
  if(SBUS_Ready())    // SBUS Frames available -> Ready for getting Servo Data
  {                               
    for(uint8_t i = 0; i<3; i++)
    {
      channel[i] = SBUS2_get_servo_data(i); // Channel = Servo Value of Channel
     //Serial.println((channel[i]-970)*4);
    }
    
    FrameErrorRate = SBUS_get_FER();
    smc1.setTargetSpeed((channel[0]-970)*4);
    smc2.setTargetSpeed((channel[2]-970)*4);
  }

  encoderLeft.update(encoder1Count);
  encoderRight.update(encoder2Count);
  
  rosImu.read();
  delay(30);
}
