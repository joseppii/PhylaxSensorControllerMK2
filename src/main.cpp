#include <Arduino.h>
#include <SBUS2.h>
#include <Wire.h>
#include <WiFi.h>
#include "SmcG2.h"
#include "encoder.h"
#include "ICM_20948.h"

#define SDA2 15
#define SCL2 33

#define RXD2 12
#define TXD2 27

#define AD0_VAL 1      // The value of the last bit of the I2C address.                \
                       // On the SparkFun 9DoF IMU breakout the default is 1, and when \
                       // the ADR jumper is closed the value becomes 0

TwoWire I2C1 = TwoWire(0); //Setup the first I2C bus for the TFT & IMU
TwoWire I2C2 = TwoWire(1); //Setup the second I2C bus for the motors 

SmcG2I2C smc1(13, I2C2); //Left Motor Controller 0x0D
SmcG2I2C smc2(14, I2C2); //Right Motor Controller 0x0E

PololuEncoder encoderLeft(34, 48, 0.03);  //Left
PololuEncoder encoderRight(34, 48, 0.03); //Right

ICM_20948_I2C myICM; //Create an ICM_20948_I2C object

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
  I2C2.begin(SDA2,SCL2,(uint32_t)400000);

  encoderLeft.init(19,18);
  encoderRight.init(17,16);

  smc1.exitSafeStart();
  smc2.exitSafeStart();

  bool initialized = false;
  while (!initialized)
  {
    myICM.begin(I2C1, AD0_VAL); // Initialize the ICM-20948

    Serial.print(F("Initialization of the IMU sensor returned: "));
    Serial.println(myICM.statusString());
    if (myICM.status != ICM_20948_Stat_Ok)
    {
      Serial.println(F("Trying again..."));
      delay(500);
    }
    else
    {
      initialized = true;
    }
  }

  Serial.println(F("Device connected!"));

  bool success = true; // Use success to show if the DMP configuration was successful

  // Initialize the DMP. initializeDMP is a weak function. You can overwrite it if you want to e.g. to change the sample rate
  success &= (myICM.initializeDMP() == ICM_20948_Stat_Ok);

  // Enable the DMP Game Rotation Vector sensor (Quat6)
  success &= (myICM.enableDMPSensor(INV_ICM20948_SENSOR_GAME_ROTATION_VECTOR) == ICM_20948_Stat_Ok);

  // Enable additional sensors / features
  success &= (myICM.enableDMPSensor(INV_ICM20948_SENSOR_RAW_GYROSCOPE) == ICM_20948_Stat_Ok);
  success &= (myICM.enableDMPSensor(INV_ICM20948_SENSOR_RAW_ACCELEROMETER) == ICM_20948_Stat_Ok);
  success &= (myICM.enableDMPSensor(INV_ICM20948_SENSOR_MAGNETIC_FIELD_UNCALIBRATED) == ICM_20948_Stat_Ok);

  success &= (myICM.setDMPODRrate(DMP_ODR_Reg_Quat6, 10) == ICM_20948_Stat_Ok);        // Set to 5Hz
  success &= (myICM.setDMPODRrate(DMP_ODR_Reg_Accel, 54) == ICM_20948_Stat_Ok);        // Set to 1Hz
  success &= (myICM.setDMPODRrate(DMP_ODR_Reg_Gyro, 54) == ICM_20948_Stat_Ok);         // Set to 1Hz
  success &= (myICM.setDMPODRrate(DMP_ODR_Reg_Gyro_Calibr, 54) == ICM_20948_Stat_Ok);  // Set to 1Hz
  success &= (myICM.setDMPODRrate(DMP_ODR_Reg_Cpass, 54) == ICM_20948_Stat_Ok);        // Set to 1Hz
  success &= (myICM.setDMPODRrate(DMP_ODR_Reg_Cpass_Calibr, 54) == ICM_20948_Stat_Ok); // Set to 1Hz

  // Enable the FIFO
  success &= (myICM.enableFIFO() == ICM_20948_Stat_Ok);

  // Enable the DMP
  success &= (myICM.enableDMP() == ICM_20948_Stat_Ok);

  // Reset DMP
  success &= (myICM.resetDMP() == ICM_20948_Stat_Ok);

  // Reset FIFO
  success &= (myICM.resetFIFO() == ICM_20948_Stat_Ok);

  // Check success
  if (success)
  {
    Serial.println(F("DMP enabled!"));
  }
  else
  {
    Serial.println(F("Enable DMP failed!"));
    Serial.println(F("Please check that you have uncommented line 29 (#define ICM_20948_USE_DMP) in ICM_20948_C.h..."));
    while (1); // Do nothing more
  }  
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
  
  icm_20948_DMP_data_t data;
  myICM.readDMPdataFromFIFO(&data);

  if ((myICM.status == ICM_20948_Stat_Ok) || (myICM.status == ICM_20948_Stat_FIFOMoreDataAvail)) // Was valid data available?
  {
    if ((data.header & DMP_header_bitmap_Quat6) > 0) // Check for orientation data (Quat9)
    {
      // Scale to +/- 1
      double q1 = ((double)data.Quat6.Data.Q1) / 1073741824.0; // Convert to double. Divide by 2^30
      double q2 = ((double)data.Quat6.Data.Q2) / 1073741824.0; // Convert to double. Divide by 2^30
      double q3 = ((double)data.Quat6.Data.Q3) / 1073741824.0; // Convert to double. Divide by 2^30

      Serial.print(F("Q1:"));
      Serial.print(q1, 3);
      Serial.print(F(" Q2:"));
      Serial.print(q2, 3);
      Serial.print(F(" Q3:"));
      Serial.println(q3, 3);
    }

    if ((data.header & DMP_header_bitmap_Accel) > 0) // Check for Accel
    {
      float acc_x = (float)data.Raw_Accel.Data.X; // Extract the raw accelerometer data
      float acc_y = (float)data.Raw_Accel.Data.Y;
      float acc_z = (float)data.Raw_Accel.Data.Z;

      Serial.print(F("Accel: X:"));
      Serial.print(acc_x);
      Serial.print(F(" Y:"));
      Serial.print(acc_y);
      Serial.print(F(" Z:"));
      Serial.println(acc_z);
    }

    if ((data.header & DMP_header_bitmap_Gyro) > 0) // Check for Gyro
    {
      float x = (float)data.Raw_Gyro.Data.X; // Extract the raw gyro data
      float y = (float)data.Raw_Gyro.Data.Y;
      float z = (float)data.Raw_Gyro.Data.Z;

      Serial.print(F("Gyro: X:"));
      Serial.print(x);
      Serial.print(F(" Y:"));
      Serial.print(y);
      Serial.print(F(" Z:"));
      Serial.println(z);
    }

    if ((data.header & DMP_header_bitmap_Compass) > 0) // Check for Compass
    {
      float x = (float)data.Compass.Data.X; // Extract the compass data
      float y = (float)data.Compass.Data.Y;
      float z = (float)data.Compass.Data.Z;

      Serial.print(F("Compass: X:"));
      Serial.print(x);
      Serial.print(F(" Y:"));
      Serial.print(y);
      Serial.print(F(" Z:"));
      Serial.println(z);
    }
  }

  if (myICM.status != ICM_20948_Stat_FIFOMoreDataAvail) // If more data is available then we should read it right away - and not delay
  {
    delay(10);
  }
  delay(30);
}
