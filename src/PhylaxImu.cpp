#include "PhylaxImu.h"

bool PhylaxImu::configure()
{
    bool success = true; // Use success to show if the DMP configuration was successful

    // Initialize the DMP. initializeDMP is a weak function. You can overwrite it if you want to 
    // e.g. to change the sample rate
    success &= (myICM_.initializeDMP() == ICM_20948_Stat_Ok);

    // Enable the DMP Game Rotation Vector sensor (Quat6)
    success &= (myICM_.enableDMPSensor(INV_ICM20948_SENSOR_GAME_ROTATION_VECTOR) == ICM_20948_Stat_Ok);

    // Enable additional sensors / features
    success &= (myICM_.enableDMPSensor(INV_ICM20948_SENSOR_RAW_GYROSCOPE) == ICM_20948_Stat_Ok);
    success &= (myICM_.enableDMPSensor(INV_ICM20948_SENSOR_RAW_ACCELEROMETER) == ICM_20948_Stat_Ok);
    success &= (myICM_.enableDMPSensor(INV_ICM20948_SENSOR_MAGNETIC_FIELD_UNCALIBRATED) == ICM_20948_Stat_Ok);

    success &= (myICM_.setDMPODRrate(DMP_ODR_Reg_Quat6, 10) == ICM_20948_Stat_Ok);        // Set to 5Hz
    success &= (myICM_.setDMPODRrate(DMP_ODR_Reg_Accel, 54) == ICM_20948_Stat_Ok);        // Set to 1Hz
    success &= (myICM_.setDMPODRrate(DMP_ODR_Reg_Gyro, 54) == ICM_20948_Stat_Ok);         // Set to 1Hz
    success &= (myICM_.setDMPODRrate(DMP_ODR_Reg_Gyro_Calibr, 54) == ICM_20948_Stat_Ok);  // Set to 1Hz
    success &= (myICM_.setDMPODRrate(DMP_ODR_Reg_Cpass, 54) == ICM_20948_Stat_Ok);        // Set to 1Hz
    success &= (myICM_.setDMPODRrate(DMP_ODR_Reg_Cpass_Calibr, 54) == ICM_20948_Stat_Ok); // Set to 1Hz

    // Enable the FIFO
    success &= (myICM_.enableFIFO() == ICM_20948_Stat_Ok);

    // Enable the DMP
    success &= (myICM_.enableDMP() == ICM_20948_Stat_Ok);

    // Reset DMP
    success &= (myICM_.resetDMP() == ICM_20948_Stat_Ok);

    // Reset FIFO
    success &= (myICM_.resetFIFO() == ICM_20948_Stat_Ok);

    // Check success
    if (success)
    {
        Serial.println(F("DMP enabled!"));
    }
    else
    {
        Serial.println(F("Enable DMP failed!"));
        Serial.println(F("Please check that you have uncommented line 29 (#define ICM_20948_USE_DMP) in ICM_20948_C.h..."));
        // Do nothing more
    }  
    return success;
}

bool PhylaxImu::startImu (TwoWire &i2c, uint8_t ad0) 
{
    bool status = true;
    uint8_t count = 5;
    while (count != 0)
    {
        myICM_.begin(i2c, ad0); // Initialize the ICM-20948

        Serial.print(F("Initialization of the IMU sensor returned: "));
        Serial.println(myICM_.statusString());
        if (myICM_.status != ICM_20948_Stat_Ok)
        {
            Serial.println(F("Trying again..."));
            count--;
            delay(500);
        }
        else
        {
            Serial.println(F("Device connected!"));
            status = false;
            count = 0;
        }
    }

    if (!status)
    {
        return this->configure();
    }

    Serial.println(F("Device cannot connect!"));
    return status;
}

void PhylaxImu::readData() 
{
    icm_20948_DMP_data_t data;
    myICM_.readDMPdataFromFIFO(&data);

    // Was valid data available?
    if ((myICM_.status == ICM_20948_Stat_Ok) || (myICM_.status == ICM_20948_Stat_FIFOMoreDataAvail))
    {
        if ((data.header & DMP_header_bitmap_Quat6) > 0) // Check for orientation data (Quat9)
        {
            // Scale to +/- 1
            double q1 = ((double)data.Quat6.Data.Q1) / 1073741824.0; // Convert to double. Divide by 2^30
            double q2 = ((double)data.Quat6.Data.Q2) / 1073741824.0; // Convert to double. Divide by 2^30
            double q3 = ((double)data.Quat6.Data.Q3) / 1073741824.0; // Convert to double. Divide by 2^30
/* 
            Serial.print(F("Q1:"));
            Serial.print(q1, 3);
            Serial.print(F(" Q2:"));
            Serial.print(q2, 3);
            Serial.print(F(" Q3:"));
            Serial.println(q3, 3); */
        }

        if ((data.header & DMP_header_bitmap_Accel) > 0) // Check for Accel
        {
            float acc_x = (float)data.Raw_Accel.Data.X; // Extract the raw accelerometer data
            float acc_y = (float)data.Raw_Accel.Data.Y;
            float acc_z = (float)data.Raw_Accel.Data.Z;

/*             Serial.print(F("Accel: X:"));
            Serial.print(acc_x);
            Serial.print(F(" Y:"));
            Serial.print(acc_y);
            Serial.print(F(" Z:"));
            Serial.println(acc_z); */
        }

        if ((data.header & DMP_header_bitmap_Gyro) > 0) // Check for Gyro
        {
            float x = (float)data.Raw_Gyro.Data.X; // Extract the raw gyro data
            float y = (float)data.Raw_Gyro.Data.Y;
            float z = (float)data.Raw_Gyro.Data.Z;

/*             Serial.print(F("Gyro: X:"));
            Serial.print(x);
            Serial.print(F(" Y:"));
            Serial.print(y);
            Serial.print(F(" Z:"));
            Serial.println(z); */
        }

        if ((data.header & DMP_header_bitmap_Compass) > 0) // Check for Compass
        {
            float x = (float)data.Compass.Data.X; // Extract the compass data
            float y = (float)data.Compass.Data.Y;
            float z = (float)data.Compass.Data.Z;
/* 
            Serial.print(F("Compass: X:"));
            Serial.print(x);
            Serial.print(F(" Y:"));
            Serial.print(y);
            Serial.print(F(" Z:"));
            Serial.println(z); */
        }
    }

    if (myICM_.status != ICM_20948_Stat_FIFOMoreDataAvail) // If more data is available then we should read it right away - and not delay
    {
        delay(10);
    }    
}