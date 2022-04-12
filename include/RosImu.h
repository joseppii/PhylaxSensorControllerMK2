#ifndef ROSIMU_H
#define ROSIMY_H

#include "ICM_20948.h"
#include <sensor_msgs/msg/imu.h>

class RosImu 
{
public:
    RosImu(ICM_20948_I2C& myICM);
    void begin(TwoWire& i2c, uint8_t ad0);
    void init();
    void read();

private:
    ICM_20948_I2C* const myICM_; //Create an ICM_20948_I2C object
    sensor_msgs__msg__Imu imu_msg_;
};

#endif /* ROSIMU_H */