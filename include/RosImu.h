#ifndef ROSIMU_H
#define ROSIMY_H

#include "Wire.h"
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <sensor_msgs/msg/imu.h>

class RosImu 
{
public:
    RosImu();
    ~RosImu();
    rcl_ret_t init(rcl_node_t& node, const char* topic_name);
    rcl_ret_t publish(struct timespec time_stamp);

protected:
    virtual bool startImu(TwoWire &i2c, uint8_t ad0) = 0;
    virtual void readData() = 0;

private:
    rcl_publisher_t imu_publisher_;   
    rcl_node_t* node_;
    sensor_msgs__msg__Imu* imu_msg_;

    const float g_to_accel_ = 9.81;
    const float mgauss_to_utesla_ = 0.1;
    float accel_covariance_ = 0.00001;
    float gyro_covariance_ = 0.00001;
};

#endif /* ROSIMU_H */