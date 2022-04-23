#include "RosImu.h"

RosImu::RosImu()
{
    imu_msg_ = sensor_msgs__msg__Imu__create();
    imu_msg_->header.frame_id.data = (char*)malloc(100*sizeof(char));
    char string3[] = "/imu_link";
    memcpy(imu_msg_->header.frame_id.data, string3, strlen(string3) + 1);
    imu_msg_->header.frame_id.size = strlen(imu_msg_->header.frame_id.data);
    imu_msg_->header.frame_id.capacity = 100;    
    
    imu_msg_->linear_acceleration_covariance[0] = accel_covariance_;
    imu_msg_->linear_acceleration_covariance[4] = accel_covariance_;
    imu_msg_->linear_acceleration_covariance[8] = accel_covariance_;    

    imu_msg_->angular_velocity_covariance[0] = gyro_covariance_;
    imu_msg_->angular_velocity_covariance[4] = gyro_covariance_;
    imu_msg_->angular_velocity_covariance[8] = gyro_covariance_;
}

RosImu::~RosImu()
{
  sensor_msgs__msg__Imu__destroy(imu_msg_);
  rcl_ret_t ret = rcl_publisher_fini(&imu_publisher_, node_);
}

rcl_ret_t RosImu::init(rcl_node_t& node, const char* topic_name)
{
    rcl_ret_t ret;
    node_ = &node;

    if (node_ == nullptr)
      return RCL_RET_NODE_INVALID;

    // create IMU publisher
    ret = rclc_publisher_init_default( 
        &imu_publisher_, 
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
        topic_name
    );
    return ret;
}

rcl_ret_t RosImu::publish(struct timespec time_stamp)
{
  imu_msg_->header.stamp.sec = time_stamp.tv_sec;
  imu_msg_->header.stamp.nanosec = time_stamp.tv_nsec;
  return rcl_publish(&imu_publisher_, imu_msg_, NULL);
}