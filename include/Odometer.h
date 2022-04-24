#ifndef ODOMETRY_H
#define ODOMETRY_H

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <geometry_msgs/msg/transform_stamped.h>
#include <nav_msgs/msg/odometry.h>

class PololuEncoder;

class Odometer
{
public:
    Odometer(PololuEncoder& encoderLeft, PololuEncoder& encoderRight);
    ~Odometer();
    rcl_ret_t init(rcl_node_t& node, const char* topic_name, float track);
    void updateEncoders();
    void evaluateRobotPose(unsigned long diff_time);
    rcl_ret_t publish_odom(struct timespec time_stamp);

private:
    rcl_publisher_t odom_publisher_;   
    rcl_node_t* node_;
    nav_msgs__msg__Odometry* odom_msg_;
    
    PololuEncoder* const encoderLeft_;
    PololuEncoder* const encoderRight_;
    uint32_t encoderLeftCountPrev_;
    uint32_t encoderRightCountPrev_;

    float track_; 
    float dPhiL_;
    float dPhiR_;

    float th_;
    float x_;
    float y_;
    float vx_;
    float vTh_;
    float pathDistance_;   
};

#endif /* ODOMETRY_H */