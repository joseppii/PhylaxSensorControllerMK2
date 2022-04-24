#include "Odometer.h"
#include "encoder.h"

Odometer::Odometer(PololuEncoder& encoderLeft, PololuEncoder& encoderRight) : encoderLeft_(&encoderLeft), encoderRight_(&encoderRight)
{
  odom_msg_ = nav_msgs__msg__Odometry__create();
  odom_msg_->header.frame_id.data = (char*)malloc(100*sizeof(char));
  char string1[] = "odom";
  memcpy(odom_msg_->header.frame_id.data, string1, strlen(string1) + 1);
  odom_msg_->header.frame_id.size = strlen(odom_msg_->header.frame_id.data);
  odom_msg_->header.frame_id.capacity = 100;    

  char string2[] = "base_link";
  odom_msg_->child_frame_id.data = (char*)malloc(100*sizeof(char));
  memcpy(odom_msg_->child_frame_id.data, string2, strlen(string2) + 1);
  odom_msg_->child_frame_id.size = strlen(odom_msg_->child_frame_id.data);
  odom_msg_->child_frame_id.capacity = 100;    

}

Odometer::~Odometer()
{
  nav_msgs__msg__Odometry__destroy(odom_msg_);
  rcl_ret_t ret = rcl_publisher_fini(&odom_publisher_, node_);
}

rcl_ret_t Odometer::init(rcl_node_t& node, const char* topic_name, float track)
{
  rcl_ret_t ret;
  node_ = &node;
  track_ = track;

  if (node_ == nullptr)
    return RCL_RET_NODE_INVALID;

  //Initialize pololu encoder(s)
  encoderLeft_->init();
  encoderRight_->init();

  // create Odom publisher
  ret = rclc_publisher_init_default( 
      &odom_publisher_, 
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry),
      topic_name
  );
    return ret;
}

void Odometer::updateEncoders()
{
  uint32_t encoderLeftCount;
  uint32_t encoderRightCount;

  encoderLeft_->update(encoderLeftCount);
  encoderRight_->update(encoderRightCount);

  int32_t dEncoderLeft = (encoderLeftCount - encoderLeftCountPrev_);
  int32_t dEncoderRight = (encoderRightCount - encoderRightCountPrev_);

  //update the angle increment in radians
  float dphi1 = ((float)dEncoderLeft * encoderLeft_->_enc2rad);
  float dphi2 = ((float)dEncoderRight * encoderRight_->_enc2rad);
  
  //for encoder index and motor position switching (Right is 1, Left is 2)
  dPhiR_ = dphi1;
  dPhiL_ = dphi2;
  
  encoderLeftCountPrev_  = encoderLeftCount;
  encoderRightCountPrev_ = encoderRightCount;
}

void Odometer::evaluateRobotPose(unsigned long diff_time)
{
  float dTh = encoderLeft_->_wheelRadius/(track_) *(dPhiR_ - dPhiL_);
  float dist = encoderLeft_->_wheelRadius *(dPhiR_ + dPhiL_) / 2;
  float dx = encoderLeft_->_wheelRadius/2 * (cos(th_)*dPhiR_ + cos(th_)*dPhiL_);
  float dy = encoderLeft_->_wheelRadius/2 * (sin(th_)*dPhiR_+ + sin(th_)*dPhiL_);
  long dt = float(diff_time)/1000;
  
  th_+= dTh;
  x_+=dx;
  y_+=dy;
  vx_ = dist/dt;
  vTh_ = dTh/dt;
  pathDistance_ = pathDistance_ + sqrt(dx*dx + dy*dy);

  Serial.println("Path distance = "+String((int32_t)pathDistance_));
}
 
rcl_ret_t Odometer::publish_odom(struct timespec time_stamp) 
{
    
  odom_msg_->pose.pose.position.x  = x_;
  odom_msg_->pose.pose.position.y  = y_;
  odom_msg_->pose.pose.position.z  = 0.0;
 // odom_msg_->pose.pose.orientation = tf::createQuaternionFromYaw(_th);

  odom_msg_->twist.twist.linear.x  = vx_;
  odom_msg_->twist.twist.linear.y  = 0;

  odom_msg_->twist.twist.angular.x = 0.0;
  odom_msg_->twist.twist.angular.y = 0.0;  
  odom_msg_->twist.twist.angular.z = vTh_; 

  odom_msg_->twist.covariance[0]  = 0.0001;
  odom_msg_->twist.covariance[7]  = 0.0001;
  odom_msg_->twist.covariance[35] = 0.0001;

  odom_msg_->header.stamp.sec = time_stamp.tv_sec;
  odom_msg_->header.stamp.nanosec = time_stamp.tv_nsec;
  return rcl_publish(&odom_publisher_, odom_msg_, NULL);

}