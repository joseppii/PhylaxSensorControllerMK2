#include <micro_ros_arduino.h>
#include <SBUS2.h>
#include <Wire.h>
#include <WiFi.h>
#include "SmcG2.h"
#include "encoder.h"
#include "PhylaxImu.h"
#include "Odometer.h"

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

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

PololuEncoder encoderLeft(34, 48, 0.03, 19, 18);  //Left motor encoder
PololuEncoder encoderRight(34, 48, 0.03, 17, 16); //Right motoer encoder
Odometer odom(encoderLeft, encoderRight);

rcl_publisher_t publisher;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

char agent_ip[] = "192.168.2.5";
uint agent_port = 8888;

unsigned long long time_offset = 0;
uint8_t FrameErrorRate = 0;
int16_t channel[16] = {0};

#define LED_PIN 13

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

PhylaxImu imu;

struct timespec getTime()
{
    struct timespec tp = {0};
    // add time difference between uC time and ROS time to
    // synchronize time with ROS
    unsigned long long now = millis() + time_offset;
    tp.tv_sec = now / 1000;
    tp.tv_nsec = (now % 1000) * 1000000;

    return tp;
}

void error_loop() 
{
  while(1){
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
  }
}

void timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{  
  RCLC_UNUSED(last_call_time);
  RCLC_UNUSED(timer);
}

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
  //I2C2.begin(SDA2,SCL2,(uint32_t)400000);

  Serial.println(F("Device connected!"));  
  set_microros_wifi_transports(ssid, password, agent_ip, agent_port);
  
  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());

  smc1.exitSafeStart();
  smc2.exitSafeStart();

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);  
  
  imu.startImu(I2C1, AD0_VAL);

  delay(2000);

  allocator = rcl_get_default_allocator();

  //create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  RCCHECK(rclc_node_init_default(&node, "phylax_sensor_node", "", &support));

  RCCHECK(imu.init(node, "imu/data"));
  RCCHECK(odom.init(node,"/odom", 0.25)); 

  // create timer,
  const unsigned int timer_timeout = 1000;
  RCCHECK(rclc_timer_init_default(
    &timer,
    &support,
    RCL_MS_TO_NS(timer_timeout),
    timer_callback));

  // create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
  }

void loop() {
  struct timespec time_stamp = getTime();

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
  
  imu.readData();
  odom.updateEncoders();
  //odom.evaluateRobotPose(time_diff)

  RCSOFTCHECK(imu.publish(time_stamp));
  RCSOFTCHECK(odom.publish_odom(time_stamp));

  delay(10);

}
