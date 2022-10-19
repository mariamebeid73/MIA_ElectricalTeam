#include<ros.h>
#include<std_msgs/Float32.h>
#include "MPU6050_6Axis_MotionApps612.h"
#include "MPU6050.h" 
#include "Wire.h"

MPU6050 mpu;

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorFloat gravity;    // [x, y, z]            gravity vector
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

unsigned long publisher_timer = 0;
float yawAngle;

//setup the rosNode and the publisher
ros::NodeHandle nh;
std_msgs::Float32 yaw_msg;
ros::Publisher mpuAngles("YAW_ANGLE",&yaw_msg);




void setup()
{
  Wire.begin();
  mpu.initialize();
  mpu.dmpInitialize();     // load and configure the DMP

  // supply your own gyro offsets here, scaled for min sensitivity
  mpu.setXGyroOffset(51);
  mpu.setYGyroOffset(8);
  mpu.setZGyroOffset(21);
  mpu.setXAccelOffset(1150);
  mpu.setYAccelOffset(-50);
  mpu.setZAccelOffset(1060);
  
  // Calibration Time: generate offsets and calibrate our MPU6050
  mpu.CalibrateAccel(6);
  mpu.CalibrateGyro(6);
  mpu.setDMPEnabled(true);

  nh.initNode();
  nh.advertise(mpuAngles);
 }

void loop()
{
  if((millis() - publisher_timer) > 10)  //publish every 10ms 
  {
       // display Euler angles in degrees
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

    yawAngle = ypr[0] * 180 / M_PI; 
    yaw_msg.data = yawAngle;
    mpuAngles.publish( &yaw_msg );
    publisher_timer = millis();
    
  }
   nh.spinOnce();
}
