#include<ros.h>
#include <sensor_msgs/Imu.h>
#include "MPU6050_6Axis_MotionApps612.h"
#include "MPU6050.h"
#include "Wire.h"

MPU6050 mpu;

uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorFloat gravity;    // [x, y, z]            gravity vector
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container

long sequence = 0;
float yawAngle;

//setup the rosNode and the publisher
ros::NodeHandle nh;
sensor_msgs::Imu imu_msg;
ros::Publisher imuAngles("Imu6050", &imu_msg);


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
  mpu.dmpGetFIFOPacketSize();

  nh.initNode();
  nh.advertise(imuAngles);
}

void loop()
{

  mpu.dmpGetCurrentFIFOPacket(fifoBuffer);
  // display Euler angles in degrees
  mpu.dmpGetQuaternion(&q, fifoBuffer);
  mpu.dmpGetEuler(euler, &q);
  mpu.dmpGetGravity(&gravity, &q);
  mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

  yawAngle = ypr[0] * 180 / M_PI;
 

  //Header
  imu_msg.header.seq = sequence++;
  imu_msg.header.stamp = nh.now();
  imu_msg.header.frame_id  ="?";

  //Linear Acceleration 
  imu_msg.linear_acceleration.x = 0 ;
  imu_msg.linear_acceleration.y = 0 ;
  imu_msg.linear_acceleration.z = 0 ;
  
  //Angular Velocity
  imu_msg.angular_velocity.x = 0 ;
  imu_msg.angular_velocity.y = 0 ; 
  imu_msg.angular_velocity.z = 0 ;  

  //Orientation
  imu_msg.orientation.x = 0 ;
  imu_msg.orientation.y = 0 ;
  imu_msg.orientation.z = yawAngle ;
  imu_msg.orientation.w = 0 ;


  imuAngles.publish( &imu_msg );

  nh.spinOnce();
}
