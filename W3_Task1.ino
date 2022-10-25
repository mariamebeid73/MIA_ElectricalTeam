#include<ros.h>
#include<std_msgs/Float32.h>
#include "MPU6050_6Axis_MotionApps612.h"
#include "MPU6050.h"
#include "Wire.h"

MPU6050 mpu;

// MPU control/status vars
/*bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
*/uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 gy;         // [x, y, z]            gyro sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

unsigned long publisher_timer = 0;
float yawAngle;

//setup the rosNode and the publisher
ros::NodeHandle nh;
std_msgs::Float32 yaw_msg;
ros::Publisher mpuAngles("YAW_ANGLE", &yaw_msg);




void setup()
{
  Serial.begin(9600);
  Wire.begin();
  Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
  mpu.initialize();
  mpu.dmpInitialize();     // load and configure the DMP
  nh.initNode();
  nh.advertise(mpuAngles);

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
  // get expected DMP packet size for later comparison
  //packetSize = mpu.dmpGetFIFOPacketSize();

  //mpuIntStatus = mpu.getIntStatus();
}

void loop()
{
  mpu.dmpGetCurrentFIFOPacket(fifoBuffer);
  mpu.dmpGetQuaternion(&q, fifoBuffer);

  // display Euler angles in degrees
  mpu.dmpGetGravity(&gravity, &q);
  mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

  yawAngle = ypr[0] * 180 / M_PI;
 yaw_msg.data = 00;         ////////
 //Serial.println(yaw_msg.data);
 mpuAngles.publish( &yaw_msg );



  nh.spinOnce();
}
