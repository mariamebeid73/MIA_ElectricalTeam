#include<ros.h>
#include<std_msgs/Float32.h>
#include<MPU6050_light.h>
#include "Wire.h"

MPU6050 mpu(Wire);

//setup the rosNode and the publisher
ros::NodeHandle nh;
std_msgs::Float32 yaw_msg;
ros::Publisher mpuAngles("YAW_ANGLE",&yaw_msg);

unsigned long publisher_timer = 0;
float xAngle, yAngle, zAngle;

void setup()
{
  Wire.begin();
  byte status = mpu.begin();
  mpu.calcOffsets();
  nh.initNode();
  nh.advertise(mpuAngles);

}

void loop()
{
  mpu.update();
  if((millis() - publisher_timer) > 10) //publish every 10 ms
    { 
        xAngle = mpu.getAngleX();
        yAngle = mpu.getAngleY();
        zAngle = mpu.getAngleZ();
        yaw_msg.data = zAngle;
        mpuAngles.publish( &yaw_msg );
        publisher_timer = millis();
    }

   nh.spinOnce();
}
