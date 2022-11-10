#include "ros.h"
#include "std_msgs/Int32.h"
#include <geometry_msgs/Pose.h>
#include <sensor_msgs/Imu.h>

ros::NodeHandle nh;

geometry_msgs::Pose msg_pose;
ros::Publisher odomData ("pose",&msg_pose);

#define resolution 2400
#define wheel_radius 0.25
#define ticks_per_meter 3100
////// Distance from center of the left tire to the center of the right tire in m
const double WHEEL_BASE = 0.17;

// Initial pose
const double initialX = 0.0;
const double initialY = 0.0;
const double initialTheta = 0.00000000001;
const double PI = 3.141592;
float yaw_angle;

// Distance both wheels have traveled
double distanceLeft = 0;
double distanceRight = 0;

void imu_reading(const sensor_msgs::Imu& imu_msg)
{
  yaw_angle = imu_msg.orientation.z;
}

void Calc_Right(const std_msgs::Int32& right_count)
{
  static int lastCounter = 0;
  int rightTicks = right_count.data - lastCounter;
  distanceRight = rightTicks / ticks_per_meter;
  lastCounter = right_count.data;
}

void Calc_Left(const std_msgs::Int32& left_count)
{
  static int lastCounter = 0;
  int leftTicks = left_count.data - lastCounter;
  distanceLeft = leftTicks / ticks_per_meter;
  lastCounter =left_count.data;
}

ros::Subscriber<sensor_msgs::Imu> imuData("Imu6050", &imu_reading);
ros::Subscriber<std_msgs::Int32 > right_ticks("right_ticks", &Calc_Right);
ros::Subscriber<std_msgs::Int32 > left_ticks("left_ticks", &Calc_Left);

// Update odometry information
void update_odom() 
{ 
  // Calculate the average distance
  double cycleDistance = (distanceRight + distanceLeft) / 2;

  msg_pose.position.x += cos(yaw_angle) * cycleDistance;
  msg_pose.position.y += sin(yaw_angle) * cycleDistance;
  msg_pose.position.z =0;  

  msg_pose.orientation.x = 0;
  msg_pose.orientation.y = 0;
  msg_pose.orientation.z = yaw_angle ;
  msg_pose.orientation.w = 0;

  odomData.publish(&msg_pose);

}

void setup()
{
  nh.initNode();
  nh.subscribe(imuData);
  nh.subscribe(right_ticks);
  nh.subscribe(left_ticks);
  nh.advertise(odomData);
}

void loop()
{
  nh.spinOnce();
}
