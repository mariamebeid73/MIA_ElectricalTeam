#include<ros.h>
#include<std_msgs/Float32.h>

std_msgs::Float32 filteredYaw;
ros::NodeHandle nh;
ros::Publisher filter("Filtered", &filteredYaw);

void KALMANfilter (const std_msgs::Float32& yaw_msg)
{
  static double z = yaw_msg.data;   //sensor reading
  static const double R = 3;     //sensor variance
  static double Q = 2;           //process variance
  static double P = 7;           //uncertainity
  static double P_hat = 0;        //prediction uncertainity
  static double X = 20;            //state variable
  static double X_hat = 0;        //predicted state
  static double K = 0;            //kalman gain
  static double y = 0;            //residual

  //state prediction
  X_hat = X;
  P_hat = P + Q;

  //measurement update 
  y = z - X_hat;
  K = P_hat / (P_hat + R);       //update kalman gain
  X = X_hat + K * y;             //update predicted state 
  P = (1 - K) * P_hat;           //update uncertainity     

  filteredYaw.data = X;          //publish filtered reading
  filter.publish(&filteredYaw);
}

ros::Subscriber<std_msgs::Float32> mpuAngles("YAW_ANGLE", &KALMANfilter);

void setup() 
{
  nh.initNode();
  nh.subscribe(mpuAngles);  //subscribe to topic mpuAngles
  nh.advertise(filter);    
}

void loop()
{
  nh.spinOnce();
  delay(1);
}
