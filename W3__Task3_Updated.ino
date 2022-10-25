#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float32
Filtered_Yaw = 0

def sub_callback(msg):
   z = msg.data ##  sensor reading
   R = 1    ##sensor variance
   Q = 20          ##process variance
   P = 7          ##uncertainity
   P_hat = 0       ##prediction uncertainity
   X = 0           ##state variable
   X_hat = 0        ##predicted state
   K = 0            ##kalman gain
   y = 0           ##residual
   X_hat = X
   P_hat = P + Q

  ##measurement update 
   y = z - X_hat
   K = P_hat / (P_hat + R)      ##update kalman gain
   X = X_hat *  (1 -K) + K * y            ##update predicted state 
   P = (1 - K) * P_hat          ##update uncertainity     
   global Filtered_Yaw 
   Filtered_Yaw = X

rospy.init_node('KalmanFilter')
sub = rospy.Subscriber('YAW_ANGLE', Float32, sub_callback)
pub = rospy.Publisher('Filtered_Yaw', Float32,queue_size=10)
rate = rospy.Rate(10)
count = 0
while not rospy.is_shutdown():
    pub.publish(Filtered_Yaw)
    count += 1
    rate.sleep()
rospy.spin()
