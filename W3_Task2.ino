#include<ros.h>
#include<std_msgs/Int32.h>

std_msgs::Int32 count_msg;
ros::NodeHandle nh;
ros::Publisher encoder("Encoder",&count_msg);

#define pinA 21
#define pinB 22

int counter = 0;
int publisher_timer = 0;


void setup()
{
  nh.initNode();
  nh.advertise( encoder );
  pinMode(pinA,INPUT_PULLUP);
  pinMode(pinB,INPUT_PULLUP);  

  attachInterrupt(digitalPinToInterrupt(pinA),isrA,CHANGE);
  attachInterrupt(digitalPinToInterrupt(pinB),isrB,CHANGE);
}

void loop() 
{
  if(millis() > publisher_timer)
  {
    count_msg.data = counter;
    encoder.publish(&count_msg);
    publisher_timer = millis() + 1000;
  }
  nh.spinOnce();
  
}

void isrA()
{ 
  if(digitalRead(pinA) != digitalRead(pinB))
    counter ++;
   else
    counter --;
}

void isrB()
{
  if(digitalRead(pinA) == digitalRead(pinB)) 
    counter ++;
  else
    counter --; 
}
