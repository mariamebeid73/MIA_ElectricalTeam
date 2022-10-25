#include <ros.h>

#include <std_msgs/Float32.h>

ros::NodeHandle nh;
std_msgs::Float32 count_msg;
ros::Publisher encoder("Encoder", &count_msg);

#define pinA 2
#define pinB 3

int counter = 0;
int publisher_timer = 0;


void setup()
{
  Serial.begin(9600);
  nh.initNode();
//  nh.advertise(encoder);/
//  pinMode(pinA, INPUT_PULLUP);
//  pinMode(pinB, INPUT_PULLUP);
//
//  attachInterrupt(digitalPinToInterrupt(pinA), isrA, CHANGE);
//  attachInterrupt(digitalPinToInterrupt(pinB), isrB, CHANGE);
}

void loop()
{

//  count_msg.data = counter;
//  encoder.publish(&count_msg);
  nh.spinOnce();

}

void isrA()
{
  if (digitalRead(pinA) != digitalRead(pinB))
    counter ++;
  else
    counter --;
}

void isrB()
{
  if (digitalRead(pinA) == digitalRead(pinB))
    counter ++;
  else
    counter --;
}
