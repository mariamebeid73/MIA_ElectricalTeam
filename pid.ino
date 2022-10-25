#include<ros.h>
#include<std_msgs/Float32.h>

ros::NodeHandle nh;

#define pinA 2
#define pinB 3
#define dir 7
#define pmw 6
#define resolution 2400

//PID constants
double kp = 2;
double ki = 5;
double kd = 1;

unsigned long currentTime, previousTime;
double elapsedTime;
double error;
double lastError;
double cumError, dVelocity;

int count = 0;
int last_count = 0;
int steps_per_second = 0;
int velocity = 0;
int last_velocity = 0;

void pid (const std_msgs::Float32& setpoint_msg)
{
  static float setpoint = setpoint_msg.data;

  currentTime = millis();                //get current time
  elapsedTime = (double)(currentTime - previousTime);        //compute time elapsed from previous computation

  steps_per_second = (count - last_count) * 1000 / (elapsedTime);    //counts per second
  velocity = steps_per_second / resolution;                 //revolution per second

  error = setpoint - velocity;                                // determine error
  cumError += error * elapsedTime;                            // compute integral
  dVelocity = (velocity - last_velocity) / elapsedTime;       // compute derivative

  double newVelocity = kp * error + ki * cumError - kd * dVelocity;          //PID output

  //lastError = error;                      //save current error
  last_count = count;
  last_velocity = velocity;          //save current velocity to overcome derivative kick
  previousTime = currentTime;                        //save current time


  analogWrite(pmw, newVelocity);

}

ros::Subscriber<std_msgs::Float32> control("SETPOINT", &pid);

void setup()
{

  nh.initNode();
  nh.subscribe(control);

  pinMode(pinA, INPUT_PULLUP);
  pinMode(pinB, INPUT_PULLUP);
  pinMode(pmw, OUTPUT);
  pinMode(dir, OUTPUT);

  attachInterrupt(digitalPinToInterrupt(pinA), isrA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(pinB), isrB, CHANGE);

  Serial.begin(9600);
}

void loop()
{
  nh.spinOnce();
  delay(50);
}


void isrA()
{
  if (digitalRead(pinA) != digitalRead(pinB))
    count++;
  else
    count--;
}


void isrB()
{
  if (digitalRead(pinA) == digitalRead(pinB))
    count++;
  else
    count--;
}
