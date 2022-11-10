#include<ros.h>
#include<std_msgs/Int32.h>

ros::NodeHandle nh;

std_msgs::Int32 right_wheel_count;
ros::Publisher rightWheel("right_ticks", &right_wheel_count);

std_msgs::Int32 left_wheel_count;
ros::Publisher leftWheel("left_ticks", &left_wheel_count);

//Interrupt pin. Tracks the tick count.
#define leftEncA 21
#define rightEncA 22

// Tracks the direction of rotation.
#define leftEncB 20
#define rightEncB 19




// True = Forward; False = Reverse
boolean Direction_left = true;
boolean Direction_right = true;

//Number of wheels ticks
volatile int leftCounter = 0;
volatile int rightCounter = 0;

// Motor A connections
const int right_dir = 7;
const int right_pwm = 6;


// Motor B connections
const int left_dir = 8;
const int left_pwm = 9;

// Set linear velocity and PWM variable values for each wheel
double leftVel = 0;
double rightVel = 0;
double leftPwm = 0;
double rightPwm = 0;

// One-second interval for measurements
int interval = 1000;
long previousMillis = 0;
long currentMillis = 0;


void setup()
{
  //Ros setup
  nh.getHardware()->setBaud(115200);
  nh.initNode();
  nh.advertise( rightWheel );
  nh.advertise( leftWheel );

  // Set pin states of the encoder
  pinMode(leftEncA , INPUT_PULLUP);
  pinMode(leftEncB , INPUT);
  pinMode(rightEncA , INPUT_PULLUP);
  pinMode(rightEncB , INPUT);

  // Set the motor speed
  analogWrite(right_pwm, 100);
  analogWrite(left_pwm, 100);

  attachInterrupt(digitalPinToInterrupt(leftEncA), left_wheel_ticks, CHANGE);
  attachInterrupt(digitalPinToInterrupt(rightEncB), right_wheel_ticks, CHANGE);
}

void loop()
{
  rightWheel.publish(&right_wheel_count);
  leftWheel.publish(&left_wheel_count);
  nh.spinOnce();

}

void right_wheel_ticks()
{
  int val = digitalRead(rightEncB);

  if (val == LOW)
  {
    Direction_right = false; // Reverse
    rightCounter--;
  }
  else
  {
    Direction_right = true; // Forward
    rightCounter++;
  }
}

void left_wheel_ticks()
{
  int val = digitalRead(leftEncB);

  if (val == LOW) {
    Direction_left = false; // Reverse
    leftCounter--;
  }
  else {
    Direction_left = true; // Forward
    leftCounter++;
  }

  left_wheel_count.data = leftCounter;
}
