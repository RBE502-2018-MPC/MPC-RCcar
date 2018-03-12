#include <ros.h>                // header files sourced from  Step 3
#include <std_msgs/Bool.h>      
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>
#include <sensor_msgs/Joy.h>
ros::NodeHandle nh;

const char* estop_topic = "racer/teensy/estop";
const char* rpm_topic = "racer/teensy/rpm";
const char* pwr_topic = "racer/teensy/power";
const char* steer_topic = "racer/teensy/steer";
const char* debug_topic = "racer/teensy/debug";

int speedPin = 3;             //  pin 3 is for detecting speed
int LEDPin = 13;
int steeringPin = 5;
int powerPin = 6;

int pwm_center_power = 9830;  //  15% duty cycle - corresponds to zero velocity, zero steering
int pwm_center_steer = 10350; //  Calibrated center steering value in PWM

// volatile variables
volatile int pwm_steer = pwm_center_steer;
volatile int pwm_power = pwm_center_power;
volatile bool estop = false;
volatile unsigned long int lastPulseTime = micros();
volatile unsigned long int thisPulseTime;

// define publishers and subscribers
std_msgs::Float32 rpm_msg;                // creater a ROS Publisher called chatter of type str_msg
ros::Publisher RPM(rpm_topic, &rpm_msg);  // Set a publisher for publishing RPM info

std_msgs::Float32 debug_msg;              // creater a ROS Publisher called chatter of type str_msg
ros::Publisher DEBUG(debug_topic, &debug_msg);  // Set a publisher for publishing debugging message

// Set a subscriber for receiving steer signals
ros::Subscriber<std_msgs::Int32> sub_steer(steer_topic, &message_steer);

// Set a subscriber for receiving power signals
ros::Subscriber<std_msgs::Int32> sub_power(pwr_topic, &message_power);

// Set a subscriber for receiving emergency signals
ros::Subscriber<std_msgs::Bool> sub_estop(estop_topic, &message_estop);


void message_steer (const std_msgs::Int32& steer_msg){
  pwm_steer = steer_msg.data;
}

void message_estop( const std_msgs::Bool& estop_msg){
  estop = estop_msg.data;
}

void message_power( const std_msgs::Int32& power_msg){
  pwm_power = power_msg.data;
}

void setup() {
  Serial.begin(115200);
  analogWriteFrequency(5, 100);     //  freq at which PWM signals is generated at pin 5.
  analogWriteFrequency(6, 100); 
  analogWriteResolution(16);        // Resolution for the PWM signal
  analogWrite(steeringPin, pwm_center_steer);  // Setup zero velocity and steering.
  analogWrite(powerPin, pwm_center_power);
  // LED indicates the Teensy is powered on.
  pinMode(LEDPin,OUTPUT);
  digitalWrite(13,HIGH);            // Setup LED on.
  pinMode(speedPin, INPUT_PULLUP);  // Setup pin that read speed in RPS
  attachInterrupt(speedPin, speedISR, RISING);

  nh.initNode();                    // intialize ROS node
  nh.advertise(RPM);                // start the publishers...
  nh.advertise(DEBUG);
  nh.subscribe(sub_power);
  nh.subscribe(sub_estop);
  nh.subscribe(sub_steer);
}

void loop() {
  nh.spinOnce();

}

void speedISR(){
  lastPulseTime = thisPulseTime;
  thisPulseTime = micros();
}
