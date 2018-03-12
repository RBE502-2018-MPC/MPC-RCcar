#include <ros.h>                // header files sourced from  Step 3
#include <std_msgs/Bool.h>      
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>
#include <sensor_msgs/Joy.h>
ros::NodeHandle nh;


boolean flagStop = false; 
int pwm_center_value = 9830;  //  15% duty cycle - corresponds to zero velocity, zero steering
int pwm_forwardMin = 10079;   //  15.38% duty cycle - correspond to minimum moving forward velocity
int pwm_backwardMin = 9522;   //  14.52% duty cycle - correspond to minimum moving backward velocity
int pwm_lowerlimit = 6554;    //  10% duty cycle - corresponds to max reverse velocity, extreme left steering
int pwm_upperlimit = 13108;   //  20% duty cycle - corresponds to max forward velocity, extreme right steering
int pwm_center_steer = 10350; //  Calibrated center steering value in PWM
int pwm_steer = pwm_center_steer;
int safePower = 25;             //  safe power in percentages -- the output torque won't exceed 80% of maximum power
int speedPin = 3;             //  pin 3 is for detecting speed
int LEDPin = 2;
float mass = 4.5;             //  car's mass is 4.5 kg
int i = 10100;
int state = 0;

const char* pid_topic = "racer/teensy/pidinfo";
const char* rpm_topic = "racer/teensy/rpm";
const char* estop_topic = "racer/teensy/estop";
const char* pwr_topic = "racer/teensy/power";
const char* debug_topic = "racer/teensy/debug";
const char* steer_topic = "racer/teensy/steer";

float kp = 3.5;                   // Initialize Kp, Ki, kd value as zero, and the car will be stable in the beginning.
float ki = 0.001;
float kd = 0.5;
volatile float desiredSpeed = 0;
volatile float heartBeat = 0;     // Initialize heartBeat. When the heartBeat is 1, it will move; if 0, it stops. 
float const_power_test = 0;       // Initialize constant power in percentage (for constant power test)  
volatile bool estop = false;      // Initialize the emergency stop as false. If it's true, then the car will emergency stop.
float error;                      // error value for PID control
volatile float prev_error = 0.0;  // saved previous error for calculating derivative part of PID.
volatile float acc_error = 0.0;
volatile float currentSpeed = 0;  // current motor's output speed in RPS.
volatile float acceleration = 0;
volatile float lastCheckSpeed;
volatile long unsigned int timeInterval;
volatile long unsigned int currentTime;
volatile long unsigned int lastCheckTime;
volatile float pidOut;
volatile float speedTrend[6] = {0,0,0,0,0,0};
volatile int speedCheckAcc = 0;
volatile long unsigned int WTHLastTime = 0;

volatile unsigned long int currentPub;
volatile unsigned long int lastPub = 0;
volatile unsigned long int cur = micros();

volatile unsigned long int lastHighTime = 0;
volatile unsigned long int lastLowTime = 0;
volatile int lowSpeedState = 0;

std_msgs::Float32 rpm_msg;                // creater a ROS Publisher called chatter of type str_msg
ros::Publisher RPM(rpm_topic, &rpm_msg);  // Set a publisher for publishing RPM info

std_msgs::Float32 pwr_msg;                // creater a ROS Publisher called chatter of type str_msg
ros::Publisher PWR(pwr_topic, &pwr_msg);  // Set a publisher for publishing power info

std_msgs::Float32 debug_msg;              // creater a ROS Publisher called chatter of type str_msg
ros::Publisher DEBUG(debug_topic, &debug_msg);  // Set a publisher for publishing debugging message

// Set a subscriber for receiving steer signals
ros::Subscriber<std_msgs::Int32> sub_steer(steer_topic, &message_steer);

// Set a subscriber for receiving PID control signals
ros::Subscriber<std_msgs::Float32MultiArray> sub_pid(pid_topic, &message_pid);

// Set a subscriber for receiving emergency signals
ros::Subscriber<std_msgs::Bool> sub_estop(estop_topic, &message_estop);


void message_steer (const std_msgs::Int32& steer_msg){
  pwm_steer = steer_msg.data;
}

void message_estop( const std_msgs::Bool& estop_msg){
  estop = estop_msg.data;
}

void message_pid( const std_msgs::Float32MultiArray& pid_msg){
  kp = pid_msg.data[0];
  ki = pid_msg.data[1];
  kd = pid_msg.data[2];
  desiredSpeed = pid_msg.data[3];
  heartBeat = pid_msg.data[4];
  const_power_test = pid_msg.data[5];
}

void setup() {
  Serial.begin(115200);
  // Need to produce PWM signals so we need to setup the PWM registers. This setup happens next.
  analogWriteFrequency(5, 100);     //  freq at which PWM signals is generated at pin 5.
  analogWriteFrequency(6, 100); 
  analogWriteResolution(16);        // Resolution for the PWM signal
  analogWrite(5,pwm_center_steer);  // Setup zero velocity and steering.
  analogWrite(6,pwm_center_value);
  pinMode(13,OUTPUT);               // Teensy's onboard LED pin. 
  pinMode(LEDPin,OUTPUT);
  digitalWrite(13,HIGH);            // Setup LED.
  digitalWrite(LEDPin,LOW);
  pinMode(speedPin, INPUT_PULLUP);  // Setup pin that read speed in RPS
  attachInterrupt(speedPin, speedISR, RISING);

  nh.initNode();                    // intialize ROS node
  nh.advertise(RPM);                // start the publishers...
  nh.advertise(PWR);
  nh.advertise(DEBUG);
  nh.subscribe(sub_pid);
  nh.subscribe(sub_estop);
  nh.subscribe(sub_steer);
}


void speedISR(){
  lastPub = currentPub;
  currentPub = micros();
}


void loop() {
  nh.spinOnce();
  currentSpeed = readSpeed();
  currentTime = micros();
  timeInterval = currentTime - lastCheckTime;

  if (currentTime - WTHLastTime > 100000){
    pubSpeed(currentSpeed);
    speedWTH();
    WTHLastTime = currentTime;
  }

  analogWrite(5,pwm_steer);
  if (estop == false){
    if (desiredSpeed >= 15){
      pidCtrl();
    }
    else if (desiredSpeed >= 1){
      if (currentSpeed >= 15){
        pidCtrl();
      } else {
        lowSpeedCtrl();
      }
    } 
    else {                        // When the command is Stop..
      if (currentSpeed >= 1){
        pidCtrl();
      } else {
        analogWrite(6,pwm_center_value);
      }
    }
  }
  else {
    digitalWrite(LEDPin,HIGH);
    analogWrite(6,pwm_center_value); // was 7373, 75% of maximum reverse power.
    acc_error = 0;
    prev_error = 0;
    estop = false;
    heartBeat = 0;
    debug_msg.data = 666;
    DEBUG.publish( &debug_msg );
  }
  lastCheckSpeed = currentSpeed;
  lastCheckTime = currentTime;
  
}


void lowSpeedCtrl2(){
  if ((int)heartBeat > 0){
    digitalWrite(LEDPin,LOW);
    switch (state){
      case 0:
        if (i>10055){
          analogWrite(6,i);
          i = i - 1;
        } else {
          state = state + 1;
        }
      break;
      case 1:
        if (i<10080){
          analogWrite(6,i);
          i = i + 1;
        } else {
          state = state - 1;
        }
        break;
    }
  }
}



void lowSpeedCtrl(){
  if ((int)heartBeat > 0){
    digitalWrite(LEDPin,LOW);
    switch (lowSpeedState){
      case 0:
        if (millis() - lastHighTime < 100) {
          analogWrite(6,10945);
        } else {
          lastHighTime = millis();
          lastLowTime = millis();
          lowSpeedState == 1;
        }
      break;
      case 1:
        if (millis() - lastLowTime < 70) {
          analogWrite(6,pwm_center_value);
        } else {
          lastHighTime = millis();
          lastLowTime = millis();
          lowSpeedState == 0;
        }
      break;  
    }
  }
}

void pidCtrl(){
  if ((int)heartBeat > 0){
    digitalWrite(LEDPin,LOW);
    float PIDoutVal = PIDOutput();
    analogWrite(6, mapSig2PWM(PIDoutVal));
  }
  else {
    analogWrite(6,pwm_center_value);
  }
}

/*
float readSpeed(){
  volatile long int durationInMicro = pulseIn(speedPin, HIGH, 200000);
  float rawSpeed = 1000000.0/(float)durationInMicro;
  if (rawSpeed >= 1000.0){
    rawSpeed = 0;
  }
  return rawSpeed;
}
*/

float readSpeed(){
  if(currentTime - cur > 100000) {
    cur = currentTime;
    volatile unsigned long int pubTimeInterval = currentPub - lastPub;
    volatile float speed;
    if ((100 <= pubTimeInterval) && (pubTimeInterval <= 1000000)) {
      speed = 1000000.0/pubTimeInterval;
      lastPub = currentPub;
      currentSpeed = speed;
    } else {
      lastPub = currentPub;
      currentSpeed = 0.0;
    }
    pubSpeed(currentSpeed);
  }
  /*
   * else {
    nh.spinOnce();
    }
   */
return currentSpeed;
}

void pubSpeed (float speed){
  rpm_msg.data = speed;
  RPM.publish( &rpm_msg );
 // delay(10);
}

/*
 * Speed to traction calculation: the performance corresponds the following 
 * quadratic equations:
 * 
 * /
 * |  F(v,a) = 0.0713 * v^2 - 1.54 * v + 44.3 + m * a   --  (10.8 <= v)
 * |  F(v,a) = 36 + m * a                               --  (0 <= v < 10.8)
 * |  F(v,a) = -36 + m * a                              --  (-10.8 <= v < 0)
 * |  F(v,a) = -0.0713 * v^2 + 1.54 * v - 44.3 + m * a  --  (v <= -10.8)
 * \
 *
 */
float speed2traction(float speed, float mass, float acceleration) {
  float traction;
  if (speed >= 10.8){
    traction = (0.0713 * sq(speed)) - (1.54 * speed) + 44.3 + mass * acceleration;
  } else if (speed >= 0){
    traction = 36 + mass * acceleration;
  } else if (speed >= -10.8){
    traction = -36 + mass * acceleration;
  } else {
    traction = -(0.0713 * sq(speed)) - (1.54 * speed) - 44.3 + mass * acceleration;
  }
  return traction;
}

float getAcceleration(){
  acceleration = (currentSpeed - lastCheckSpeed)/timeInterval;
  return acceleration;
}

// Error calculation based on friction model (need to be improved)
float tractionErr(float currentSpeed, float desiredSpeed) {
  float outputVal;
  outputVal = speed2traction(desiredSpeed, mass, 0) - speed2traction(currentSpeed, mass, getAcceleration());
  return outputVal;
}

// PID output calculation and output corresponding PWM signal to motor.
float PIDOutput(){
  error = tractionErr(currentSpeed, desiredSpeed);
  pidOut = kp * error + kd * ((error - prev_error)/timeInterval) + ki * acc_error * timeInterval;
  if (pidOut >= safePower){
    pidOut = safePower;
  } else if (pidOut <=(-1*safePower)) {
    pidOut = -1*safePower;
  }

  if ((pidOut < -0.1) && (currentSpeed < 10)){
    pidOut = 0;
  }
  
  pwr_msg.data = pidOut;
  PWR.publish( &pwr_msg );
  
  prev_error = error;
  acc_error = acc_error + error;
  return pidOut;
}

int mapSig2PWM(float pidOut){
  int pidSig;
  if (pidOut >= 0.0){
    pidSig = (int)map(pidOut, 0, 100, pwm_forwardMin, pwm_upperlimit);
  }
  else {
    pidSig = (int)map(pidOut, -100, 0, pwm_lowerlimit, pwm_backwardMin);
  }
  return pidSig;
}

void speedWTH(){
  for (int j=5; j<0; j--){
     speedTrend[j] = speedTrend[j-1];
  }
  speedTrend[0] = currentSpeed;
  speedCheckAcc++;
  if ((pidOut < -4.5) && (speedCheckAcc >= 5)){
    if ((speedTrend[0] > speedTrend[4]) && (speedTrend[0] > speedTrend[5])){// && (speedTrend[2] > speedTrend[3])){
      analogWrite(6, mapSig2PWM(0));
      while (readSpeed() >= 0.1){
        currentTime = micros();
      }
      acc_error = 0;
      prev_error = 0;
      heartBeat = 0;
      
      debug_msg.data = 10101;
      DEBUG.publish( &debug_msg );
    }
    speedCheckAcc = 0;
  }
}

