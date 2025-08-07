#include <ros.h>            // Library to communicate to ROS
#include <Stepper.h>        // Library to use Stepper Motor
#include <std_msgs/Bool.h>

#define EN 8
const int stepPin = 2;      // X.STEP Motor's direction control. Motor moves one step by pulse sent to this pin
const int dirPin = 5;       // X.DIR Motor's Rotation Direction Control
const int degree_per_revolition = 200;

// Y Direction Actuator Connection
const int enA = 52;         // Motor's PWM Input Pin. Set the motor's speed by this pin's value
const int in1 = 50;         // Set Motor's Direction
const int in2 = 48;         // Set Motor's Direction
bool Z_Running = false;     // Variable to track if the motor is currently running

const int relay = 32;       // Pin to control Relay

unsigned long currentMicros = 0;
unsigned long previousMicros = 0;
unsigned long periodMicros = 0;
int pwmFrequency = 100;      // PWM frequency in Hz                 
int pwmDutyCycle = 255;      // PWM duty cycle (0-255) for speed control

ros::NodeHandle nh;

void goalReachedCallback(const std_msgs::Bool &msg){
  if (msg.data){
    unsigned long ZstartTime = millis();
    while (millis() - ZstartTime < 3000) {
      digitalWrite(dirPin, HIGH);   // Set Z Actuator to move Upward for 3 seconds
      Z_Running = true;             // Start Z Actuator
    }
    Z_Running = false;            // Stop Z Actuator

    unsigned long YstartTime = millis();
    while (millis() - YstartTime < 5000) {
      digitalWrite(in1, HIGH);  // Move Y Axis actuator
      digitalWrite(in2, LOW);
    }
    digitalWrite(in1, LOW);  // Stop Y-axis movement

    unsigned long RelaystartTime = millis();
    while (millis() - RelaystartTime < 5000) {
      digitalWrite(relay, HIGH);  // Turn on the relay
    }
    digitalWrite(relay, LOW);  // Turn off the relay

    // Y-axis movement backward using millis()
    YstartTime = millis();
    while (millis() - YstartTime < 5000) {
      digitalWrite(in1, LOW);   // Move Y Axis actuator backward
      digitalWrite(in2, HIGH);
    }
    digitalWrite(in1, LOW);  // Stop Y-axis movement
    digitalWrite(in2, LOW);
  }
}


ros::Subscriber<std_msgs::Bool> sub_goal_reached("goal_reached", &goalReachedCallback);

void setup() {
  nh.initNode();
  nh.subscribe(sub_goal_reached);

  // Set all pins to Output related to Z Actuator
  pinMode(stepPin, OUTPUT);
  pinMode(dirPin, OUTPUT);
  pinMode(EN, OUTPUT);
//  pinMode(sw, INPUT);
  digitalWrite(EN, LOW);

  // Set all pins to Output related to Y Actuator
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(enA, OUTPUT);
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);

  // Set Relay pin to Output
  pinMode(relay, OUTPUT);
}

void loop() {

  nh.spinOnce();
  delay(1);

  /* Set Z Actuator */
  if (Z_Running) {
    // 1.8 degrees per 1 revolution
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(800);         // Set Z Actuator's speed
    digitalWrite(stepPin, LOW);
    delayMicroseconds(800);
  }

 /* Set Y Actuator speed using software PWM */
  currentMicros = micros();
  periodMicros = 1000000 / pwmFrequency;

  if (currentMicros - previousMicros >= periodMicros) {
    previousMicros = currentMicros;
    analogWrite(enA, pwmDutyCycle);
  }
}