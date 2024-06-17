#include "MicroROSArduino.h"
#include <Wire.h>
#define ENCODER_OPTIMIZE_INTERRUPTS
#include <Encoder.h>
#include <PID_v2.h>
#include <Adafruit_INA219.h>

// define pin numbers
int Mot_L_EN = 4;
int Mot_L_PH = 5;
int Mot_R_EN = 3;
int Mot_R_PH = 2;
int Encode_L1 = 17;
int Encode_L2 = 16;
int Encode_R1 = 0;
int Encode_R2 = 1;

int left = 1;
int right = 2;

// define objects
MicroROSArduino micro_ros;
Adafruit_INA219 power_monitor;
Encoder encoder_left(Encode_L2, Encode_L1); 
Encoder encoder_right(Encode_R1, Encode_R2); 
// default sample time for the PID controller is 100ms
// can be changed with object.SetSampleTime(int NewSampleTime);
// change setpoint with object.Setpoint(double v);
double Kp = 5.0, Ki = 100.0, Kd = 0.0;
PID_v2 control_left(Kp, Ki, Kd, PID::Direct);
PID_v2 control_right(Kp, Ki, Kd, PID::Direct);

// battery variables
float battery_voltage = 0;
float battery_current = 0;

// controller variables
long encoder_l = 0;
long encoder_r = 0;
long previous_l = 0;
long previous_r = 0;
double input_l = 0.0;
double output_l = 0.0;
double input_r = 0.0;
double output_r = 0.0;

// The documentation says the motor encoders count 44 edges per revolution
// The gear ratio is 108:1, 108*44 = 4752
// The wheels are 6 inch diameter
float scale_angle_pos = 108*44/(2*3.14159); // step per radian
float scale_steps_pos = 1.0/scale_angle_pos;  // radian per step
// The loop for measuring velocity counts for 1/10 second
float scale_angle_vel = scale_angle_pos*0.1;
float scale_steps_vel = 1.0/scale_angle_vel;

void set_PWM(int motor, int value) {
  if (motor == left) {
    if (value < 0) {
      digitalWrite(Mot_L_PH, LOW);  // left backward
      analogWrite(Mot_L_EN, -value);
    } else {
      digitalWrite(Mot_L_PH, HIGH);  // left forward
      analogWrite(Mot_L_EN, value);
    }
  } else {
    if (value < 0) {
      digitalWrite(Mot_R_PH, HIGH);  // right backward
      analogWrite(Mot_R_EN, -value);
    } else {
      digitalWrite(Mot_R_PH, LOW);  // right forward
      analogWrite(Mot_R_EN, value);
    }
  }
}

void battery_timer_cb(rcl_timer_t * timer, int64_t last_call_time) {
  battery_voltage = power_monitor.getBusVoltage_V() + (power_monitor.getShuntVoltage_mV() / 1000);
  battery_current = -1000*power_monitor.getCurrent_mA();
  micro_ros.battery_msg.voltage = battery_voltage;
  micro_ros.battery_msg.voltage = battery_current;
  micro_ros.publishBattery();
}

void imu_timer_cb(rcl_timer_t * timer, int64_t last_call_time) {
  micro_ros.imu_msg.orientation.w = 0.707;
  micro_ros.imu_msg.orientation.z = 0.707;
  micro_ros.publishImu();
}

void joint_state_timer_cb(rcl_timer_t * timer, int64_t last_call_time) {
  previous_l = encoder_l;
  encoder_l = encoder_left.read();
  previous_r = encoder_r;
  encoder_r = encoder_right.read();
  input_l = scale_steps_vel*(encoder_l - previous_l); // radians per second
  input_r = scale_steps_vel*(encoder_r - previous_r); // radians per second
  micro_ros.joint_state_msg.position.data[0] = scale_steps_pos*encoder_l;
  micro_ros.joint_state_msg.velocity.data[0] = input_l;
  micro_ros.joint_state_msg.position.data[1] = scale_steps_pos*encoder_r;
  micro_ros.joint_state_msg.velocity.data[1] = input_r;
  micro_ros.publishJointState();
  digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));  
}

void commander_cb(const void * msgin)
{  
  const sensor_msgs__msg__JointState * cmd_msg = (const sensor_msgs__msg__JointState *)msgin;
  control_left.Setpoint(cmd_msg->velocity.data[0]);
  control_right.Setpoint(cmd_msg->velocity.data[1]);
  digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));  
}

void setup() {
  Wire.begin();

  // setup micro ros
  micro_ros.beginBatteryBroadcaster(&battery_timer_cb);
  micro_ros.beginImuBroadcaster(&imu_timer_cb);
  micro_ros.beginJointStateBroadcaster(&joint_state_timer_cb);
  micro_ros.beginJointStateCommander(&commander_cb);

  // setup battery
  power_monitor.begin();

  // setup motors and controllers
  analogWrite(Mot_L_EN, 0); // left speed pin
  pinMode(Mot_L_PH, OUTPUT);  // left direction pin
  digitalWrite(Mot_L_PH, HIGH);  // left forward
  analogWrite(Mot_R_EN, 0); // right speed pin
  pinMode(Mot_R_PH, OUTPUT);  //right direction pin
  digitalWrite(Mot_R_PH, LOW);  // right forward
  encoder_left.write(0);
  encoder_right.write(0);
  // Clamps the output to a specific range. 0-255 by default
  control_left.SetOutputLimits(-255, 255);
  control_right.SetOutputLimits(-255, 255);
  // object.Start(input, current output, setpoint);
  control_left.Start(0, 0, 0);
  control_right.Start(0, 0, 0);

  control_left.Setpoint(3.0);
  control_right.Setpoint(-6.5);

}

void loop() {
  // keep ROS callbacks running
  micro_ros.spin();

  // keep motors running
  output_l = control_left.Run(input_l);
  set_PWM(left, output_l);
  output_r = control_right.Run(input_r);
  set_PWM(right, output_r);
}
