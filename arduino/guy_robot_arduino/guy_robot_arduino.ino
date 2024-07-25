/*
  guy robot experiment 7/20/2024
  can it drive!
*/

#include "MicroROSArduino.h"
#include <Adafruit_INA219.h>
#include <Adafruit_BNO055.h>
#include <EEPROM.h>
#include <PID_v2.h>
#define ENCODER_OPTIMIZE_INTERRUPTS
#include <Encoder.h>

// GPIO pin definitions
const int Mot_L_EN = 4;
const int Mot_L_PH = 5;
const int Mot_R_EN = 3;
const int Mot_R_PH = 2;
const int Encode_L1 = 17;
const int Encode_L2 = 16;
const int Encode_R1 = 0;
const int Encode_R2 = 1;
const int Step1_Or = 20;
const int Step1_Ye = 6;
const int Step1_Pi = 12;
const int Step1_Bl = 11;
const int Step2_Or = 7;
const int Step2_Ye = 8;
const int Step2_Pi = 9;
const int Step2_Bl = 10;
const int LED_GRN1 = 14; // SOC LEDs
const int LED_RED1 = 15;
const int LED_GRN2 = 22; // Current LEDs
const int LED_RED2 = 23;
const int Teensy_LED = 13;
const int Push_But = 21;

const int LEFT = 1;
const int RIGHT = 2;

// create a micro ros object
MicroROSArduino micro_ros;

// define battery variables and objects
uint8_t battery_id;
Adafruit_INA219 power_monitor;

// define imu variables and objects
uint8_t imu_id;
int eeAddress = 0;
long bnoID;
adafruit_bno055_offsets_t calibrationData;
sensor_t sensor;
uint8_t system_cal, gyro_cal, accel_cal, mag_cal = 0;
imu::Quaternion quat;
imu::Vector<3> gyro;
imu::Vector<3> accel;
imu::Vector<3> magnet;
Adafruit_BNO055 imu_sensor = Adafruit_BNO055(55);  

// define the encoder variables and objects
uint8_t joint_states_id;
long encoder_l = 0;
long encoder_r = 0;
long previous_l = 0;
long previous_r = 0;
Encoder encoder_left(Encode_L2, Encode_L1); 
Encoder encoder_right(Encode_R1, Encode_R2); 

// define controller variables and objects
// default sample time for the PID controller is 100ms
// can be changed with object.SetSampleTime(int NewSampleTime);
// change setpoint with object.Setpoint(double v);
double input_l = 0.0;
double output_l = 0.0;
double input_r = 0.0;
double output_r = 0.0;
double Kp = 5.0, Ki = 100.0, Kd = 0.0;
PID_v2 control_left(Kp, Ki, Kd, PID::Direct);
PID_v2 control_right(Kp, Ki, Kd, PID::Direct);

// define scaling variables
// The documentation says the motor encoders count 44 edges per revolution
// The gear ratio is 108:1, 108*44 = 4752
// The wheels are 6 inch diameter
const float scale_counts_pos = 108*44/(2*3.1415926536); // encoder counts per radian
const float scale_radians_pos = 1.0/scale_counts_pos;  // radians per encoder count
// The loop for measuring velocity counts for 1/10 second
const float scale_counts_vel = scale_counts_pos*0.1;
const float scale_radians_vel = 1.0/scale_counts_vel;
const float scale_deg_rad = 3.1415926536/180.0;

void battery_timer_cb(rcl_timer_t * timer, int64_t last_call_time) {
  micro_ros.battery_msg[battery_id].voltage = power_monitor.getBusVoltage_V() + (power_monitor.getShuntVoltage_mV()/1000.0);
  micro_ros.battery_msg[battery_id].current = power_monitor.getCurrent_mA()/-1000.0;
  micro_ros.publishBattery(battery_id);
}

void imu_timer_cb(rcl_timer_t * timer, int64_t last_call_time) {
  imu_sensor.getCalibration(&system_cal, &gyro_cal, &accel_cal, &mag_cal);
  if (gyro_cal==3 && accel_cal==3 && mag_cal==3) {
    if (system_cal==3) {
      analogWrite(LED_GRN2, 255); analogWrite(LED_RED2, 0);
    } else {
      analogWrite(LED_GRN2, 190); analogWrite(LED_RED2, 65);
    }
  } else {
    analogWrite(LED_GRN2, 0); analogWrite(LED_RED2, 255);
  }
  quat = imu_sensor.getQuat();
  gyro = imu_sensor.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
  accel = imu_sensor.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
  micro_ros.imu_msg[imu_id].orientation.w = quat.w();
  micro_ros.imu_msg[imu_id].orientation.x = quat.x();
  micro_ros.imu_msg[imu_id].orientation.y = quat.y();
  micro_ros.imu_msg[imu_id].orientation.z = quat.z();
  micro_ros.imu_msg[imu_id].angular_velocity.x = scale_deg_rad*gyro.x();
  micro_ros.imu_msg[imu_id].angular_velocity.y = scale_deg_rad*gyro.y();
  micro_ros.imu_msg[imu_id].angular_velocity.z = scale_deg_rad*gyro.z();
  micro_ros.imu_msg[imu_id].linear_acceleration.x = accel.x();
  micro_ros.imu_msg[imu_id].linear_acceleration.y = accel.y();
  micro_ros.imu_msg[imu_id].linear_acceleration.z = accel.z();
  micro_ros.publishImu(imu_id);
}

void joint_state_timer_cb(rcl_timer_t * timer, int64_t last_call_time) {
  previous_l = encoder_l;
  encoder_l = encoder_left.read();
  previous_r = encoder_r;
  encoder_r = encoder_right.read();
  input_l = scale_radians_vel*(encoder_l - previous_l); // radians per second
  input_r = scale_radians_vel*(encoder_r - previous_r); // radians per second
  micro_ros.joint_state_msg.position.data[0] = scale_radians_pos*encoder_l;
  micro_ros.joint_state_msg.velocity.data[0] = input_l;
  micro_ros.joint_state_msg.position.data[1] = scale_radians_pos*encoder_r;
  micro_ros.joint_state_msg.velocity.data[1] = input_r;
  micro_ros.publishJointStateBroad(joint_states_id);
}

void commander_cb(const void * msgin)
{  
  const sensor_msgs__msg__JointState * cmd_msg = (const sensor_msgs__msg__JointState *)msgin;
  control_left.Setpoint(cmd_msg->velocity.data[0]);
  control_right.Setpoint(cmd_msg->velocity.data[1]);
  digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));  
}

void set_PWM(int motor, int value) {
  if (motor == LEFT) {
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

void setup() {
  Wire.begin();
  
  // set up LEDs and pushbutton
  analogWrite(LED_RED1, 0); // red SOC LED
  analogWrite(LED_GRN1, 0); // green SOC LED
  analogWrite(LED_RED2, 0); // red current LED
  analogWrite(LED_GRN2, 0); // green current LED
  pinMode(Teensy_LED, OUTPUT); // Teensy LED
  digitalWrite(Teensy_LED, LOW); 
  pinMode(Push_But, INPUT_PULLDOWN); // Pushbutton

  // set up power monitor
  battery_id = micro_ros.beginBroadcaster(MicroROSArduino::BATTERY, "battery", 1.0, &battery_timer_cb);
  rosidl_runtime_c__String__assignn(&micro_ros.battery_msg[battery_id].header.frame_id, "bat_lead", 8);
  micro_ros.battery_msg[battery_id].present = true;
  micro_ros.battery_msg[battery_id].power_supply_technology = sensor_msgs__msg__BatteryState__POWER_SUPPLY_TECHNOLOGY_LIFE;
  micro_ros.battery_msg[battery_id].power_supply_health = sensor_msgs__msg__BatteryState__POWER_SUPPLY_HEALTH_GOOD;
  micro_ros.battery_msg[battery_id].design_capacity = 6.0;  //Ah
  power_monitor.begin();

  // set up IMU
  imu_id = micro_ros.beginBroadcaster(MicroROSArduino::IMU, "imu", 20.0, &imu_timer_cb);
  rosidl_runtime_c__String__assignn(&micro_ros.imu_msg[imu_id].header.frame_id, "imu_link", 8);
  micro_ros.imu_msg[imu_id].orientation_covariance[0] = 0.01;
  micro_ros.imu_msg[imu_id].orientation_covariance[4] = 0.01;
  micro_ros.imu_msg[imu_id].orientation_covariance[8] = 0.01;
  micro_ros.imu_msg[imu_id].angular_velocity_covariance[0] = 0.00005;
  micro_ros.imu_msg[imu_id].angular_velocity_covariance[4] = 0.00005;
  micro_ros.imu_msg[imu_id].angular_velocity_covariance[8] = 0.00005;
  micro_ros.imu_msg[imu_id].linear_acceleration_covariance[0] = 0.002;
  micro_ros.imu_msg[imu_id].linear_acceleration_covariance[4] = 0.002;
  micro_ros.imu_msg[imu_id].linear_acceleration_covariance[8] = 0.002;
  imu_sensor.begin(); 
  EEPROM.get(eeAddress, bnoID); // Check for calibration data in Teensy's EEPROM
  imu_sensor.getSensor(&sensor);
  if (bnoID == sensor.sensor_id) {
    eeAddress += sizeof(long);
    EEPROM.get(eeAddress, calibrationData);
    imu_sensor.setSensorOffsets(calibrationData);
	// Flash Red Green to indicate calib data found
    analogWrite(LED_RED1, 255);
    analogWrite(LED_GRN2, 255);
  }
  delay(100);  // Example sketch used a delay here
  imu_sensor.setExtCrystalUse(true);

  // set up encoders and broadcaster
  String JointNames[2];
  JointNames[0] = "gear_left_shaft";
  JointNames[1] = "gear_right_shaft";
  joint_states_id = micro_ros.beginBroadcaster(MicroROSArduino::JOINTSTATEBROAD, "joint_states", 10.0, &joint_state_timer_cb, 2, JointNames);
  encoder_left.write(0);
  encoder_right.write(0);
  
  // set up PID controller, motors, and commander
  micro_ros.beginJointStateCommander(&commander_cb, "commands", 4, JointNames);
  analogWrite(Mot_L_EN, 0); // left speed pin
  pinMode(Mot_L_PH, OUTPUT);  // left direction pin
  digitalWrite(Mot_L_PH, HIGH);  // left forward
  analogWrite(Mot_R_EN, 0); // right speed pin
  pinMode(Mot_R_PH, OUTPUT);  //right direction pin
  digitalWrite(Mot_R_PH, LOW);  // right forward
  // Clamps the output to a specific range. 0-255 by default
  control_left.SetOutputLimits(-255, 255);
  control_right.SetOutputLimits(-255, 255);
  // object.Start(input, current output, setpoint);
  control_left.Start(0, 0, 0);
  control_right.Start(0, 0, 0);
}

void loop() {
  // keep ROS callbacks running
  micro_ros.spin();

  // keep wheels spinning
  output_l = control_left.Run(input_l);
  set_PWM(LEFT, output_l);
  output_r = control_right.Run(input_r);
  set_PWM(RIGHT, output_r);
}
