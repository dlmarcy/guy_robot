/*
  test_BNO055
*/

#include <Adafruit_BNO055.h>
#include <EEPROM.h>

const int LED_GRN2 = 22; // Current LEDs
const int LED_RED2 = 23;
const int Push_But = 21;

int eeAddress = 0;
long bnoID;
bool foundCalib = false;
adafruit_bno055_offsets_t calibrationData;
sensor_t sensor;
uint8_t system_cal, gyro_cal, accel_cal, mag_cal = 0;
imu::Quaternion quat;
imu::Vector<3> gyro;
imu::Vector<3> accel;
imu::Vector<3> magnet;

Adafruit_BNO055 imu_sensor = Adafruit_BNO055(55);  

void setup() {
  Wire.begin();
  pinMode(Push_But, INPUT_PULLDOWN); // Pushbutton
  imu_sensor.begin(); 
  EEPROM.get(eeAddress, bnoID); // Check for calibration data in Teensy's EEPROM
  imu_sensor.getSensor(&sensor);  // Get the BNO sensor ID and compare to that stored in the EEPROM (check if same sensor)
  if (bnoID == sensor.sensor_id) {
    // Serial.println("\nFound Calibration for this sensor in EEPROM.");
    eeAddress += sizeof(long);
    EEPROM.get(eeAddress, calibrationData);
    // Serial.println("\n\nRestoring Calibration data to the BNO055...");
    imu_sensor.setSensorOffsets(calibrationData);
    // Serial.println("\n\nCalibration data loaded into BNO055");
    foundCalib = true;
  }
  delay(100);  // Example sketch used a delay here
  imu_sensor.setExtCrystalUse(true);
}

void loop() {
  // Test the IMU
  quat = imu_sensor.getQuat();
  gyro = imu_sensor.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
  accel = imu_sensor.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
  magnet = imu_sensor.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);
  imu_sensor.getCalibration(&system_cal, &gyro_cal, &accel_cal, &mag_cal);
  Serial.println("Calibration:");
  Serial.print("System = ");
  Serial.print(system_cal);
  Serial.print(" Gyro = ");
  Serial.print(gyro_cal);
  Serial.print(" Accel = ");
  Serial.print(accel_cal);
  Serial.print(" Mag = ");
  Serial.println(mag_cal);
  Serial.println("Quaternion:");
  Serial.print("w = ");
  Serial.print(quat.w());
  Serial.print(" x = ");
  Serial.print(quat.x());
  Serial.print(" y = ");
  Serial.print(quat.y());
  Serial.print(" z = ");
  Serial.println(quat.z());
  Serial.println("Gyroscope [radian/second]:");
  Serial.print("x = ");
  Serial.print(gyro.x());
  Serial.print(" y = ");
  Serial.print(gyro.y());
  Serial.print(" z = ");
  Serial.println(gyro.z());
  Serial.println("Acceleration [m/s^2]:");
  Serial.print("x = ");
  Serial.print(accel.x());
  Serial.print(" y = ");
  Serial.print(accel.y());
  Serial.print(" z = ");
  Serial.println(accel.z());
  Serial.println("Magnetic Field [uT]:");
  Serial.print("x = ");
  Serial.print(magnet.x());
  Serial.print(" y = ");
  Serial.print(magnet.y());
  Serial.print(" z = ");
  Serial.println(magnet.z());
  Serial.println();
  delay(2000);

  imu_sensor.getCalibration(&system_cal, &gyro_cal, &accel_cal, &mag_cal);
  if (gyro_cal==3 && accel_cal==3) {  // publish if gyro and accel are calibrated
    quat = imu_sensor.getQuat();
    gyro = imu_sensor.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
    accel = imu_sensor.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
    if (mag_cal==3) { 
      if (system_cal==3) {
        analogWrite(LED_GRN2, 255); analogWrite(LED_RED2, 0); // all calibrated
      } else {
        analogWrite(LED_GRN2, 210); analogWrite(LED_RED2, 45); // everything but system
      }
    } else {
      analogWrite(LED_GRN2, 127); analogWrite(LED_RED2, 128); // only gyro and accel calib
    }
  } else {
    analogWrite(LED_GRN2, 0); analogWrite(LED_RED2, 255); // not enough calibration
  }
  delay(1000);

  // check if push button is pressed or not
  if (digitalRead(Push_But) == HIGH) {  
    imu_sensor.getCalibration(&system_cal, &gyro_cal, &accel_cal, &mag_cal);
    if (system_cal==3 && gyro_cal==3 && accel_cal==3 && mag_cal==3) {
      analogWrite(LED_GRN2, 0); analogWrite(LED_RED2, 0);
      // getSensorOffsets only works when all cal are 3
      imu_sensor.getSensorOffsets(calibrationData);
      eeAddress = 0;
      imu_sensor.getSensor(&sensor);
      bnoID = sensor.sensor_id;
      EEPROM.put(eeAddress, bnoID);
      eeAddress += sizeof(long);
      EEPROM.put(eeAddress, calibrationData);
    }
  }
  delay(1000);

}
