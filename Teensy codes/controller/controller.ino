#include <I2Cdev.h>
#include <Servo.h>
#include <MPU6050_6Axis_MotionApps20.h>

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  #include "Wire.h"
#endif

const bool _DEBUG = true;

Servo pitch_servo;
Servo roll_servo;
MPU6050 mpu;

const float r2d = 180 / M_PI;
const float d2r = 1 / r2d;

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 gy;         // [x, y, z]            gyro sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
float euler[3];
float offset_pr[2];

int pin_roll_servo = 20;
int pin_pitch_servo = 21;

// update rate: 10 m/s
const int16_t LOOP_PERIOD_MS = 10;

// making separate variables for euler angles
float pitch; float roll;

// dummy target pointing vector
// a vector pointing from center of gimbal to the survey point on ground. 
// NAV Frame: For Front Gimbal of the vehicle
float target_point_vector[3] = {64, -100, 175};
// corresponding target angle should be roll: 30 deg, pitch angle: 20 deg

// dummy target angle
float target_angle_pr[2];
float output_angle_rate[2];

unsigned long current_time, previous_time;
unsigned long elapsed_time;
float error_pr[2], previous_error_pr[2];
float cum_error_pr[2], rate_error_pr[2];
float control_out_pr[2];

// Controller gains
float Kp = 28.0; // 24.0;
float Kd = 0.0;
float Ki = 4.0; // 4.0;

// range of Servo movements
float max_pr[2] = {45, 75};
float min_pr[2] = {-45, -75};
float max_angular_rate = 350;  // 

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer


void setup() {

  // initialize serial communication
  Serial.begin(38400);
  while (!Serial); 

  // setting the servos to neutral point
  pitch_servo.attach(pin_pitch_servo);
  roll_servo.attach(pin_roll_servo);
  Serial.println("Centering the servos...");
  delay(2000);
  Serial.println("-----------------------");
  pitch_servo.write(90);
  roll_servo.write(90);
  
  // setting up I2C connection to IMU
  Wire.begin();
  Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties

  // wait for ready:: Needs keyboard entry
  if (_DEBUG) {
    Serial.println(F("\nSend any character to begin the loop: "));
    while (Serial.available() && Serial.read()); // empty buffer
    while (!Serial.available());                 // wait for data
    while (Serial.available() && Serial.read()); // empty buffer again
  } 
  else {
    // For real case, wait for 2 sec. 
    delay(2000);
  }
  
  // initialize device
  Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();
  
  // verify connection
  Serial.println(F("Testing device connections..."));
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

  // load and configure the DMP
  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();
  
  // Provided offsets
  mpu.setXGyroOffset(220);
  mpu.setYGyroOffset(76);
  mpu.setZGyroOffset(-85);
  mpu.setZAccelOffset(1780);
  Serial.println();
  Serial.print("Offsets before calibration...\n");
  mpu.PrintActiveOffsets();

  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
    // Calibration Time: generate offsets and calibrate our MPU6050
    mpu.CalibrateAccel(6);
    mpu.CalibrateGyro(6);
    Serial.println();
    mpu.PrintActiveOffsets();
    // turn on the DMP, now that it's ready
    Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);
    dmpReady = true;
  } 

  else {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
  } 

  // Calculating some existing bias on final euler angles
  Serial.println("Calculating offsets for Euler angles...");
  calculate_offsets_pr();
  Serial.print("Pitch offset is ");
  Serial.println(offset_pr[0]);
  Serial.print("Roll offset is ");
  Serial.println(offset_pr[1]);

  // wait for ready:: Needs keyboard entry
  if (_DEBUG) {
    Serial.println(F("\nSend any character to begin the loop: "));
    while (Serial.available() && Serial.read()); // empty buffer
    while (!Serial.available());                 // wait for data
    while (Serial.available() && Serial.read()); // empty buffer again
  }
  else {
    // For real case, wait for 2 sec. 
    delay(2000);
  }

  // dummy previous_time variable so that first iteration doesnt blow up
  previous_time = millis();
}

void loop(){ 
  if (!dmpReady) return;
  
  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
    unsigned long loop_start_time_ms = millis();
    
    // calculates target euler angles from target point's position vector
    calculate_target_angle_pr();

    // get current euler angles
    get_gimbal_state();   

    // Angular rate controller
    compute_controller_output();

    // Commanded angle output to the servos
    write_to_servo();

    unsigned long loop_frame_time_ms = millis() - loop_start_time_ms;
    // keeping the loop run time constant
    delay(LOOP_PERIOD_MS - loop_frame_time_ms);
  }
}

void calculate_offsets_pr() {
  float pr_vals[2] = {0, 0};
  int sample_num = 10;
  // taking simple average of sample_num measurements
  for (int i = 0; i < sample_num; i++) {
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    pr_vals[0] += (ypr[1] * -r2d);
    pr_vals[1] += (ypr[2] * -r2d);
    delay(100);
  }
  offset_pr[0] = pr_vals[0] / sample_num;
  offset_pr[1] = pr_vals[1] / sample_num;
}

void calculate_target_angle_pr() {
  // target pitch
  target_angle_pr[0] = r2d * atan2(target_point_vector[0], target_point_vector[2]);
  // target roll
  // negative sign based on the convention
  target_angle_pr[1] = - r2d * atan2(target_point_vector[1], target_point_vector[2]);

  Serial.print("Target angle:: pr\t");
  Serial.print(target_angle_pr[0]);
  Serial.print("\t");
  Serial.println(target_angle_pr[1]);
}

void get_gimbal_state(){
  // Based on the MPU_6050 library example
  // Getting Euler Angles
  mpu.dmpGetQuaternion(&q, fifoBuffer);
  mpu.dmpGetGravity(&gravity, &q);
  mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

  pitch = (ypr[1] * -r2d) - offset_pr[0];
  roll = (ypr[2] * -r2d) - offset_pr[1];

  Serial.print("ypr\t");
  Serial.print(pitch);
  Serial.print("\t");
  Serial.println(roll);

}

void compute_controller_output(){
  current_time = millis();
  elapsed_time = (unsigned long)(current_time - previous_time);

  // Calculating error from feedback
  error_pr[0] = target_angle_pr[0] - pitch;
  error_pr[1] = target_angle_pr[1] - roll;
  
  Serial.print("Error_pr:: pr\t");
  Serial.print(error_pr[0]);
  Serial.print("\t");
  Serial.println(error_pr[1]);

  // Cumulative error
  cum_error_pr[0] += error_pr[0] * elapsed_time / 1000;
  cum_error_pr[1] += error_pr[1] * elapsed_time / 1000;

  // rate error
  rate_error_pr[0] = (error_pr[0] - previous_error_pr[0]) * 1000 / elapsed_time;
  rate_error_pr[1] = (error_pr[1] - previous_error_pr[1]) * 1000 / elapsed_time;

  // PID output
  output_angle_rate[0] = Kp * error_pr[0] + Ki * cum_error_pr[0] + Kd * rate_error_pr[0];
  output_angle_rate[1] = Kp * error_pr[1] + Ki * cum_error_pr[1] + Kd * rate_error_pr[1];

  // saturating the angular rate
  for (int i=0; i<2; i++ ){
    if (abs(output_angle_rate[i]) > max_angular_rate) {
      if (output_angle_rate[i] < 0) {
        output_angle_rate[i] = -max_angular_rate;
      }
      else {
        output_angle_rate[i] = max_angular_rate;
      }
    }
  }

  Serial.print("Output angular rate:: pitch_rate: \t");
  Serial.print(output_angle_rate[0]);
  Serial.print("\t \t roll_rate: \t");
  Serial.println(output_angle_rate[1]);

  previous_error_pr[0] = error_pr[0];
  previous_error_pr[1] = error_pr[1];
  previous_time = current_time;
}


void write_to_servo(){
  float servo_cmd_pr[2];

  // Integrating the angular rate for commanded angle
  control_out_pr[0] += output_angle_rate[0] * elapsed_time / 1000; 
  control_out_pr[1] += output_angle_rate[1] * elapsed_time / 1000;

  // saturating commanded angle based on servo limits
  for (int i=0; i<2; i++) {
    if (control_out_pr[i] > max_pr[i]) {
      control_out_pr[i] = max_pr[i];
    }
    else if (control_out_pr[i] < min_pr[i]) {
      control_out_pr[i] = min_pr[i];
    }
  }

  // final commanded angle. 90deg is center for servo. 
  servo_cmd_pr[0] = 90 + control_out_pr[0];
  servo_cmd_pr[1] = 90 + control_out_pr[1];

  Serial.print("Output angles:: pitch: \t");
  Serial.print(control_out_pr[0]);
  Serial.print("\t \t roll: \t");
  Serial.println(control_out_pr[1]);

  // sending angle command to servo
  pitch_servo.write(servo_cmd_pr[0]);
  roll_servo.write(servo_cmd_pr[1]);
}
  
