#include <I2Cdev.h>
#include <Servo.h>
#include <MPU6050_6Axis_MotionApps_V6_12.h>
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

Servo pitch_servo;
Servo roll_servo;
MPU6050 mpu;

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 gy;         // [x, y, z]            gyro sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// dummy target angle
int target_angle_pr[2] = {0, 0};
unsigned long current_time, previous_time;
float elapsed_time;
float error_pr[2], previous_error_pr[2];
float cum_error_pr[2], rate_error_pr[2];
float control_out_pr[2];
float max_control_out = 1;
float min_control_out = -1;

// Controller gains
double Kp = 0.035;
double Kd = 0;
double Ki = 0;

// range of roll and pitch outputs possible
float max_pr[2] = {20, 15};
float min_pr[2] = {-20, -15};
int pin_roll_servo = 20;
int pin_pitch_servo = 21;

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
  while (!Serial); // wait for Leonardo enumeration, others continue immediately

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

  // initialize device
  Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();
  
  // verify connection
  Serial.println(F("Testing device connections..."));
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

  // load and configure the DMP
  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();

    mpu.setXGyroOffset(51);
    mpu.setYGyroOffset(8);
    mpu.setZGyroOffset(21);
    mpu.setXAccelOffset(1150);
    mpu.setYAccelOffset(-50);
    mpu.setZAccelOffset(1060);
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
}

void loop(){
  if (!dmpReady) return;
  
  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
    get_gimbal_state();   
    compute_controller_output();
    write_to_servo();
  }
}

void write_to_servo(){
  float servo_cmd_pr[2];
  // mapping from (min -> max) to (0 -> 180)
  servo_cmd_pr[0] = 1000 + (((2000 - 1000) / (max_control_out - min_control_out)) * (control_out_pr[0] - min_control_out)); 
  servo_cmd_pr[1] = 1000 + (((2000 - 1000) / (max_control_out - min_control_out)) * (control_out_pr[1] - min_control_out));
  // pitch_servo.writeMicroseconds(servo_cmd_pr[0]);
  // roll_servo.writeMicroseconds(servo_cmd_pr[1]);
}

void compute_controller_output(){
  current_time = millis();
  elapsed_time = (double)(current_time - previous_time);

  error_pr[0] = target_angle_pr[0] - ypr[1];
  error_pr[1] = target_angle_pr[1] - ypr[2];

  cum_error_pr[0] += error_pr[0] * elapsed_time;
  cum_error_pr[1] += error_pr[1] * elapsed_time;

  rate_error_pr[0] = (error_pr[0] - previous_error_pr[0]) / elapsed_time;
  rate_error_pr[1] = (error_pr[1] - previous_error_pr[1]) / elapsed_time;

  control_out_pr[0] = Kp * error_pr[0] + Ki * cum_error_pr[0] + Kd * rate_error_pr[0];
  control_out_pr[1] = -Kp * error_pr[1] + Ki * cum_error_pr[1] + Kd * rate_error_pr[1];

  for (int i=0; i<2; i++ ){
    if (control_out_pr[i] > max_control_out){
      control_out_pr[i] = max_control_out;
      }
    else if (control_out_pr[i] < min_control_out){
      control_out_pr[i] = min_control_out;
      }
  }
    
  previous_error_pr[0] = error_pr[0];
  previous_error_pr[1] = error_pr[1];
  previous_time = current_time;
  Serial.print("Control output:: pitch_cmd: \t");
  Serial.print(control_out_pr[0]);
  Serial.print("\t \t roll_cmd: \t");
  Serial.println(control_out_pr[1]);

}


void get_gimbal_state(){
 
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    
    Serial.print("YPR: \t");
    ypr[1] = ypr[1] * -180 / M_PI;
    ypr[2] = ypr[2] * -180 / M_PI;
    Serial.print("pitch: \t");
    Serial.print(ypr[1]);
    Serial.print("\t roll: \t");
    Serial.println(ypr[2]);
  
}
  
