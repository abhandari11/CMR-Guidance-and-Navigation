/* Get tilt angles on X and Y, and rotation angle on Z
 * Angles are given in degrees
 * 
 * License: MIT
 */

#include "Wire.h"
#include <MPU6050_light.h>

MPU6050 mpu(Wire);
unsigned long timer = 0;

class Iir {
public:
  void Init(float cutoff_hz, float samp_hz, float initial_val){
    float fc = cutoff_hz / samp_hz;
    float c_twoPI_fc = cos(2.0 * PI * fc);
    b_ = 2.0 - c_twoPI_fc - sqrt(pow(2.0 - c_twoPI_fc, 2.0) - 1.0);
    a_ = 1.0 - b_;
    prev_output_ = initial_val;
  }
  float Filter (float val){
    float ret;
    ret = a_ * val + b_ * prev_output_;
    prev_output_return_ = prev_output_;
    prev_output_ = ret;
    return ret;
  }
  float prev_val(){
    return prev_output_return_;
  }
private:
  float a_, b_, prev_output_, prev_output_return_;
}; 

Iir xpos_filter, ypos_filter, zpos_filter;
xpos_filter.Init(1.5, 4.0, x_pos_mm.i4);
ypos_filter.Init(1.5, 4.0, y_pos_mm.i4);
zpos_filter.Init(1.5, 4.0, z_pos_mm.i4);


void setup() {
  Serial.begin(9600);
  Wire.begin();
  
  byte status = mpu.begin();
  Serial.print(F("MPU6050 status: "));
  Serial.println(status);
  while(status!=0){ } // stop everything if could not connect to MPU6050
  
  Serial.println(F("Calculating offsets, do not move MPU6050"));
  delay(1000);
  // mpu.upsideDownMounting = true; // uncomment this line if the MPU6050 is mounted upside-down
  mpu.calcOffsets(); // gyro and accelero
  Serial.println("Done!\n");
}

void loop() {
  mpu.update();
  
  if((millis()-timer)>10){ // print data every 10ms
  float x = mpu.getAngleX();
  Serial.println(x);
  float filtered_x = xpos_filter.Filter(x_pos_mm.i4);
  Serial.println(filtered_x);
//  
//  Serial.print("X : ");
//  Serial.print(mpu.getAngleX());
//  Serial.print("\tY : ");
//  Serial.print(mpu.getAngleY());
//  Serial.print("\tZ : ");
//  Serial.println(mpu.getAngleZ());
  timer = millis();  
  }
}
