#include <MPU9250.h>

#ifndef _IMU_H_
#define _IMU_H_

MPU9250 mpu;
float imu_offsets[4] = {0};

float pitch = 0;
float roll = 0;
float prev_pitch = 0;
float sum_pitch = 0;
float prev_roll = 0;
float sum_roll = 0;

float pitch_P = 0.007;
float pitch_I =  0;
float pitch_D = 0.005;

float roll_P = 0.007;
float roll_I = 0;
float roll_D = .005;

void runIMU(bool is_trot) {
 // Serial.println("called runimu");
  if (mpu.update()) { 
    pitch = mpu.getPitch();
    roll = mpu.getRoll();
    
    sum_pitch += pitch;
    sum_roll += roll;

    float pitch_err = pitch - prev_pitch;
    float roll_err = roll - prev_roll;
    
    float pitch_val;
    float roll_val;
    
    if (abs(pitch) < 5) { 
      pitch_val = 0;
    } else {
      pitch_val = pitch_P*pitch + pitch_I*sum_pitch + pitch_D*pitch_err;
    } if (abs(roll) < 5) {
      roll_val = 0;
    } else {
      roll_val = roll_P*roll + roll_I*sum_roll + roll_D*roll_err; 
   }
    
    Serial.println("PID Values: " + String(pitch_val) + ", " + String(roll_val));

    float fl = -pitch_val + roll_val;
    float bl = -pitch_val - roll_val;
    float fr = pitch_val + roll_val;
    float br = pitch_val - roll_val;

    if (abs(fl) < 0.1) { 
      fl = 0;
    } if (abs(bl) < 0.1) {
      bl = 0;
    } if (abs(fr) < 0.1) {
      fr = 0;
    } if (abs(br) < 0.1) {
      br = 0;
    }

    
    prev_pitch = pitch;
    prev_roll = roll;

    Serial.println("FL: " + String(fl));
    Serial.println("BL: " + String(bl));
    Serial.println("FR: " + String(fr));
    Serial.println("BR: " + String(br));
   
   imu_offsets[0] += fl;
   
   imu_offsets[1] += bl;

   imu_offsets[2] += fr;
   
   imu_offsets[3] += br;

   int limit = 65; 
   if (is_trot)  
    limit = 10; 

   for (int i = 0; i < 4; i++) {
    if (imu_offsets[i] > limit ) 
      imu_offsets[i] = limit;
    else if (imu_offsets[i] < -limit)
      imu_offsets[i] = -limit;
   }

    Serial.println("fl offset: " + String(imu_offsets[0]));
    Serial.println("bl offset: " + String(imu_offsets[1]));
    Serial.println("fr offset: " + String(imu_offsets[2]));
    Serial.println("br offset: " + String(imu_offsets[3]));
  }
}

#endif


 
