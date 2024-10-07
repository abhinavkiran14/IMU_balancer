#include "MPU9250.h"
#include <Servo.h>

Servo pitch; //hip
Servo roll; //knee
MPU9250 mpu;
float pitchDegree = 90.0;
float rollDegree = 90.0;
float pitch_incr;
float roll_incr;

void setup() {
    Serial.begin(500000);
    Wire.begin();
    if (!mpu.setup(0x68)) {  // change to your own address
        while (1) {
            Serial.println("MPU connection failed. Please check your connection with `connection_check` example.");
            delay(5000);
        }
    }
    pitch.attach(6); //top servo
    roll.attach(5);  //bottom servo
    pitch.write(90);
    roll.write(90);
    delay(3000);
}

float Ico = 25;
float Pco = .05;
float Dco = 10l;
float pI = 0;
float rI = 0;
float rD = 0;
float pD = 0;
int msDelay = 20; //how often the loop runs
float prevRoll = 0;
float prevPitch = 0;
void loop() {
    if (mpu.update()) {
        static uint32_t prev_ms = millis();
        if (millis() > prev_ms + msDelay) {
            if (pitchDegree > 170){
              pitchDegree = 170;
            }
            else if (pitchDegree < 10){
              pitchDegree = 10;
            }

            if (rollDegree > 170){
              rollDegree = 170;
            }
            else if (rollDegree < 10){
              rollDegree = 10;
            }


            pI += mpu.getPitch()/(msDelay*1000);
            rI += mpu.getRoll()/(msDelay*1000);

            pD = (mpu.getPitch() - prevPitch)/(msDelay*1000);
            rD = (mpu.getPitch() - prevPitch)/(msDelay*1000);

            
            //pitch_incr = abs(mpu.getPitch()/10);
            //roll_incr = abs(mpu.getRoll()/10);

            pitch_incr = mpu.getPitch();
            roll_incr = mpu.getRoll();
         
            
            prev_ms = millis();
            prevPitch = mpu.getPitch();
            prevRoll = mpu.getRoll();


              
            if (mpu.getPitch() < -5){       // bottom servo
              pitchDegree += pitch_incr;
              pitch.write(pitchDegree);
            }
            else if (mpu.getPitch() > 5){
              pitchDegree -= pitch_incr;
              pitch.write(pitchDegree);
            }
             if (mpu.getRoll() < -5)   {
              rollDegree -= roll_incr;
              roll.write(rollDegree);
            }
            else if(mpu.getRoll() > 5) {    //top servo
              rollDegree += roll_incr;
              roll.write(rollDegree);
            } //comment out till here
      

            rollDegree += Pco*roll_incr+Ico*rI+Dco*rD;
//            rollDegree += roll_incr;
//            if(abs(mpu.getRoll() - rollDegree) < 5){
//              roll.write(rollDegree);
//            }
            if(abs(mpu.getRoll()) > 3){
              roll.write(rollDegree);
            }
            
            pitchDegree -= Pco*pitch_incr+Ico*pI+Dco*pD;
            if(abs(mpu.getPitch()) > 3){
              pitch.write(pitchDegree);
            }
//            

            if (prevPitch == 0){
              pI = 0;
            }
            if (prevRoll == 0){
              rI = 0;
            }

            if (mpu.getPitch() / prevPitch < 0){
              pI = 0;
            }
            if (mpu.getRoll() / prevRoll < 0){
              rI = 0;
            }
        }
    }  3
    
    print_roll_pitch_yaw();
}


    void print_roll_pitch_yaw() {
    Serial.print("Yaw, Pitch, Roll: ");
    Serial.print(mpu.getYaw(), 2);
    Serial.print(":");
    Serial.print(mpu.getPitch(), 2);
    Serial.print(":");
    Serial.println(mpu.getRoll(), 2);
   /* Serial.print  ln("PI " + (String) pI);
    Serial.println("rI" + (String) rI);
    Serial.println("roll_incr " + (String) roll_incr);
    Serial.println("pitch_incr" + (String) pitch_incr); */

    }
    
/*
if (Serial.available() >0){
              String input = Serial.readString();
              if (input.indexOf("d") > -1){
                Dco = (input.substring(1)).toFloat();
                Serial.println("d:" + (String) Dco);
              }
              else if (input.indexOf("p") > -1){
                Pco = (input.substring(1)).toFloat();
                Serial.println("p:" + (String) Pco);
              }
              else if (input.indexOf("i") > -1){
                Ico = (input.substring(1)).toFloat();
                Serial.println("i:" + (String) Ico);
              }
            }*/
