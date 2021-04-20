#include <AS5047P.h>
#include "RunningAverage.h"

#define AS5047P_CHIP_SELECT_PORT 10 
#define AS5047P_CUSTOM_SPI_BUS_SPEED 100000


// initialize a new AS5047P sensor object.
AS5047P as5047p(AS5047P_CHIP_SELECT_PORT, AS5047P_CUSTOM_SPI_BUS_SPEED);
RunningAverage rotation_position(10);

byte PWM_PIN = 3;
int pwmValue =125;
int moduelate_pwmValue =125;

// arduino setup routine
void setup() {

  // set the pinmode of the led pin to output.
  // initialize the serial bus for the communication with your pc.
  Serial.begin(115200);

  // initialize the AS5047P sensor and hold if sensor can't be initialized..
  analogWrite(PWM_PIN, pwmValue); 

  while (!as5047p.initSPI()) {
    Serial.println(F("Can't connect to the AS5047P sensor! Please check the connection... "));
    delay(5000);
  }

}
unsigned long pretime, current_time;
unsigned long time_diff;
float motorAngle;
float previousMotorAngle;
int counter=0;
float rpm;
// arduino loop routine

unsigned long timer = 0;
unsigned long timer_1 = 0;
unsigned long timer_2 = 0;
int angle_offset = 0;

void loop() {
    timer_2 = micros();
    handleSerial();
    manipulateSpeed();

    //getrpm();
    analogWrite(PWM_PIN, moduelate_pwmValue); 
    
    if(micros() - timer > 1000){
      
      motorAngle = as5047p.readAngleDegree();
      float angleDifference;
      if(abs(motorAngle- previousMotorAngle)>300){
          if(motorAngle>300){
              angleDifference = -(360 - motorAngle + previousMotorAngle);
            }
          else{
              angleDifference = -(360 + motorAngle - previousMotorAngle);
            }
        }
      else{
          angleDifference = motorAngle - previousMotorAngle;
        }
      
      rpm = (angleDifference)/360*1000; 
      previousMotorAngle = motorAngle;
      timer = micros();
    }
    
    if(micros() - timer_1 > 50000){
      Serial.print(rpm);
      //Serial.print(" "); 
      //Serial.print(time_diff);                        
      Serial.println();
      timer_1 = micros();
    }
    //Serial.print(micros() - timer_2);
    //Serial.println(" "); 
}
int modulation_offset = 0;

void manipulateSpeed(){
    if((int(motorAngle)+angle_offset)%360 > 180){
        moduelate_pwmValue = pwmValue + modulation_offset; 
      }
    else{
        moduelate_pwmValue = pwmValue ; 
      }
  } 

void getrpm(){
    if (abs(motorAngle-previousMotorAngle) > 300){
        current_time = micros();
        time_diff = current_time - pretime;
        rpm = 10e5/time_diff ;
        pretime = current_time;                       
      }
  }


void handleSerial() {
   char incomingCharacter = Serial.read();
   switch (incomingCharacter) {
     case '+':
      pwmValue = pwmValue + 1;
      break;

     case '-':
      pwmValue = pwmValue - 1;
      break;
      
     case '0':
      pwmValue = 127;
      break;

      case 'i':
      pwmValue = 130;
      break;
     
     case 's':
      modulation_offset = 20;
      break;

    case 'b':
      modulation_offset = 0;
      break;
    
    case 'a':
      angle_offset += 10;
      break;
    }
}
