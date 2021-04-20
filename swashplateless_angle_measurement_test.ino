// This script test the rpm of the brushless motor and performs a test on the swashplateless prototyp design.   

#include <AS5047P.h>

// Pin declaration for Teensy (9 by Arduino Uno)
#define AS5047P_CHIP_SELECT_PORT 10 
#define AS5047P_CUSTOM_SPI_BUS_SPEED 100000


// initialize a new AS5047P sensor object.
AS5047P as5047p(AS5047P_CHIP_SELECT_PORT, AS5047P_CUSTOM_SPI_BUS_SPEED);

byte PWM_PIN = 3;
int pwmValue =125;
int moduelate_pwmValue =125;

unsigned long pretime, current_time;
unsigned long time_diff;
float motorAngle;
float previousMotorAngle;
int counter=0;
float rpm;

unsigned long timer = 0;
unsigned long timer_1 = 0;
unsigned long timer_2 = 0;
int angle_offset = 0;
int modulation_offset = 0;

// arduino setup routine
void setup() {
  // initialize the serial bus for the communication with your pc.
  Serial.begin(115200);
  analogWrite(PWM_PIN, pwmValue); 
  
  // initialize the AS5047P sensor and hold if sensor can't be initialized..
  while (!as5047p.initSPI()) {
    Serial.println(F("Can't connect to the AS5047P sensor! Please check the connection... "));
    delay(5000);
  }
}


void loop() {
    timer_2 = micros();
   
    handleSerial();
    manipulateSpeed();
    //getrpm();

    //Set speed for the ESC
    analogWrite(PWM_PIN, moduelate_pwmValue); 

    // update the 
    if(micros() - timer > 1000){
      computeRPM();
      timer = micros();
    }
    
    if(micros() - timer_1 > 50000){
      timer_1 = micros();
    }
}

void printInfo(){
   //print different output using Serial print
    Serial.print(rpm);
    //Serial.print(" "); 
    //Serial.print(time_diff);                        
    Serial.println();
  }

void computeRPM(){
    // compute the RPM of the brushless motor using the as5047p sensory.  
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
  }

void manipulateSpeed(){
    //changing speed using the serial inputs 
    if((int(motorAngle)+angle_offset)%360 > 180){
        moduelate_pwmValue = pwmValue + modulation_offset; 
      }
    else{
        moduelate_pwmValue = pwmValue ; 
      }
  } 

void getrpm(){
    // An alternative way to determine the rpm
    if (abs(motorAngle-previousMotorAngle) > 300){
        current_time = micros();
        time_diff = current_time - pretime;
        rpm = 10e5/time_diff ;
        pretime = current_time;                       
      }
  }


void handleSerial() {
   // Logik of the Serial input 
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
