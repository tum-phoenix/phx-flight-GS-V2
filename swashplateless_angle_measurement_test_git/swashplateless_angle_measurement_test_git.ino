// This script test the rpm of the brushless motor and performs a test on the swashplateless prototyp design.   

#include <AS5047P.h>
#include <PulsePosition.h>
#include<math.h>

// Pin declaration for Teensy (9 by Arduino Uno)
#define AS5047P_CHIP_SELECT_PORT 10  
#define AS5047P_CUSTOM_SPI_BUS_SPEED 100000


// initialize a new AS5047P sensor object.
AS5047P as5047p(AS5047P_CHIP_SELECT_PORT, AS5047P_CUSTOM_SPI_BUS_SPEED);

byte PWM_PIN = 23; // 3;
int pwmValue =118;
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
int main_loop_duration;
double controlAngle;
PulsePositionInput myIn;
float ppmValues[8];
float throttle, pitch, roll, yaw;


// arduino setup routine
void setup() {
  //TCCR2B = TCCR2B & B11111000 | B00000010; // for PWM frequency of 31372.55 Hz
  analogWriteFrequency(23,4000);
  myIn.begin(15);  
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
    getReceiverValues();
    getTPRY();
    handleSerial();
    motorAngle = as5047p.readAngleDegree();
    if(micros()-timer > 1000){
      computeRPM();
      timer = micros();
      }
    computeControl();
    manipulateSpeed();
    //manipulateSpeed_Sinus();
    //getrpm();

    //Set speed for the ESC
    if(micros()-timer_1 > 1000){
      printInfo();
      timer_1 = micros();
      }
    
    analogWrite(PWM_PIN, moduelate_pwmValue); 
    main_loop_duration = micros()- timer_2;
}

void printInfo(){
   //print different output using Serial print
    Serial.print("V: ");
    Serial.print(moduelate_pwmValue);
    Serial.print(" ");
    Serial.print("R: ");
    Serial.print(rpm);
    Serial.print(" ");
    Serial.print("x: ");
    Serial.print(controlAngle);
    Serial.print(" ");
    Serial.print(angle_offset);
    Serial.print(" ");
    Serial.print(modulation_offset);
    Serial.print(" ");
    
    //Serial.print("main_loop_duration: ");
    //Serial.print(main_loop_duration);                        
    Serial.println();
  }

void computeRPM(){
    // compute the RPM of the brushless motor using the as5047p sensory.  
      float angleDifference;
      if(abs(motorAngle- previousMotorAngle)>300){
          if(motorAngle>300){
              angleDifference = -(360 - motorAngle + previousMotorAngle);
            }
          else{
              angleDifference = 360 + motorAngle - previousMotorAngle;
            }
        }
      else{
          angleDifference = motorAngle - previousMotorAngle;
        }
      rpm = (angleDifference)/360*1000; 
      previousMotorAngle = motorAngle;
  }


void getTPRY(){
  throttle = ppmValues[0];
  roll = ppmValues[1];
  pitch = ppmValues[2];
  yaw = ppmValues[3];
  }


void computeControl(){
  int minpwmValue = 145;
  if (pwmValue > minpwmValue){
    modulation_offset = (pwmValue-minpwmValue) * sqrt(pow(normalizePRY(pitch),2) + pow(normalizePRY(roll),2) );
  }
  controlAngle = atan2(normalizePRY(pitch), normalizePRY(roll));  
  
  if (controlAngle > 0 ){
    angle_offset = 180*float(controlAngle)/PI;
  }else{
    angle_offset = 360 + 180*float( controlAngle)/PI;
    }
  pwmValue = 128 + 128 * normalizeThrottle(throttle);
  }


void manipulateSpeed(){
    //changing speed using the serial inputs 
    if((int(motorAngle)+angle_offset)%360 > 180){
        moduelate_pwmValue = pwmValue + modulation_offset; 
      }
    else{
        moduelate_pwmValue = pwmValue - modulation_offset; 
      }
  } 

void manipulateSpeed_Sinus(){
    //Serial.println(sin((int(motorAngle)+angle_offset)));
    moduelate_pwmValue = pwmValue+int(modulation_offset*sin((motorAngle+angle_offset)/180 * PI));
  }

void manipulateSpeed_Sinus_V2(){
    //Serial.println(sin((int(motorAngle)+angle_offset)));
    //moduelate_pwmValue = pwmValue+int(modulation_offset*sin(inverse_rotion_pos_func( (int(motorAngle)+angle_offset),rpm )));
  }

void getReceiverValues(){
  int i, num;
  num = myIn.available();
  if (num > 0) {
    for (i=1; i <= num; i++) {
      ppmValues[i-1] = myIn.read(i);
      float val = myIn.read(i);
    }    
  }
}


float normalizeThrottle(float pulseValue){
    return (pulseValue - 1000)/ 1000;
  }


float normalizePRY(float pulseValue){
    return (pulseValue - 1500)/ 500;
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
     case '9':
      pwmValue = 255; 
      Serial.println(pwmValue);
      break;

     case '5':
      pwmValue = 192; 
      Serial.println(pwmValue);
      break; 
     
     case '+':
      pwmValue = pwmValue + 2; 
      Serial.println(pwmValue);
      break;

     case '-':
      pwmValue = pwmValue - 1;
      Serial.println(pwmValue);
      break;
      
     case '0':
      pwmValue = 124;
      break;

      case 'i':
      pwmValue = 135;
      break;
     
     case 's':
      modulation_offset = pwmValue - 140;
      break;

    case 'b':
      modulation_offset = 0;
      break;
    
    case 'a':
      angle_offset += 45;
      break;

    case 'p':
      printInfo();
      break;
    }
}
