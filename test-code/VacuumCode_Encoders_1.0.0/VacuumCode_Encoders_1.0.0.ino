//Code: VaccumCode_Encoders
//Version: 1.0.0
//Author: Cesar Nieto refer to ces.nietor@gmail.com
//Last change: 04/10/2017
//Last Changes: Code added to support Pololu Motor Encoders, pins have been changed.
//              A PD controller is used to control the speed of the motors.                
#include <math.h> 
////////////PINS////////////////
// Distance Analog Sensors (Sharp)
#define SD1     (0)   //left front sensor
#define SD2     (1)   //right front sensor
#define SD3     (2)   //left side sensor
#define SD4     (3)   //right side sensor
// Battery Voltage input
#define battery     (4)   //Analog
// IndicatorLED
#define led         (13)
// Fan output
#define fanmotor    (12)  // the number of the LED pin
// Motor1 Right
#define motor1Pin1  (9)
#define motor1Pin2  (10)
#define encodPinA1  (2)   // encoder A pin, interrupt pin of Arduino Uno
#define encodPinB1  (4)   // encoder B pin, read motor direction
// Motor2 Left
#define motor2Pin1  (5)
#define motor2Pin2  (6)
#define encodPinA2  (3)   // encoder A pin, interrupt pin of Arduino Uno
#define encodPinB2  (7)   // encoder B pin, read motor direction
// Bumper
#define bumper1     (8)
#define bumper2     (11)
// PWM for the micro metal motors //Values to delete soon since use of PID
#define pwmMax      (160) 
#define pwmMin      (0)  
// PID loop time
#define LOOPTIME  (100)                     

///////////////Constants////////////////
const float voltageBatCharged = 12.68; // Voltage measured when battery fully charged //Change this
const float batteryLimitDischarge = 11.6; // Safe value to not kill the Battery

// Variables will change:
int bumperState = 0;  // variable for reading the pushbutton status
int counter = 0; //   Prevents from being stuck
boolean control = true;
boolean printOnce = true; //For debugging

unsigned long lastMilli = 0;    // loop timing 
unsigned long lastMilliPrint = 0;
unsigned long lastErrorMilli = 0;
//Motor 1
float speed_req1 = 60.0;            // speed (Set Point)
float speed_act1 = 0.0;             // speed (actual value)
int PWM_val1 = 0;               // (25% = 64; 50% = 127; 75% = 191; 100% = 255)
volatile long count_1 = 0;      // rev counter
//Motor 2
float speed_req2 = 60.0;
float speed_act2 = 0.0;
int PWM_val2 = 0;
volatile long count_2 = 0;

float Kp =   0.6;// 0.65;        // PID Proportional control Gain   (Good values as well: 0.5)     
float Kd =   0.0;// 0.005;       // PID Derivitave control Gain

//////////////CODE/////////////
void setup() {
  Serial.begin(9600); // Initialize serial
  //Initialize outputs and inputs
  //Fan motor as output
  pinMode(fanmotor, OUTPUT);
  //Motor1
  pinMode(motor1Pin1, OUTPUT);
  pinMode(motor1Pin2, OUTPUT);
  //Motor2
  pinMode(motor2Pin1, OUTPUT);
  pinMode(motor2Pin2, OUTPUT);
  //LED
  pinMode(led, OUTPUT);
  //INPUTS
  //Initialize the pushbutton inputs 
  //Motor encoders
  pinMode(encodPinA1, INPUT_PULLUP); 
  pinMode(encodPinB1, INPUT_PULLUP);
  pinMode(encodPinA2, INPUT_PULLUP); 
  pinMode(encodPinB2, INPUT_PULLUP); 
  //Bumper
  pinMode(bumper1, INPUT_PULLUP); 
  pinMode(bumper2, INPUT_PULLUP); 
  //Sensor
  pinMode(SD1, INPUT);
  pinMode(SD2, INPUT);
  pinMode(SD3, INPUT);
  pinMode(SD4, INPUT);
  //Batt
  pinMode(battery, INPUT);
  //Encoder Interrupt executes rencoder_# function when falling edge of the signal
  //Refer to: https://www.arduino.cc/en/Reference/AttachInterrupt
  //Arduino UNO only has 2 external interrupts
  attachInterrupt(0, rencoder_1, FALLING);  //Pin 2 of Arduino Uno
  attachInterrupt(1, rencoder_2, FALLING);  //Pin 3 of Arduino Uno
    
  ///////////////////////////////Wait////////////////////////////////////////
  //Wait about 5 s and initialize fan if voltage ok
  Serial.println("Starting...");
  waitBlinking(3,1); //5 seconds at 1 Hz
  //Crank (initialize the fan because the voltage drops when cranking)
  if(readBattery(battery)>12.1){
    digitalWrite(fanmotor, HIGH); //Turn the Fan ON
    delay(500); //For 1000ms
  }
  else {
    //do nothing Convention
    }
} 
/////////////////////////////////////////////////MAIN CODE//////////////////////////////
void loop(){
  //Keep the control of the battery automatically turn the fan off
  //If control = true the battery level is ok, otherwise the battery is low.
  batteryControl(battery); //modifies the variable control of the battery is low
  setMotors(); //Set pwm of each motor according to the actual speed
  controlRobot(); // Execute all conditions to move
  printMotorsInfo();
}

//////////Functions To Use //////////
//PORTD, containts Digital pins 0-7, 4 and 7 used to consider the direction of each motor. (PIND is for read only) 
void rencoder_1()  {                                  // pulse and direction, direct port reading to save cycles
  if (PIND & 0b00010000)     count_1++;                // if(digitalRead(encodPinB1)==HIGH)   count ++;
  else                      count_1--;                // if (digitalRead(encodPinB1)==LOW)   count --;
}

void rencoder_2()  {                                  // pulse and direction, direct port reading to save cycles
  if (PIND & 0b10000000)     count_2++;                // if(digitalRead(encodPinB2)==HIGH)   count ++;
  else                      count_2--;                // if (digitalRead(encodPinB2)==LOW)   count --;
}
void waitBlinking(int n, int frequency){
  //blink for n seconds at frequency hz
  for (int i=1; i <= n; i++){
    for(int j=1; j<=frequency; j++){
      digitalWrite(led, HIGH);   
      delay((1000/frequency)/2);   //Half time on            
      digitalWrite(led, LOW);   
      delay((1000/frequency)/2);   //Half time off
    }
   } 
}
double sdSHARP(int Sensor){
  //Returns the distance in cm
  double dist = pow(analogRead(Sensor), -0.857); // x to power of y
  return (dist * 1167.9);
}
float  readBattery(int input){
  int readInput;
  float voltage;
  readInput = analogRead(input);
  voltage = (((readInput*4.9)/1000)*voltageBatCharged ) / 5; // resolution of analog input = 4.9mV per Bit resolution
  Serial.print(" Battery= ");
  Serial.print(voltage);
  return voltage;
  } 
void batteryControl(int input){
  //Turn everything off in case the battery is low
  float v_battery;
  v_battery = readBattery(input);
  if(v_battery<=batteryLimitDischarge){ //battery limit of discharge, Don't put this limit lower than  11.1V or you can kill the battery
    control = false;
    }
  else {
    //Do nothing Convention
    }
}

void moveMotors(int moveTime, int pwmMotor1, int pwmMotor2, char direc){
  //Manipulate direction according the desired movement of the motors
  switch(direc){
    case 'f':
      analogWrite(motor1Pin1, pwmMotor1); 
      analogWrite(motor1Pin2, 0); //PWM value wher 0 = 0% and 255 = 100%
      analogWrite(motor2Pin1, pwmMotor2); 
      analogWrite(motor2Pin2, 0); 
      delay(moveTime);
      break;
    case 'b':
      analogWrite(motor1Pin1, 0); 
      analogWrite(motor1Pin2, pwmMotor1);
      analogWrite(motor2Pin1, 0); 
      analogWrite(motor2Pin2, pwmMotor2); 
      delay(moveTime);
      break;
    case 'r':
      analogWrite(motor1Pin1, 0); 
      analogWrite(motor1Pin2, pwmMotor1); 
      analogWrite(motor2Pin1, pwmMotor2);
      analogWrite(motor2Pin2, 0); 
      delay(moveTime);
      break;
    case 'l':
      analogWrite(motor1Pin1, pwmMotor1); 
      analogWrite(motor1Pin2, 0); 
      analogWrite(motor2Pin1, 0);
      analogWrite(motor2Pin2, pwmMotor2); 
      delay(moveTime);
      break;
    case 's':
      analogWrite(motor1Pin1, 0);
      analogWrite(motor1Pin2, 0); 
      analogWrite(motor2Pin1, 0); 
      analogWrite(motor2Pin2, 0); 
      delay(moveTime);
      break;
    default:
      //Do nothing convention
      Serial.println("Default");
      break;
  }
}
void getMotorsSpeed()  {  
  static long countAnt_1 = 0, countAnt_2 = 0, countAnt_3 = 0, countAnt_4 = 0;   // last count, static variables preserve the last value
  speed_act1 = ((count_1 - countAnt_1)*(60*(1000/LOOPTIME)))/(3*298);         // 3 pulses X 298 gear ratio = 894 counts per output shaft rev
  countAnt_1 = count_1;
  speed_act2 = ((count_2 - countAnt_2)*(60*(1000/LOOPTIME)))/(3*298);         
  countAnt_2 = count_2;
}
int updatePid(int command, int targetValue, int currentValue)   {             // compute PWM value
  float pidTerm = 0;                                                            // PID correction
  float error = 0;                                  
  static float last_error = 0;
  unsigned long reachTime = 0;
  error = abs(targetValue) - abs(currentValue); 
  //Serial.print(" Error: ");      Serial.print(error);                
  /*if (error <= 1.0 && printOnce){
    //Measure time to see when the motor reached the Set point
    reachTime = millis()-lastErrorMilli;
    Serial.print("SP reachead: ");  Serial.print(reachTime); Serial.println();
    printOnce = false;
    }
  */
  //PID controller, not using Ki at the moment
  pidTerm = (Kp * error) + (Kd * (error - last_error));                            
  last_error = error;
  return constrain(command + int(pidTerm), 0, 255);
}
void setMotors(){
  if((millis()-lastMilli) >= LOOPTIME)   {                                    // enter tmed loop
    lastMilli = millis();
    getMotorsSpeed();                                                          // calculate speed, volts and Amps
    //Global values
    PWM_val1 = updatePid(PWM_val1, speed_req1, speed_act1);                   // compute PWM value
    PWM_val2 = updatePid(PWM_val2, speed_req2, speed_act2);                   // compute PWM value
  }
}
void printMotorsInfo()  {                                                      // display data
  if((millis()-lastMilliPrint) >= 500)   {                     
    lastMilliPrint = millis();
    Serial.print("  SP_1:");              Serial.print(speed_req1);  
    Serial.print("  RPM_1: ");          Serial.print(speed_act1);
    Serial.print("  PWM_1: ");          Serial.print(PWM_val1);
    Serial.print(", SP_2: ");           Serial.print(speed_req2);  
    Serial.print("  RPM_2: ");          Serial.print(speed_act2);
    Serial.print("  PWM_2: ");          Serial.print(PWM_val2);    
    Serial.println();
  }
}
void controlRobot(){
  
  Serial.print("SD1= ");
  Serial.print(sdSHARP(SD1));
  Serial.println();
  Serial.print("  SD2= ");
  Serial.print(sdSHARP(SD2));
  Serial.println();
  //delay(200);*/
  float minDistanceSharp = 5; // Distance in cm
  bumperState = digitalRead(bumper1);
  if (control){
    digitalWrite(led, HIGH);
    if (sdSHARP(SD1)<= minDistanceSharp){ 
      //If the distance between an object and the left front sensor is less than 4.3 cm or the bumper hits, it will move to the left
      if (counter ==2){ // prevent of being stuck on corners
        counter = 0;
        }
      else {
        //Do nothing Convention
      }
      moveMotors(100, PWM_val1, PWM_val2, 'f'); // approach a bit
      moveMotors(500, PWM_val1, PWM_val2, 'b'); // backward delay of 500ms
      moveMotors(300, PWM_val1, PWM_val2, 'l');
      counter = counter + 2;
      Serial.print("  Turn Left ");
      }
    else if (sdSHARP(SD2)<= minDistanceSharp){ 
      //If the distance between an object and the right front sensor is less than 4.3 cm, it will move to the right
      if (counter ==1){
        counter = 0;
        }
      else{
        //Do nothing Convention
      }
      moveMotors(100, PWM_val1, PWM_val2, 'f'); 
      moveMotors(500, PWM_val1, PWM_val2, 'b');
      moveMotors(300, PWM_val1, PWM_val2, 'r');
      counter++;
      Serial.print("  Turn Right");
      }
    else if (bumperState==0){
      counter = 0;
      moveMotors(500, PWM_val1, PWM_val2, 'b'); 
      moveMotors(300, PWM_val1, PWM_val2, 'l');
      Serial.print("  Turn Left ");
      }
    else {
      if(counter==3){ //Corner
        moveMotors(1000, PWM_val1, PWM_val2, 'l');
        counter = 0;
        }
      else {
        moveMotors(300, PWM_val1, PWM_val2, 'f');
      }
      //Serial.print("  Move Forward");
      }
  }
  else if (!control){
    //If the battery is low, turn everything off
    digitalWrite(fanmotor, LOW); //Turn the Fan OFF
    moveMotors(0, 0, 0, 's');
    Serial.print(" Low Battery! ");
    Serial.println();
    waitBlinking(1,3);  //blink as warning 3hz in a loop
    }
  //Serial.println();
}


