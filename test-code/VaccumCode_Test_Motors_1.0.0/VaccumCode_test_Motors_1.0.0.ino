//Code: VaccumCode_test_Motors
//Version: 1.0.0
//Author: Cesar Nieto refer to ces.nietor@gmail.com
//Last change: 05/09/2017
//Changes: Just to debug the motors and sensors
#include <math.h> 
////////////PINS////////////////
//Distance Analog Sensors (Sharp)
const int SD1 = 0; //left front sensor
const int SD2 = 1; //right front sensor
const int SD3 = 2; //left side sensor
const int SD4 = 3; //right side sensor

//Battery Voltage input
const int battery = 4;

//IndicatorLED
const int led = 13;

//Fan output
const int fanmotor =  12;      // the number of the LED pin

// Motor1 Right
const int motor1Pin1 = 3;
const int motor1Pin2 = 5;

// Motor2 Left
const int motor2Pin1 = 6;
const int motor2Pin2 = 9;

//Bumper
const int bumper1 = 10;
const int bumper2 = 11;

///////////////Constants////////////////
const float voltageBatCharged = 12.68; // Voltage measured when battery fully charged //Change this
//PWM for the micro metal motors
const int pwmMax = 160;// Max value = 255
const int pwmMin = 70;;// 
//Mínimun distance of the sensor
const int minSharp = 30;

// Variables will change:
int bumperState = 0;  // variable for reading the pushbutton status
boolean control = true;
int counter = 0; //   Prevents from being stuck

//////////////CODE/////////////
void setup() {
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
  // initialize the pushbutton inputs 
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
  // Initialize serial
  Serial.begin(9600);    
  ///////////////////////////////Wait////////////////////////////////////////
  //Wait about 5 s and initialize fan if voltage ok
  waitBlinking(5,1); //5 seconds at 1 Hz
  //Crank (initialize the fan because the voltage drops when cranking)
  if(readBattery(battery)>12.1){
    digitalWrite(fanmotor, HIGH); //Turn the Fan ON
    delay(1000); //For 1000ms
  }
  else {
    //do nothing Convention
    }
} 
//////////Functions To Use //////////
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
void forwardMotors(int moveTime){  
  //Manipulate direction according the desired movement of the motors
   analogWrite(motor1Pin1, pwmMin); 
   analogWrite(motor1Pin2, 0); //PWM value wher 0 = 0% and 255 = 100%
   analogWrite(motor2Pin1, pwmMin); 
   analogWrite(motor2Pin2, 0); 
   delay(moveTime);
}
void rightMotors(int moveTime){ 
   analogWrite(motor1Pin1, 0); 
   analogWrite(motor1Pin2, pwmMin); 
   analogWrite(motor2Pin1, pwmMin);
   analogWrite(motor2Pin2, 0); 

   delay(moveTime);
}
void leftMotors(int moveTime){ 
   analogWrite(motor1Pin1, pwmMin); 
   analogWrite(motor1Pin2, 0); 
   analogWrite(motor2Pin1, 0);
   analogWrite(motor2Pin2, pwmMin+20); 
   delay(moveTime);
}
void backwardMotors(int moveTime){
   analogWrite(motor1Pin1, 0); 
   analogWrite(motor1Pin2, pwmMin+20);
   analogWrite(motor2Pin1, 0); 
   analogWrite(motor2Pin2, pwmMin+20); 
   delay(moveTime);
}
void stopMotors(){ 
   analogWrite(motor1Pin1, 0);
   analogWrite(motor1Pin2, 0); 
   analogWrite(motor2Pin1, 0); 
   analogWrite(motor2Pin2, 0); 
}
float  readBattery(int input){
  int readInput;
  float voltage;
  readInput = analogRead(input);
  voltage = (((readInput*4.9)/1000)*voltageBatCharged ) / 5; // resolution of analog input = 4.9mV per Bit 
  Serial.print(" Battery= ");
  Serial.print(voltage);
  return voltage;
  } 
void batteryControl(int input){
  //Turn everything off in case the battery is low
  float v_battery;
  v_battery = readBattery(input);
  if(v_battery<=11.6){ //battery limit of discharge, Don't put this limit lower than  11.1V or you can kill the battery
    control = false;
    }
  else {
    //Do nothing Convention
    }
}
/////////////////////////////////////////////////MAIN CODE//////////////////////////////
void loop(){
  Serial.print("SD1= ");
  Serial.print(sdSHARP(SD1));
  Serial.println();
  Serial.print("  SD2= ");
  Serial.print(sdSHARP(SD2));
  Serial.println();
  delay(200);
  bumperState = digitalRead(bumper1);
  //Keep the control of the battery automatically turn the fan off
  //If control = true the battery level is ok, otherwise the battery is low.
  batteryControl(battery); //modifies the variable control of the battery is low
  
  if (control){
    digitalWrite(led, HIGH);
    forwardMotors(0);
  }
  else if (!control){
    //If the battery is low, turn everything off
    digitalWrite(fanmotor, LOW); //Turn the Fan OFF
    stopMotors();
    Serial.print(" Low Battery! ");
    Serial.println();
    waitBlinking(1,3);  //blink as warning 3hz in a loop
    }
  Serial.println();
}
