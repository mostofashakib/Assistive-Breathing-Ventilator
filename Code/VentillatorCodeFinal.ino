/*	Code for Texas Tech University Project Lab 1
* 	Authors:
*	Adib Shakib, Austin Cawley, Mushfique Kahn
*
*	ARDUINO MEGA PIN ASSIGNMENTS
*	2	PWM	trigPin
*	3	PWM	ONE_WIRE_BUS - data wire
*	4	PWM	buzzer
*	5	PWM	pinSTEP
*	6	PWM	
*	7	PWM	pressureMoniter
*	8	PWM	
*	9	PWM	
*	10	PWM	
*	11	PWM	
*	12	PWM	
*	13	PWM	
*	
*	22	LCD0
*	23	LCD1
*	24	LCD2
*	25 	LCD3
*	26	LCD4
*	27	LCD5
*	28	echoPin
*	29	pressure output 1
*	30	pressure output 2
*	31 	pressure output 3
*	32	pinDIR
*	33	pinENABLE
*	34	pinExPeriod0
*	35	pinExPeriod1
*	36	pinExPeriod2
*	37	pinInPeriod0
*	38	pinInPeriod1
*	39	pinInPeriod2
*	40	pinVolume0
*	41	pinVolume1
*	42	pinVolume2
*	43	
*	44	
*	45  valveInServo
*	46	valueOutServo
*	47	
*	48	
*	49	
*	50	
*	51	
*	52	
*	53	
*/
 
#include <Arduino.h>
#include <LiquidCrystal.h>                      //  Load Liquid Crystal Library
#include <Wire.h>  
#include <Servo.h>
//#include <PressureMonitor.h>
#include "BasicStepperDriver.h"
#include "DRV8834.h"

// Motor steps per revolution. Most steppers are 200 steps or 1.8 degrees/step
#define MOTOR_STEPS 200

// Since microstepping is set externally, make sure this matches the selected mode
// If it doesn't, the motor will move at a different RPM than chosen
// 1=full step, 2=half step etc.
#define MICROSTEPS 1

#define pinDIR 32
#define pinSTEP 5
#define pinENABLE 33

#define pinExPeriod0 34
#define pinExPeriod1 35
#define pinExPeriod2 36

#define pinInPeriod0 37
#define pinInPeriod1 38
#define pinInPeriod2 39

#define pinVolume0 40
#define pinVolume1 41
#define pinVolume2 42

#define ONE_WIRE_BUS 4                          //  Data wire is plugged into pin 6 on the Arduino 


//------------LCD, Pressure Sensor, Buzzer Setup-----------
LiquidCrystal LCD(27, 26, 25, 24, 23, 22);          //  Create Liquid Crystal Object called LCD

int trigPin = 9;                                //  Sensor Trigger pin connected to Arduino pin 9
int echoPin = 7;                                //  Sensor Echo pin connected to Arduino pin 7
int myCounter = 0;                              //  Declare your variable myCounter and set to 0
int buzzerPin = 4;                              //  Declare buzzer pin to be 6
int redLED = 29;
int greenLED = 30;
int yellowLED = 31;

float pingTime;                                 //  Time for ping to travel from the sensor to the target and return
float targetDistance;                           //  Distance to Target in Centimeters
float heightDifference;                         //  Height difference in Meters
float calculatedPressure;                       //  Pressure in Pascals
float speedOfSound = 776.5;                     //  Speed of sound in miles per hour
float lowerBoundPressure = 392.4;               //  This is the lower bound at 4 cm
float upperBoundPressure = 1962;                //  This is the upper bound at 20cm
float densityOfWater = 1000;                    //  This is the density of water 1000kg/m^3
float accelerationGravity = 9.81;               //  This is the acceleration due to gravity 9.81 m/s^2
float initialHeight = 10;                       //  Taking initial height to 10 cm

//----------Stepper Motor Setup-----------------------------
//setup stepper motor
DRV8834 stepper(200, pinDIR, pinSTEP, pinENABLE);
float rpmIn = 0;
float rpmOut = 0;

int periodIn = 0;
int periodEx = 0;
int volume = 0;

int warning = 0;
//--------------Airflow Sensors Setup----------------------

float CurrentReading;
float Sum;
float Average;
float MeasuresN=2500 ; // number of measures to average
float Flow;

//-------------Servomotor Setup------------------------
Servo valveInServo;       // create servo object to control a servo
Servo valveOutServo;
//PressureMonitor pin(7); // creates PressureMonitor Object and declares input pin as pin 7

int openPosition = 0;     // variable to store the servo position to open the valve
int closedPosition = 180; // variable to store the servo position to close the valve

void setup() {
	//------PRESSURE SENSOR
  Serial.begin(9600);
  pinMode(trigPin, OUTPUT);                     //  Set Sensor Trigger pin as an output
  pinMode(echoPin, INPUT);                      //  Set Sensor Echo pin as an input
  pinMode(buzzerPin, OUTPUT);                           //  Set buzzer - pin 6 as an output
  LCD.begin(16, 2);                             //  Start the 16x2 LCD Liquid Crystal display
  LCD.setCursor(0, 0);                          //  Set LCD cursor to upper left corner, column 0, row 0
  LCD.print("Distance:");                       //  Print Message on First Row
  pinMode(yellowLED, OUTPUT);                           
  pinMode(redLED, OUTPUT);
  pinMode(greenLED, OUTPUT);                           
  
  //--------STEPPER
	pinMode(pinDIR, OUTPUT);
	pinMode(pinSTEP, OUTPUT);
	pinMode(pinENABLE, OUTPUT);
	
	pinMode(pinExPeriod0, INPUT_PULLUP);
	pinMode(pinExPeriod1, INPUT_PULLUP);
	pinMode(pinExPeriod2, INPUT_PULLUP);
	pinMode(pinInPeriod0, INPUT_PULLUP);
	pinMode(pinInPeriod1, INPUT_PULLUP);
	pinMode(pinInPeriod2, INPUT_PULLUP);
	pinMode(pinVolume0, INPUT_PULLUP);
	pinMode(pinVolume1, INPUT_PULLUP);
	pinMode(pinVolume2, INPUT_PULLUP);

  //---------Servo
  valveInServo.attach(45);  // attaches the servo on pin 45 to the servo object
  valveOutServo.attach(46);  // attaches the servo on pin 46 to the servo object
  
}

void loop() {

	//--------PRESSUR SENSOR-------
    digitalWrite(trigPin, LOW);                 // Set trigger pin low
    delayMicroseconds(2000);                    // Let signal settle
    digitalWrite(trigPin, HIGH);                // Set trigPin high
    delayMicroseconds(15);                      // Delay in high state
    digitalWrite(trigPin, LOW);                 // Ping has now been sent
    delayMicroseconds(10);                      // Delay in high state

    pingTime = pulseIn(echoPin, HIGH);          // PingTime in microceconds
    pingTime = pingTime / 1000000;              // Convert pingTime to seconds by dividing by 1000000 (microseconds in a second)
    pingTime = pingTime / 3600;                 // Convert pingtime to hours by dividing by 3600 (seconds in an hour)
    targetDistance = speedOfSound * pingTime;   // Distance tranvelled in miles
    targetDistance = targetDistance / 2;        // Actual distance travelled
    targetDistance = targetDistance * 160934.4; // Convert miles to centimeters by multipling by 160934.4
    heightDifference = targetDistance * 0.01;   // Converts the height difference from centimeter to meter
    calculatedPressure = heightDifference * densityOfWater * accelerationGravity;  // This is the pressure of the gas hpg

    LCD.setCursor(10, 0);                       // Set the cursor to the tenth column of the first row
    LCD.print("                ");              // Print blanks to clear the row
    LCD.setCursor(10, 0);                       // Set Cursor again to the tenth column of the first row
    LCD.print((int) (targetDistance + 0.5));    // Print measured distance
    LCD.print("cm ");                           // Print your units
    LCD.setCursor(0, 1);                        // Set the cursor to the first column of the second row
    LCD.print("                ");              // Print blanks to clear the row
    LCD.setCursor(0, 1);                        // Set Cursor again to the first column of the second row


    if (calculatedPressure > upperBoundPressure) {
        LCD.print("Pressure");
        LCD.print((int) (calculatedPressure));
        LCD.print("Pa "); 
    }
    else if (calculatedPressure < upperBoundPressure && calculatedPressure > lowerBoundPressure) {
        LCD.print("Pressure");
        LCD.print((int) (calculatedPressure));
        LCD.print("Pa "); 
    }
    else (calculatedPressure < lowerBoundPressure); {
        LCD.print("Pressure");
        LCD.print((int) (calculatedPressure));
        LCD.print("Pa ");
    }

    delay(2000);

   if (calculatedPressure > upperBoundPressure) {
        digitalWrite (redLED, HIGH);                 //   Red LED ON 10
        for(int i = 0; i < 500; i++){
          digitalWrite (buzzerPin, HIGH);                  // turn the buzzer on (HIGH is the voltage level)
          }
		warning = 1;
    }
    else {
        digitalWrite(redLED, LOW);                   //    Red LED OFF
        digitalWrite (buzzerPin, LOW);              // turn the buzzer OFF (HIGH is the voltage level)
    }

    if (calculatedPressure < upperBoundPressure && calculatedPressure > lowerBoundPressure) {
        digitalWrite (greenLED, HIGH);                  //    Green LED ON 13
		warning = 0;
    }
    else {
        digitalWrite(greenLED, LOW);                    //    Green LED OFF
    }
    
    if (calculatedPressure < lowerBoundPressure) {
        digitalWrite (yellowLED, HIGH);                  //   Yellow LED ON 8
        for(int i = 0; i < 500; i++){
          digitalWrite (buzzerPin, HIGH);                  // turn the buzzer on (HIGH is the voltage level)
          }
		warning = 1;
    }
    else {
        digitalWrite(yellowLED,  LOW);                   //   Yellow LED OFF
        digitalWrite (buzzerPin, LOW);                   // turn the buzzer OFF (HIGH is the voltage level)
		warning = 0;
    }
	//----------END OF PRESSURE SENSOR-------
	
	//----------STEPPER MOTOR--------    
    long timeOut = 0L;
    long STEPS = 0L;
    long timeIn = 0L;
    bitWrite(periodIn,0,digitalRead(pinInPeriod0));
    bitWrite(periodIn,1,digitalRead(pinInPeriod1));
    bitWrite(periodIn,2,digitalRead(pinInPeriod2));

    bitWrite(periodEx,0,digitalRead(pinExPeriod0));
    bitWrite(periodEx,1,digitalRead(pinExPeriod1));
    bitWrite(periodEx,2,digitalRead(pinExPeriod1));
  
    bitWrite(volume,0, digitalRead(pinVolume0));
    bitWrite(volume,1, digitalRead(pinVolume1));
    bitWrite(volume,2, digitalRead(pinVolume2));

    switch(volume) {
      case B111:{
        STEPS = 250;
        break;
      }
      case B110:{
        STEPS = 350;
        break;
      }
      case B101:{
        STEPS = 450;
        break;
      }
      case B100:{
        STEPS = 550;
        break;
      }
      case B011:{
        STEPS = 600;
        break;
      }
      case B010:{
        STEPS = 650;
        break;
      }
      case B001:{
        STEPS = 700;
        break;
      }
      case B000:{
        STEPS = 750;
        break;
      }
    }

    switch(periodIn){
      case B111:{   //2.00 seconds
        rpmIn = (STEPS/2.00)*(60/200);
		    timeIn = 2;
        break;
      }
      case B110:{   //2.5 seconds
        rpmIn = (STEPS/2.5)*(60/200);
		    timeIn = 2.5;
        break;
      }
      case B101:{   //2.75 seconds
        rpmIn = (STEPS/2.75)*(60/200);
		    timeIn = 2.75;
      }
      case B100:{   //3.00 seconds
        rpmIn = (STEPS/3)*(60/200);
		    timeIn = 3;
        break;
      }
      case B011:{   //3.25 seconds
        rpmIn = (STEPS/3.25)*(60/200);
		    timeIn = 3.25;
        break;
      }
      case B010:{   //3.50 sedonds
        rpmIn = (STEPS/3.5)*(60/200);
		    timeIn = 3.5;
        break;
      }
      case B001:{   //3.75 seconds
        rpmIn = (STEPS/3.75)*(60/200);
		    timeIn = 3.75;
        break;
      }
      case B000:{   //4.00 seconds
        rpmIn = (STEPS/4.00)*(60/200);
		    timeIn = 4;
        break;
      }
    }

    switch(periodEx){
      case B111:{   //4.00 seconds
        rpmOut = -(STEPS/4.00*(60/200);
		timeOut = 4;
        break;
      }
      case B110:{   //4.50 seconds
        rpmOut = -(STEPS/4.5)*(60/200);
		timeOut = 4.5;
        break;
      }
      case B101:{   //4.75 seconds
        rpmOut = -(STEPS/4.75)*(60/200);
		timeOut = 4.75;
        break;
      }
      case B100:{   //5.00 seconds
        rpmOut = -(STEPS/5)*(60/200);
		timeOut = 5;
        break;
      }
      case B011:{   //5.25 seconds
        rpmOut = -(STEPS/5.25)*(60/200);
		timeOut = 5.25;
        break;
      }
      case B010:{   //5.50 sedonds
        rpmOut = -(STEPS/5.5)*(60/200);
		timeOut = 5.5;
        break;
      }
      case B001:{   //5.75 seconds
        rpmOut = -(STEPS/5.75)*(60/200);
		timeOut = 5.75;
        break;
      }
      case B000:{   //6.00 seconds
        rpmOut = -(STEPS/6.00)*(60/200);
		timeOut = 6;
        break;
      }
    }

    if(warning == 0){
    //enable pin is active LOW, set to HIGH by default
    stepper.setEnableActiveState(LOW);
    }
	if(warning == 1){
		digitalWrite (buzzerPin, HIGH); 
	}
 
    //stepper moves STEPS number of 
	//	steps in timeIn and timeOut in microseconds)
    stepper.setRPM(rpmIn);
	valveInServo.write(openPosition);
	valveOutServo.write(closedPosition);
    stepper.move(STEPS);
	delay(timeIn*1000);
	
    stepper.setRPM(rpmOut);
	valveInServo.write(closedPosition);
	valveOutServo.write(openPosition);
    stepper.move(STEPS);
	delay(timeOut*1000);
	
	//------END OF STEPPER MOTOR

  //-------------------------------servo motors----------------------------------------
  if (calculatedPressure > upperBoundPressure 
                   || calculatedPressure < lowerBoundPressure) {
    valveOutServo.write(openPosition); // pressure is out of bounds, open relief valve
  }

  //------------------------------airflow sensor----------------------------------------
  for (int i = 0; i < MeasuresN; i++){
    CurrentReading = analogRead(A0);
    Sum += CurrentReading;
    delay(1);
   }
  Average=Sum/MeasuresN;
  Sum=0;
  Flow=(Average-68)*0.265;   

  if (Flow <= 0) {
    warning = 1;;
  }  
  else if (Flow > 0){
    warning = 0;
  }
 }
