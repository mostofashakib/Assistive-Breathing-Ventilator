/*
 * Written by: Mostofa Adib Shakib
 * Project Lab 1 - Assitive Ventilator Project
 * Description: Pressure monitor & safety alarm arduino code
 * 
*/ 
 

#include <LiquidCrystal.h>                      //  Load Liquid Crystal Library

#define ONE_WIRE_BUS 6                          //  Data wire is plugged into pin 6 on the Arduino 

LiquidCrystal LCD(12, 11, 5, 4, 3, 2);          //  Create Liquid Crystal Object called LCD

int trigPin = 9;                                //  Sensor Trigger pin connected to Arduino pin 9
int echoPin = 7;                                //  Sensor Echo pin connected to Arduino pin 7
int myCounter = 0;                              //  Declare your variable myCounter and set to 0

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

void setup() {

  Serial.begin(9600);
  pinMode(trigPin, OUTPUT);                     //  Set Sensor Trigger pin as an output
  pinMode(echoPin, INPUT);                      //  Set Sensor Echo pin as an input
  LCD.begin(16, 2);                             //  Start the 16x2 LCD Liquid Crystal display
  LCD.setCursor(0, 0);                          //  Set LCD cursor to upper left corner, column 0, row 0
  LCD.print("Distance:");                       //  Print Message on First Row
  pinMode(8, OUTPUT);                           //  Output pin 8, 10, 13
  pinMode(10, OUTPUT);
  pinMode(13, OUTPUT);
  pinMode(6, OUTPUT);                           //  Set buzzer - pin 6 as an output
}

void loop() {

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
        digitalWrite (10, HIGH);                 //   Red LED ON 10
        digitalWrite (6, HIGH);                  // turn the buzzer on (HIGH is the voltage level)
    }
    else {
        digitalWrite(10, LOW);                   //    Red LED OFF
        digitalWrite (6, LOW);                  // turn the buzzer OFF (HIGH is the voltage level)
    }

    if (calculatedPressure < upperBoundPressure && calculatedPressure > lowerBoundPressure) {
        digitalWrite (13, HIGH);                  //    Green LED ON 13
    }
    else {
        digitalWrite(13, LOW);                    //    Green LED OFF
    }
    
    if (calculatedPressure < lowerBoundPressure) {
        digitalWrite (8, HIGH);                  //   Yellow LED ON 8
        digitalWrite (6, HIGH);                  // turn the buzzer on (HIGH is the voltage level)
    }
    else {
        digitalWrite(8,  LOW);                   //   Yellow LED OFF
        digitalWrite (6, LOW);                   // turn the buzzer OFF (HIGH is the voltage level)
    }
 }
