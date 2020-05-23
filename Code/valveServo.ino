#include <Servo.h>
#include <PressureMonitor.h>

Servo valveServo;       // create servo object to control a servo
PressureMonitor pin(7); // creates PressureMonitor Object and declares input pin as pin 7

int openPosition = 0;     // variable to store the servo position to open the valve
int closedPosition = 180; // variable to store the servo position to close the valve

void setup() {
  Serial.begin(9600);
  valveServo.attach(45);  // attaches the servo on pin 45 to the servo object
}                         // pin 45 can produce PWM output

void loop() {
  if (calculatedPressure > upperBoundPressure 
                   || calculatedPressure < lowerBoundPressure) {
    valveServo.write(closePosition)); // close the valve or remain closed
                                      // as pressure is too high or too low
  }

  if (calculatedPressure < upperBoundPressure
                    && calculatedPressure > lowerBoundPressure) {
    valveServo.write(openPosition)); // open valve or remain open as pressure is normal
  }
  delay(2000); //let the pressure monitor refresh
}
