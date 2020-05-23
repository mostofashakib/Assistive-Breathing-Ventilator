#include <Wire.h>  
#include <LiquidCrystal_I2C.h> 
LiquidCrystal_I2C lcd(0x27, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE); 

float CurrentReading;
float Sum;
float Average;
float MeasuresN=2500 ; // number of measures to average
float Flow;

void setup() 
{
Serial.begin(9600);
lcd.begin(16,2); 
}
void loop() 
{
for (int i = 0; i < MeasuresN; i++){
CurrentReading = analogRead(A0);
Sum += CurrentReading;
delay(1);
}
Average=Sum/MeasuresN;
Sum=0;
Flow=(Average-68)*0.265;   

if (Flow <= 0) {
  // set off alarm;
}  
delay(5);
}
