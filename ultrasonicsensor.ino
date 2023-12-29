// Include NewPing Library
#include "NewPing.h"
#include <LiquidCrystal.h>

// Hook up HC-SR04 with Trig to Arduino Pin 9, Echo to Arduino pin 10
#define TRIGGER_PIN A0
#define ECHO_PIN A1

// Maximum distance we want to ping for (in centimeters).
#define MAX_DISTANCE 400	

// NewPing setup of pins and maximum distance.
NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE);

int in1 = 10;
int in2 = 12;
int in3 = 11;
int in4 = 13;

const int rs=8 , en=9 , d4=4 , d5=5 , d6=6 , d7=7 ;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);

void setup() {
	Serial.begin(9600);
  
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);

  lcd.begin(16, 2);
}

void loop() {
	Serial.print("Distance = ");
	Serial.print(sonar.ping_cm());
	Serial.println(" cm");
	delay(500);

  lcd.setCursor(0,0);
  lcd.print("Distance: ");
  lcd.setCursor(11,0);
  lcd.print(sonar.ping_cm());

  if(sonar.ping_cm() <= 10) {
    analogWrite(in1, 0);
    analogWrite(in2, 0);
    analogWrite(in3, 0);
    analogWrite(in4, 0);
    /*delay(500);
    analogWrite(in1, 150);
    analogWrite(in2, 0);
    analogWrite(in3, 0);
    analogWrite(in4, 180);
    delay(300);*/

  }else {
    analogWrite(in1, 80);
    analogWrite(in2, 0);
    analogWrite(in3, 80);
    analogWrite(in4, 0);
  }
}
