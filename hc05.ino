#include <LiquidCrystal.h>
#include <SoftwareSerial.h>
#include "pitches.h"


int in1 = 10;
int in2 = 12;
int in3 = 11;
int in4 = 13;
int buzzer=A2;


const int rs = 8, en = 9, d4 = 4, d5 = 5, d6 = 6, d7 = 7;

LiquidCrystal lcd(rs, en, d4, d5, d6, d7);


// Create software serial object to communicate with HC-05
SoftwareSerial mySerial(3,2); //HC-05 Tx & Rx is connected to Arduino #3 & #2
char data;


void setup() {
  pinMode(in1 ,OUTPUT);
  pinMode(in2 ,OUTPUT);
  pinMode(in3 ,OUTPUT);
  pinMode(in4 ,OUTPUT);
  pinMode(buzzer ,OUTPUT);
  
  lcd.begin(16,2);
  Serial.begin(9600);
  //Begin serial communication with Arduino and HC-05
  mySerial.begin(9600);
  
  Serial.println("Initializing...");
  Serial.println("The device started, now you can pair it with bluetooth!");
  lcd.setCursor(0,0);
  lcd.print("Proton Miewmiew");
}

void loop() {
  
  if(Serial.available()) 
  {
    mySerial.write(Serial.read());//Forward what Serial received to Software Serial Port
  }
  if(mySerial.available()) 
  {
    Serial.write(mySerial.read());//Forward what Software Serial received to Serial Port
  }
  delay(20);
  if(mySerial.available()>0){
    data=mySerial.read();
    Serial.println(data);

  }
  digitalWrite(buzzer,LOW);
  
  switch (data){

    case 'F':
      forward();
      break;

    case 'B':
      backward();
      break;
    
    case 'S':
      stop();
      break;

    case 'L':
      left();
      break;
    
    case 'R':
      right();
      break;

  
  }
  

}

void forward(){
  analogWrite(in1,200); 
  analogWrite(in2,0); 
  analogWrite(in3,200); 
  analogWrite(in4,0); 
}
void backward(){
  digitalWrite(buzzer,HIGH);
  analogWrite(in1,0); 
  analogWrite(in2,150); 
  analogWrite(in3,0); 
  analogWrite(in4,150);
  
}
void left(){
  analogWrite(in1,0); 
  analogWrite(in2,200); 
  analogWrite(in3,200); 
  analogWrite(in4,0); 
}
void right(){
  analogWrite(in1,200); 
  analogWrite(in2,0); 
  analogWrite(in3,0); 
  analogWrite(in4,200); 
}
void stop(){
  analogWrite(in1,0); 
  analogWrite(in2,0); 
  analogWrite(in3,0); 
  analogWrite(in4,0);
}

 


