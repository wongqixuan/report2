#include <LiquidCrystal.h>
#include <Wire.h>

// Rotary Encoder connections
#define ENCODER1_DIGITAL 2
#define ENCODER2_DIGITAL 3
int lastState;

int in1 = 10;
int in2 = 12;
int in3 = 11;
int in4 = 13;
int leftSensorPin = A2;
int rightSensorPin = A3;
  

volatile long encoder1Pos = 0;
volatile long encoder2Pos = 0;

float circumferenceOfWheel= 21.36;
unsigned int pulsesPerTurn = 16;
float distancePerPulse = circumferenceOfWheel/pulsesPerTurn; // Adjust this value based on your specific setup
float totalDistance = 0;
float totalDistanceThisLoop=0;
float tempq=0;
float distanceStop;
int distanceFlag=0;
int distanceCount=0;
int resetMPU=0;
int downRamp=0;

//MPU-6050
//Gyro Variables
float elapsedTime, time, timePrev;        //Variables for time control
int gyro_error=0;                         //We use this variable to only calculate once the gyro data error
float Gyr_rawX, Gyr_rawY, Gyr_rawZ;     //Here we store the raw data read 
float Gyro_angle_x, Gyro_angle_y;         //Here we store the angle value obtained with Gyro data
float Gyro_raw_error_x, Gyro_raw_error_y; //Here we store the initial gyro data error

//Acc Variables
int acc_error=0;                         //We use this variable to only calculate once the Acc data error
float rad_to_deg = 180/3.141592654;      //This value is for pasing from radians to degrees values
float Acc_rawX, Acc_rawY, Acc_rawZ;    //Here we store the raw data read 
float Acc_angle_x, Acc_angle_y;          //Here we store the angle value obtained with Acc data
float Acc_angle_error_x, Acc_angle_error_y; //Here we store the initial Acc data error

float Total_angle_x, Total_angle_y;


const int rs = 8, en = 9, d4 = 4, d5 = 5, d6 = 6, d7 = 7;

LiquidCrystal lcd(rs, en, d4, d5, d6, d7);

void updateEncoder(volatile long &encoderPos, int encoderPin)
{
  static int lastState = LOW;
  int currentState = digitalRead(encoderPin);

  if (currentState == HIGH && lastState == LOW)
  {
    encoderPos++;
  }

  lastState = currentState;
}


void setup()
{
  Wire.begin();                           //begin the wire comunication
  
  Wire.beginTransmission(0x68);           //begin, Send the slave adress (in this case 68)              
  Wire.write(0x6B);                       //make the reset (place a 0 into the 6B register)
  Wire.write(0x00);
  Wire.endTransmission(true);             //end the transmission
  //Gyro config
  Wire.beginTransmission(0x68);           //begin, Send the slave adress (in this case 68) 
  Wire.write(0x1B);                       //We want to write to the GYRO_CONFIG register (1B hex)
  Wire.write(0x10);                       //Set the register bits as 00010000 (1000dps full scale)
  Wire.endTransmission(true);             //End the transmission with the gyro
  //Acc config
  Wire.beginTransmission(0x68);           //Start communication with the address found during search.
  Wire.write(0x1C);                       //We want to write to the ACCEL_CONFIG register
  Wire.write(0x10);                       //Set the register bits as 00010000 (+/- 8g full scale range)
  Wire.endTransmission(true); 

  Serial.begin(9600);                     //Remember to set this same baud rate to the serial monitor  
  time = millis();                        //Start counting time in milliseconds


/*Here we calculate the acc data error before we start the loop
 * I make the mean of 200 values, that should be enough*/
  if(acc_error==0)
  {
    for(int a=0; a<200; a++)
    {
      Wire.beginTransmission(0x68);
      Wire.write(0x3B);                       //Ask for the 0x3B register- correspond to AcX
      Wire.endTransmission(false);
      Wire.requestFrom(0x68,6,true); 
      
      Acc_rawX=(Wire.read()<<8|Wire.read())/4096.0 ; //each value needs two registres
      Acc_rawY=(Wire.read()<<8|Wire.read())/4096.0 ;
      Acc_rawZ=(Wire.read()<<8|Wire.read())/4096.0 ;

      
      /*---X---*/
      Acc_angle_error_x = Acc_angle_error_x + ((atan((Acc_rawY)/sqrt(pow((Acc_rawX),2) + pow((Acc_rawZ),2)))*rad_to_deg));
      /*---Y---*/
      Acc_angle_error_y = Acc_angle_error_y + ((atan(-1*(Acc_rawX)/sqrt(pow((Acc_rawY),2) + pow((Acc_rawZ),2)))*rad_to_deg)); 
      
      if(a==199)
      {
        Acc_angle_error_x = Acc_angle_error_x/200;
        Acc_angle_error_y = Acc_angle_error_y/200;
        acc_error=1;
      }
    }
  }//end of acc error calculation   


/*Here we calculate the gyro data error before we start the loop
 * I make the mean of 200 values, that should be enough*/
  if(gyro_error==0)
  {
    for(int i=0; i<200; i++)
    {
      Wire.beginTransmission(0x68);            //begin, Send the slave adress (in this case 68) 
      Wire.write(0x43);                        //First adress of the Gyro data
      Wire.endTransmission(false);
      Wire.requestFrom(0x68,4,true);           //We ask for just 4 registers 
         
      Gyr_rawX=Wire.read()<<8|Wire.read();     //Once again we shif and sum
      Gyr_rawY=Wire.read()<<8|Wire.read();
   
      /*---X---*/
      Gyro_raw_error_x = Gyro_raw_error_x + (Gyr_rawX/32.8); 
      /*---Y---*/
      Gyro_raw_error_y = Gyro_raw_error_y + (Gyr_rawY/32.8);
      if(i==199)
      {
        Gyro_raw_error_x = Gyro_raw_error_x/200;
        Gyro_raw_error_y = Gyro_raw_error_y/200;
        gyro_error=1;
      }
    }//end of gyro error calculation  
  } //end of setup void


  
  pinMode(ENCODER1_DIGITAL, INPUT);
  pinMode(ENCODER2_DIGITAL, INPUT);
  attachInterrupt(digitalPinToInterrupt(ENCODER1_DIGITAL), [] { updateEncoder(encoder1Pos, ENCODER1_DIGITAL); }, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER2_DIGITAL), [] { updateEncoder(encoder2Pos, ENCODER2_DIGITAL); }, CHANGE);
  
  pinMode(leftSensorPin, INPUT);
  pinMode(rightSensorPin, INPUT);

  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);

  lcd.begin(16, 2);
}

void loop()
{
  timePrev = time;                        // the previous time is stored before the actual time read
  time = millis();                        // actual time read
  elapsedTime = (time - timePrev) / 1000; //divide by 1000 in order to obtain seconds

  //////////////////////////////////////Gyro read/////////////////////////////////////

    Wire.beginTransmission(0x68);            //begin, Send the slave adress (in this case 68) 
    Wire.write(0x43);                        //First adress of the Gyro data
    Wire.endTransmission(false);
    Wire.requestFrom(0x68,4,true);           //We ask for just 4 registers
        
    Gyr_rawX=Wire.read()<<8|Wire.read();     //Once again we shif and sum
    Gyr_rawY=Wire.read()<<8|Wire.read();
    /*Now in order to obtain the gyro data in degrees/seconds we have to divide first
    the raw value by 32.8 because that's the value that the datasheet gives us for a 1000dps range*/
    /*---X---*/
    Gyr_rawX = (Gyr_rawX/32.8) - Gyro_raw_error_x; 
    /*---Y---*/
    Gyr_rawY = (Gyr_rawY/32.8) - Gyro_raw_error_y;
    
    /*Now we integrate the raw value in degrees per seconds in order to obtain the angle
    * If you multiply degrees/seconds by seconds you obtain degrees */
    /*---X---*/
    Gyro_angle_x = Gyr_rawX*elapsedTime;
    /*---X---*/
    Gyro_angle_y = Gyr_rawY*elapsedTime;


    
  
  //////////////////////////////////////Acc read/////////////////////////////////////

  Wire.beginTransmission(0x68);     //begin, Send the slave adress (in this case 68) 
  Wire.write(0x3B);                 //Ask for the 0x3B register- correspond to AcX
  Wire.endTransmission(false);      //keep the transmission and next
  Wire.requestFrom(0x68,6,true);    //We ask for next 6 registers starting withj the 3B  
  /*We have asked for the 0x3B register. The IMU will send a brust of register.
  * The amount of register to read is specify in the requestFrom function.
  * In this case we request 6 registers. Each value of acceleration is made out of
  * two 8bits registers, low values and high values. For that we request the 6 of them  
  * and just make then sum of each pair. For that we shift to the left the high values 
  * register (<<) and make an or (|) operation to add the low values.
  If we read the datasheet, for a range of+-8g, we have to divide the raw values by 4096*/    
  Acc_rawX=(Wire.read()<<8|Wire.read())/4096.0 ; //each value needs two registres
  Acc_rawY=(Wire.read()<<8|Wire.read())/4096.0 ;
  Acc_rawZ=(Wire.read()<<8|Wire.read())/4096.0 ; 
  /*Now in order to obtain the Acc angles we use euler formula with acceleration values
   after that we substract the error value found before*/  
   /*---X---*/
  Acc_angle_x = (atan((Acc_rawY)/sqrt(pow((Acc_rawX),2) + pow((Acc_rawZ),2)))*rad_to_deg) - Acc_angle_error_x;
   /*---Y---*/
  Acc_angle_y = (atan(-1*(Acc_rawX)/sqrt(pow((Acc_rawY),2) + pow((Acc_rawZ),2)))*rad_to_deg) - Acc_angle_error_y;    


  //////////////////////////////////////Total angle and filter/////////////////////////////////////
   /*---X axis angle---*/
   Total_angle_x = 0.98 *(Total_angle_x + Gyro_angle_x) + 0.02*Acc_angle_x;
   /*---Y axis angle---*/
   Total_angle_y = 0.98 *(Total_angle_y + Gyro_angle_y) + 0.02*Acc_angle_y;
   

 
 
  /*Uncoment the rest of the serial prines
   * I only print the Y angle value for this test */
  Serial.print("Xº: ");
  Serial.print(Total_angle_x);
  Serial.print("   |   ");
  //Serial.print("Yº: ");
  //Serial.print(Total_angle_y);
  //Serial.print("   |   ");
  Serial.print("Gyr_rawX: ");
  Serial.print(Gyr_rawX);
  Serial.print("   |   ");
  //Serial.print("Time: ");
  //Serial.print(time);
  //Serial.print("   |   ");
  
  
  /*lcd.setCursor(0,1);
  lcd.print("Time: ");
  lcd.setCursor(8, 1);
  lcd.print(time/1000);*/

  int leftSensorValue = digitalRead(leftSensorPin);
  int rightSensorValue = digitalRead(rightSensorPin);

  float distance1 = encoder1Pos * distancePerPulse;
  float distance2 = encoder2Pos * distancePerPulse;
  totalDistanceThisLoop = (distance1 + distance2) / 200;

  lcd.setCursor(0,1);
  lcd.print("Angle: ");
  lcd.setCursor(11,1);
  lcd.print(Total_angle_x, 2);

  if(Total_angle_x>3){
    analogWrite(in1, 255);
    analogWrite(in2, 0);
    analogWrite(in3, 255);
    analogWrite(in4, 0);
    delay(1);
      if(Gyr_rawX <-40){
      analogWrite(in1, 255);
      analogWrite(in2, 0);
      analogWrite(in3, 255);
      analogWrite(in4, 0);
      delay(260);
        analogWrite(in1, 0);
        analogWrite(in2, 0);
        analogWrite(in3, 0);
        analogWrite(in4, 0);
        delay(4000);
        analogWrite(in1, 255);
        analogWrite(in2, 0);
        analogWrite(in3, 0);
        analogWrite(in4, 255);
        delay(1640);
          analogWrite(in1, 0);
          analogWrite(in2, 0);
          analogWrite(in3, 0);
          analogWrite(in4, 0);
          delay(3000);
          
          resetMPU=1;
          distanceFlag=1;
          downRamp=1;
      }
    }
  

  if (resetMPU==1){
    Total_angle_x=0;
    
  }
if(distanceFlag==1){
  distanceFlag=0;
  distanceCount=1;
  
}
if(downRamp==1){
  if(Gyr_rawX>40){
    tempq=totalDistanceThisLoop;
    downRamp=2;
  }
}

  distanceStop=totalDistanceThisLoop - tempq;
  // Calculate and display distance only when both front wheels are moving forward
  if (leftSensorValue == HIGH && rightSensorValue == HIGH && Total_angle_x>-50 && Total_angle_x<3)
  {
    displayDistance(11, 0, totalDistanceThisLoop);
    

    // Motor control
    analogWrite(in1, 70);
    analogWrite(in2, 0);
    analogWrite(in3, 70);
    analogWrite(in4, 0);
      if(distanceStop >= 0.3 && distanceCount==1 && downRamp==2){
        analogWrite(in1, 0);
        analogWrite(in2, 0);
        analogWrite(in3, 0);
        analogWrite(in4, 0);
        delay(2000);
        distanceCount=0;
        downRamp=0;
      }

  }else if (leftSensorValue == LOW && rightSensorValue ==HIGH) {
      analogWrite(in1,0);
      analogWrite(in2,180);
      analogWrite(in3,140);
      analogWrite(in4,0);

  }else if (leftSensorValue == HIGH && rightSensorValue == LOW) {
      analogWrite(in1,140);
      analogWrite(in2,0);
      analogWrite(in3,0);
      analogWrite(in4,180);
  }
  else if (leftSensorValue == LOW && rightSensorValue == LOW){
    // If the vehicle is turning or not on the line, stop the motors
    analogWrite(in1, 0);
    analogWrite(in2, 0);
    analogWrite(in3, 0);
    analogWrite(in4, 0);
  }
  Serial.print("tempq: ");
  Serial.print(tempq);
  Serial.print("   |   ");
  Serial.print("distancestop: ");
  Serial.print(distanceStop);
  Serial.print("   |   ");
  Serial.print("totalDistanceThisLoop: ");
  Serial.print(totalDistanceThisLoop);
  Serial.print("   |   ");
  Serial.print("distanceCount: ");
  Serial.print(distanceCount);
  Serial.println(" ");
}

void displayDistance(int col, int row, float distance){
 // Ensure enough spaces to clear previous content
  lcd.setCursor(0,0);
  lcd.print("Distance: ");
  lcd.setCursor(col, row);
  lcd.print(distance, 2);
}
