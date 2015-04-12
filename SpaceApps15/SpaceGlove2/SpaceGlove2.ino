//** SOFTWARE SERIAL FOR BLUETOOTH **//

#include <SoftwareSerial.h>
int rxPin = 3;
int txPin = 4;
SoftwareSerial bluetooth(rxPin, txPin);

//** ACCELEROMETER **//

#include <Wire.h>
#include <ADXL345.h>


ADXL345 adxl; //variable adxl is an instance of the ADXL345 library

//******************//

// **GYROSCOPE **//

#define CTRL_REG1 0x20
#define CTRL_REG2 0x21
#define CTRL_REG3 0x22
#define CTRL_REG4 0x23
#define CTRL_REG5 0x24

int L3G4200D_Address = 105; //I2C address of the L3G4200D

int gyro_x;
int gyro_y;
int gyro_z;

int thumb = 0;
int index = 0;

//***************//

void setup(){
  
  pinMode(13, OUTPUT);
  
  Serial.begin(9600);
  bluetooth.begin(9600);
  
  adxl.powerOn();

  //set activity/ inactivity thresholds (0-255)
  adxl.setActivityThreshold(75); //62.5mg per increment
  adxl.setInactivityThreshold(75); //62.5mg per increment
  adxl.setTimeInactivity(10); // how many seconds of no activity is inactive?
 
  //look of activity movement on this axes - 1 == on; 0 == off 
  adxl.setActivityX(1);
  adxl.setActivityY(1);
  adxl.setActivityZ(1);
 
  //look of inactivity movement on this axes - 1 == on; 0 == off
  adxl.setInactivityX(1);
  adxl.setInactivityY(1);
  adxl.setInactivityZ(1);
 
  //look of tap movement on this axes - 1 == on; 0 == off
  adxl.setTapDetectionOnX(0);
  adxl.setTapDetectionOnY(0);
  adxl.setTapDetectionOnZ(1);
 
  //set values for what is a tap, and what is a double tap (0-255)
  adxl.setTapThreshold(50); //62.5mg per increment
  adxl.setTapDuration(15); //625μs per increment
  adxl.setDoubleTapLatency(80); //1.25ms per increment
  adxl.setDoubleTapWindow(200); //1.25ms per increment
 
  //set values for what is considered freefall (0-255)
  adxl.setFreeFallThreshold(7); //(5 - 9) recommended - 62.5mg per increment
  adxl.setFreeFallDuration(45); //(20 - 70) recommended - 5ms per increment
 
  //setting all interupts to take place on int pin 1
  //I had issues with int pin 2, was unable to reset it
  adxl.setInterruptMapping( ADXL345_INT_SINGLE_TAP_BIT,   ADXL345_INT1_PIN );
  adxl.setInterruptMapping( ADXL345_INT_DOUBLE_TAP_BIT,   ADXL345_INT1_PIN );
  adxl.setInterruptMapping( ADXL345_INT_FREE_FALL_BIT,    ADXL345_INT1_PIN );
  adxl.setInterruptMapping( ADXL345_INT_ACTIVITY_BIT,     ADXL345_INT1_PIN );
  adxl.setInterruptMapping( ADXL345_INT_INACTIVITY_BIT,   ADXL345_INT1_PIN );
 
  //register interupt actions - 1 == on; 0 == off  
  adxl.setInterrupt( ADXL345_INT_SINGLE_TAP_BIT, 1);
  adxl.setInterrupt( ADXL345_INT_DOUBLE_TAP_BIT, 1);
  adxl.setInterrupt( ADXL345_INT_FREE_FALL_BIT,  1);
  adxl.setInterrupt( ADXL345_INT_ACTIVITY_BIT,   1);
  adxl.setInterrupt( ADXL345_INT_INACTIVITY_BIT, 1);
  
  //*****************************************//
  
  setupL3G4200D(2000); // Configure L3G4200  - 250, 500 or 2000 deg/sec

  delay(1500); //wait for the sensor to be ready 
  
  
}

int t = 500;

void loop()
{
  //**** ACCELEROMETER CODE ****//
  
  //Boring accelerometer stuff   
  int x,y,z;  
  adxl.readAccel(&x, &y, &z); //read the accelerometer values and store them in variables  x,y,z

  // Output x,y,z values - Commented out
  /*Serial.print(x);
  Serial.print(" ");
  Serial.print(y);
  Serial.print(" ");
  Serial.println(z);*/
  /*
  bluetooth.print(x);
  bluetooth.print(" ");
  bluetooth.print(y);
  bluetooth.print(" ");
  bluetooth.print(z);
  bluetooth.print(" ");
  */
  getGyroValues();
  if(x > 100){
    bluetooth.println("FW");
    digitalWrite(13, HIGH); delay(t); digitalWrite(13, LOW);
  }
  else if(x < -100){
    bluetooth.println("BW");
    digitalWrite(13, HIGH); delay(t); digitalWrite(13, LOW);
  }
  else if(y > 150){
    bluetooth.println("SX");
    digitalWrite(13, HIGH); delay(t); digitalWrite(13, LOW);
  }
  else if(y < -100){
    bluetooth.println("DX");
    digitalWrite(13, HIGH); delay(t); digitalWrite(13, LOW);
  }
  else if(z < 60){
    bluetooth.println("DW");
    digitalWrite(13, HIGH); delay(t); digitalWrite(13, LOW);
  }
  else if(z > 300){
    bluetooth.println("UP");
    digitalWrite(13, HIGH); delay(t); digitalWrite(13, LOW);
  }
  
  //Fun Stuff!    
  //read interrupts source and look for triggerd actions
  
  //getInterruptSource clears all triggered actions after returning value
  //so do not call again until you need to recheck for triggered actions
//   byte interrupts = adxl.getInterruptSource();
//  
//  // freefall
//  if(adxl.triggered(interrupts, ADXL345_FREE_FALL)){
//    Serial.println("freefall");
//    //add code here to do when freefall is sensed
//  } 
//  
//  //inactivity
//  if(adxl.triggered(interrupts, ADXL345_INACTIVITY)){
//    Serial.println("inactivity");
//     //add code here to do when inactivity is sensed
//  }
//  
//  //activity
//  if(adxl.triggered(interrupts, ADXL345_ACTIVITY)){
//    Serial.println("activity"); 
//     //add code here to do when activity is sensed
//  }
//  
//  //double tap
//  if(adxl.triggered(interrupts, ADXL345_DOUBLE_TAP)){
//    Serial.println("double tap");
//     //add code here to do when a 2X tap is sensed
//  }
//  
//  //tap
//  if(adxl.triggered(interrupts, ADXL345_SINGLE_TAP)){
//    Serial.println("tap");
//     //add code here to do when a tap is sensed
//  } 

   // *************************//
   
     // This will update x, y, and z with new values

  //Serial.print("X:");
 /* Serial.print(gyro_x);
  Serial.print(" ");

  //Serial.print(" Y:");
  Serial.print(gyro_y);
  Serial.print(" ");

  //Serial.print(" Z:");
  Serial.println(gyro_z);*/
  /*
  bluetooth.print(gyro_x);
  bluetooth.print(" ");

  //Serial.print(" Y:");
  bluetooth.print(gyro_y);
  bluetooth.print(" ");
  bluetooth.print(gyro_z);
  bluetooth.print(" ");
  bluetooth.print(analogRead(A6));
  bluetooth.print(" ");
  bluetooth.println(analogRead(A7));
  delay(50); //Just here to slow down the serial to make it more readable
  */   
  
  else if(gyro_x > 4000){
    bluetooth.println("RR");
    digitalWrite(13, HIGH); delay(t); digitalWrite(13, LOW);
  }
  else if(gyro_x < -4000){
    bluetooth.println("RL");
    digitalWrite(13, HIGH); delay(t); digitalWrite(13, LOW);
  }
  
  else if(analogRead(A7) > 910){
    bluetooth.println("ID");
    digitalWrite(13, HIGH); delay(t); digitalWrite(13, LOW);
  }
  else if(analogRead(A6) > 910){
    bluetooth.println("TH");
    digitalWrite(13, HIGH); delay(t); digitalWrite(13, LOW);
  }
}

void getGyroValues(){

  byte xMSB = readRegister(L3G4200D_Address, 0x29);
  byte xLSB = readRegister(L3G4200D_Address, 0x28);
  gyro_x = ((xMSB << 8) | xLSB);

  byte yMSB = readRegister(L3G4200D_Address, 0x2B);
  byte yLSB = readRegister(L3G4200D_Address, 0x2A);
  gyro_y = ((yMSB << 8) | yLSB);

  byte zMSB = readRegister(L3G4200D_Address, 0x2D);
  byte zLSB = readRegister(L3G4200D_Address, 0x2C);
  gyro_z = ((zMSB << 8) | zLSB);
}

int setupL3G4200D(int scale){
  //From  Jim Lindblom of Sparkfun's code

  // Enable x, y, z and turn off power down:
  writeRegister(L3G4200D_Address, CTRL_REG1, 0b00001111);

  // If you'd like to adjust/use the HPF, you can edit the line below to configure CTRL_REG2:
  writeRegister(L3G4200D_Address, CTRL_REG2, 0b00000000);

  // Configure CTRL_REG3 to generate data ready interrupt on INT2
  // No interrupts used on INT1, if you'd like to configure INT1
  // or INT2 otherwise, consult the datasheet:
  writeRegister(L3G4200D_Address, CTRL_REG3, 0b00001000);

  // CTRL_REG4 controls the full-scale range, among other things:

  if(scale == 250){
    writeRegister(L3G4200D_Address, CTRL_REG4, 0b00000000);
  }else if(scale == 500){
    writeRegister(L3G4200D_Address, CTRL_REG4, 0b00010000);
  }else{
    writeRegister(L3G4200D_Address, CTRL_REG4, 0b00110000);
  }

  // CTRL_REG5 controls high-pass filtering of outputs, use it
  // if you'd like:
  writeRegister(L3G4200D_Address, CTRL_REG5, 0b00000000);
}

void writeRegister(int deviceAddress, byte address, byte val) {
    Wire.beginTransmission(deviceAddress); // start transmission to device 
    Wire.write(address);       // send register address
    Wire.write(val);         // send value to write
    Wire.endTransmission();     // end transmission
}

int readRegister(int deviceAddress, byte address){

    int v;
    Wire.beginTransmission(deviceAddress);
    Wire.write(address); // register to read
    Wire.endTransmission();

    Wire.requestFrom(deviceAddress, 1); // read a byte

    while(!Wire.available()) {
        // waiting
    }

    v = Wire.read();
    return v;
}
