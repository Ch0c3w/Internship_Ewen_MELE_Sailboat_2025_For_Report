#include "IMU.h"

float sawtooth(float x){ // sawtooth function from Luc Jaulin's roblib.py librairy. Converts any radian angle value to it's equivalent value between -pi nd pi.
  return 2*atan(tan(x/2));
}

IMU::IMU()
{
  Serial.println("Initialising IMU...");
  cap = 0.0;
  CMPS12_SERIAL.begin(9600); // Start the communication on the Serial port associated with the IMU in the config header
  calibrate();
}

void IMU::update() // Update the heading value using IMU data.
{
  CMPS12_SERIAL.write(ANGLE_16);  // Request 16 bit angle
  while(CMPS12_SERIAL.available() < 2); // Wait for full angle (16 bits = 2 bytes)
  unsigned char high_byte = CMPS12_SERIAL.read(); // read first byte
  unsigned char low_byte = CMPS12_SERIAL.read(); // reads seconf byte
  unsigned int angle16 = high_byte;           // Compute 16 bit angle using the formula given in IMU documentation and usage example.
  angle16 <<= 8;
  angle16 += low_byte;
  cap = -sawtooth((angle16 / 10 + (float)(angle16%10)/10 - 180) * PI / 180) * 180 / PI; // heading = 0 if the boat is heading north, angle are given following the counterclockwise direction to facilitate computing.
  // Serial.print("Heading:");
  // Serial.println(cap);
  return;           
}

float IMU::get_heading(){ // Heading getter
  //Serial.print("Current heading:");
  //Serial.println(cap);
  return cap; 
}

bool IMU::calibrate(){
  // Code made by Titouan Leost : https://github.com/TitouanLeost/Aston-Autonomous-Sailboat-2024
  // Erasing calibration data stored in the IMU:
  Serial.println("Calibration started...");
  CMPS12_SERIAL.write(0xE0);
  while(CMPS12_SERIAL.available() < 1);
  CMPS12_SERIAL.read();
  CMPS12_SERIAL.write(0xE5);
  while(CMPS12_SERIAL.available() < 1);
  CMPS12_SERIAL.read();
  CMPS12_SERIAL.write(0xE2);
  while(CMPS12_SERIAL.available() < 1);
  CMPS12_SERIAL.read();
  Serial.println("Checking calibration status...");
  CMPS12_SERIAL.write(CMPS12_CALIBRATION_STATUS);
  while(CMPS12_SERIAL.available() < 1);
  unsigned char status = CMPS12_SERIAL.read();
  int count = 0;
  while(count < 50 or status!= 0b11111111) { // The code awaits for the IMU to be fully calibrated on at least 50 measurements before saving calibration data.
    CMPS12_SERIAL.write(CMPS12_CALIBRATION_STATUS);
    while(CMPS12_SERIAL.available() < 1);
    status = CMPS12_SERIAL.read();
    Serial.println(int(status), BIN);
    if(status == 0b11111111){
        count += 1;
    }
    delay(10);
  }
  // Saving calibration data on the IMU : 
  CMPS12_SERIAL.write(0xF0);
  while(CMPS12_SERIAL.available() < 1);
  CMPS12_SERIAL.read();
  CMPS12_SERIAL.write(0xF5);
  while(CMPS12_SERIAL.available() < 1);
  CMPS12_SERIAL.read();
  CMPS12_SERIAL.write(0xF6);
  while(CMPS12_SERIAL.available() < 1);
  CMPS12_SERIAL.read();
  Serial.println("Calibration done");
  return true;
}

