



#include <SD.h>
#include <SPI.h>
#include "SBUS.h"


const int chipSelect = 4;


// SBUS setup

bool isSbusAvailable = 0;
uint16_t channels[16];
bool failSafe;
bool lostFrame;
SBUS x8r(Serial2);


// Lidar Setup
#include <Wire.h>
#include <VL53L0X.h>

VL53L0X sensor;
bool isLidarAvailable = 0;
int range;

//Sonar setup
int sonarPin = 14;


void setup() {


  Serial.begin(115200);
  delay(100);
  Serial.println("Serial coms init");

  Serial.print("Initializing SBUS.....");
  x8r.begin();
  delay(10);
  Serial.println("X");


  Serial.print("Initializing Lidar....");
  Wire.begin();
  sensor.setTimeout(500);
  if (!sensor.init())
  {
    Serial.println("Failed to detect and initialize sensor!");
    while (1) {}
  }
  sensor.startContinuous();
  Serial.println("x");

  Serial.print("Initializing Lidar....");
  

}

void loop() {
  // put your main code here, to run repeatedly:
  int lastTime = micros();
  
  readSBUS();
  readLidar();

  Serial.print(micros()-lastTime);
  Serial.print(",");
  Serial.print(channels[0]);
  Serial.print(",");
  Serial.println(range);

  delay(100);

}

void readSBUS() {
  if(x8r.read(&channels[0], &failSafe, &lostFrame)){
    isSbusAvailable = 1;
    // write the SBUS packet to an SBUS compatible servo
    x8r.write(channels);
  }
}

void readLidar() {
  if(sensor.available())
  {
       range = sensor.readRangeMillimeters();
       isLidarAvailable = 1;
  }
}
