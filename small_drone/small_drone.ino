



#include <SD.h>
#include <SPI.h>
#include "SBUS.h"
#include "TeensyThreads.h"

const int chipSelect = 4;


// SBUS setup
bool isSbusAvailable = 0;
uint16_t channels[16];
bool failSafe;
bool lostFrame;
SBUS x8r(Serial2);
bool armed = true;


// Lidar Setup
#include <Wire.h>
#include <VL53L0X.h>

VL53L0X sensor;
bool isLidarAvailable = 0;
int range;

//Sonar setup
int sonarPin = 14;
int sonarValue;
bool isSonarAvailable = 0;

//GPS Setup
#include "SparkFun_Ublox_Arduino_Library.h" //http://librarymanager/All#SparkFun_u-blox_GNSS
SFE_UBLOX_GPS myGPS;
#include <SoftwareSerial.h>
long lastTime = 0; //Simple local timer. Limits amount of I2C traffic to Ublox module.
byte minSat = 0;
long latitude;
long longitude;
long altitude;
byte SIV;
bool isGPSAvailable = 0;

//IMU Setup
float quats_f[4],rates_f[3],acc_f[3];
byte in[100];
union {unsigned short s; byte b[2];} checksum;
union {float f; char c[4];} tmp;
bool isIMUAvailable = 0;

//blackbox setup
//char dataString[150];

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


  Serial.print("Initializing blackbox...");
  Serial8.begin(115200);
  delay(100);
  //while(!Serial8) {};
  Serial.println("X");
  
  Serial.println("Initializing Sonar....X");

  Serial.print("Initializing GPS....");
  do {
    //Serial.println("GPS: trying 115200 baud");
    Serial5.begin(115200);
    if (myGPS.begin(Serial5) == true) {
        //Serial.println("GPS: connected at 115200 baud!");
        delay(100);
        break;
    } else {
        //myGPS.factoryReset();
        delay(2000); //Wait a bit before trying again to limit the Serial output
    }

    //Serial.println("GPS: trying 38400 baud");
    Serial5.begin(38400);
    if (myGPS.begin(Serial5) == true) 
    {
        //Serial.println("GPS: connected at 38400 baud, switching to 115200");
        myGPS.setSerialRate(115200);
        delay(100);
        break;
    }

    delay(100);
    //Serial.println("GPS: trying 9600 baud");
    Serial5.begin(9600);
    if (myGPS.begin(Serial5) == true) {
        //Serial.println("GPS: connected at 9600 baud, switching to 115200");
        myGPS.setSerialRate(115200);
        delay(100);
    }

    delay(100);
    
  } while(1);

  myGPS.setUART1Output(COM_TYPE_UBX); //Set the UART port to output UBX only
  myGPS.setI2COutput(COM_TYPE_UBX); //Set the I2C port to output UBX only (turn off NMEA noise)
  myGPS.saveConfiguration(); //Save the current settings to flash and BBR

  myGPS.setUART1Output(COM_TYPE_UBX); //Set the UART port to output UBX only
  myGPS.setI2COutput(COM_TYPE_UBX); //Set the I2C port to output UBX only (turn off NMEA noise)

  myGPS.setNavigationFrequency(19);           //Set output to 10 times a second
  byte rate = myGPS.getNavigationFrequency(); //Get the update rate of this module
  Serial.print("Current update rate:");
  Serial.println(rate);
  
  myGPS.saveConfiguration(); //Save the current settings to flash and BBR
  
  Serial.println("X");

  Serial.println("Searching for satelights");

  while(1) {
    if(myGPS.getPVT()) {
      SIV = myGPS.getSIV();
      if(SIV >= minSat) {
        break;
      }
      else {
        Serial.print(",");
        Serial.print(SIV);
      }
    }
  }
  initIMU();
  
  Serial.println("attaching threads");

//  threads.addThread(readSBUS_thread);
//  threads.addThread(readLidar_thread);
//  threads.addThread(readSonar_thread);
//  threads.addThread(readGPS_thread);
  threads.addThread(readIMU_thread);
}

void loop() {
  // put your main code here, to run repeatedly:
  int lastTime = micros();


  //these will go into threads 
//  readSBUS();
//  isSbusAvailable = 0;
//  readLidar();
//  isLidarAvailable = 0;
//  readSonar();
//  isSonarAvailable = 0;
//  readGPS();
//  isGPSAvailable = 0;

  // write to SD card if somthing has changed
  if(isSbusAvailable || isLidarAvailable || isSonarAvailable || isGPSAvailable || isIMUAvailable) {
    String dataString = "";
    dataString += micros(); dataString += ",";
    dataString += armed; dataString += ",";

    dataString += isSbusAvailable; dataString += ",";
    dataString += channels[0]; dataString += ",";
    dataString += channels[1]; dataString += ",";
    dataString += channels[2]; dataString += ",";
    dataString += channels[3]; dataString += ",";
    dataString += channels[4]; dataString += ",";
    dataString += channels[5]; dataString += ",";
    dataString += channels[6]; dataString += ",";
    dataString += channels[7]; dataString += ",";
    dataString += channels[8]; dataString += ",";

    dataString += isLidarAvailable; dataString += ",";
    dataString += String(range, 7);
    dataString += ",";

    dataString += isSonarAvailable; dataString += ",";
    dataString += String(sonarValue, 7);
    dataString += ",";

    dataString += isGPSAvailable; dataString += ",";
    dataString += String(latitude, 15);
    dataString += ",";
    dataString += String(longitude, 15);
    dataString += ",";
    dataString += String(altitude, 15);
    dataString += ",";
    dataString += SIV; dataString += ",";

    dataString += isIMUAvailable; dataString += ",";
    for (int i = 0; i < 4; i++) 
    {
      dataString += String(quats_f[i], 7);
      dataString += ",";
    }
    for (int i = 0; i < 3; i++) 
    {
      dataString += String(rates_f[i], 7);
      dataString += ",";
    }
    for (int i = 0; i < 3; i++) 
    {
      dataString += String(acc_f[i], 7);
      dataString += ",";
    }

    Serial8.println(dataString);
    Serial8.write(13);

    isSbusAvailable = 0;
    isLidarAvailable = 0;
    isSonarAvailable = 0;
    isGPSAvailable = 0;
    isIMUAvailable = 0;

    Serial.println(dataString);
  }
  threads.yield();

//  Serial.print(micros()-lastTime);
//  Serial.print(",");
//  Serial.print(channels[0]);
//  Serial.print(",");
//  Serial.print(range);
//  Serial.print(",");
//  Serial.print(sonarValue);
//  Serial.print(",");
//  Serial.println(SIV);

  //delay(100);

}

void readSBUS() {
  if(x8r.read(&channels[0], &failSafe, &lostFrame)){
    isSbusAvailable = 1;
    // write the SBUS packet to an SBUS compatible servo
    int Ltime = micros();
    if (armed)
      x8r.write(channels);
    int timeElapsed = micros() - Ltime;
//      Serial.println(timeElapsed);
  }
}

void readLidar() {
  if(sensor.available())
  {
       range = sensor.readRangeMillimeters();
       isLidarAvailable = 1;
  }
}

void readSonar() {
  sonarValue = analogRead(A0);
  isSonarAvailable = 1;
}

void readGPS() {
  if (myGPS.getPVT()) {
    latitude = myGPS.getLatitude();
    longitude = myGPS.getLongitude();
    altitude = myGPS.getAltitude();
    SIV = myGPS.getSIV();

    isGPSAvailable = 1;
  }
}

void initIMU()
{
  Serial.println("Initializing VN-100 Serial");
  Serial1.begin(115200);
  delay(100);
  Serial.println("turning off output");
  Serial1.print("$VNASY,0*XX\r\n");
  delay(100);
  Serial.println("resetting");
  Serial1.print("$VNWRG,06,0*XX\r\n");
  delay(100);
  Serial.println("Setting output packet rate");
  Serial1.print("$VNWRG,75,2,1,01,0130*XX\r\n");
  delay(100);
  Serial.println("Setting baud rate");
  Serial1.print("$VNWRG,05,921600*XX\r\n");
  delay(100);
  Serial1.flush();
  delay(100);
  Serial1.begin(921600);
  delay(100);
  Serial.println("starting");
  Serial1.print("$VNASY,1*XX\r\n");
  delay(100);
  while (Serial1.available() > 0) {
    Serial1.read();
  }
  Serial.println("Initialized VN-100 Serial");
}

void readSBUS_thread()
{
  while (1)
  {
    readSBUS();
    threads.yield();
  }
}

void readLidar_thread()
{
  while (1)
  {
    readLidar();
    threads.yield();
  }
}

void readSonar_thread()
{
  while (1)
  {
    readSonar();
    threads.yield();
  }
}

void readGPS_thread()
{
  while (1)
  {
    readGPS();
    threads.yield();
  }
}

void readIMU_thread()
{
  while (1)
  {
    readIMU();
    threads.yield();
  }
}

unsigned short calculate_imu_crc(byte data[], unsigned int length)
  {
    unsigned int i;
    unsigned short crc = 0;
    for(i=0; i<length; i++){
      crc = (byte)(crc >> 8) | (crc << 8);
      crc ^= data[i];
      crc ^= (byte)(crc & 0xff) >> 4;
      crc ^= crc << 12;
      crc ^= (crc & 0x00ff) << 5;
    }
    return crc;
  }

int check_sync_byte(void)
  {
    for (int i = 0; i < 6; i++) {
      Serial1.readBytes(in, 1);
      if (in[0] == 0xFA) {
        return 1;
      }
    }
    return 0;
  }

void read_imu_data(void) {
    Serial1.readBytes(in, 45);

    checksum.b[0] = in[44];
    checksum.b[1] = in[43];

    // uint8_t euler[12]; rpy
    uint8_t quat[16];
    uint8_t acc[12];
    uint8_t rates[12];

    if (calculate_imu_crc(in, 43) == checksum.s) {
      for (int i = 0; i < 4; i++) {
        quat[i] = in[3 + i];
        quat[4+i] = in[7 + i];
        quat[8+i] = in[11 + i];
        quat[12+i] = in[15 + i];
        rates[i] = in[19 + i];
        rates[4+i] = in[23 + i];
        rates[8+i] = in[27 + i];
        acc[i] = in[31 + i];
        acc[4+i] = in[35 + i];
        acc[8+i] = in[39 + i];
      }
    }

    for (int i = 0; i < 4; i++)
    {
      for (int j = 0; j < 4; j++)
      {
        tmp.c[j] = quat[j+i*4];
      }
      quats_f[i] = tmp.f;
    }
    for (int i = 0; i < 3; i++)
    {
      for (int j = 0; j < 4; j++)
      {
        tmp.c[j] = rates[j+i*4];
      }
      rates_f[i] = tmp.f;
    }
    for (int i = 0; i < 3; i++)
    {
      for (int j = 0; j < 4; j++)
      {
        tmp.c[j] = acc[j+i*4];
      }
      acc_f[i] = tmp.f;
    }
  }

int readIMU(void)
{
  int check = 0;
  if (Serial1.available() > 4) {
    check = check_sync_byte();
  }
  if (check == 1) {
    read_imu_data();
    isIMUAvailable = 1;
    return 1;
  }
  return 0;
}
