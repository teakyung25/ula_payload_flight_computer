#include <Arduino.h>
//MPU6050 libraries
#include <Wire.h>
#include <MPU6050.h>

// sd card libs
#include <SPI.h>
#include <../.pio/libdeps/nanoatmega328/SD/src/SD.h>

// Arduino code used to read analog data from XGMP3v3 Differential Pressure
// Sensor and approximate airspeed in [m/s] using a pitot tube
// 
// by Joshua Hrisko, Maker Portal LLC (c) 2021
// 
//
//int input_pin = A6;    // analog input pin
float V_ref = 3.3; // reference voltage
float ADC_res = pow(2.0,10); // ADC resolution (ATmega = 10-bit, SAMD21 = 12-bit)
float P_max =  1.0; // pressure max in [kPa] for specific XGMP3v3 sensor
float P_min = -1.0; // pressure min in [kPa] for specific XGMP3v3 sensor
float V_max = 2.7; // max analog voltage output
float V_min = 0.2; // min analog voltage output
float rho = 1.204; // air density


MPU6050 mpu;
// SD card
File myFile;
int num_data = 0;
void testSD() {
  Serial.print("Initializing SD card...");

  if (!SD.begin(10)) {
    Serial.println("initialization failed!");
    while (1);
  }
  Serial.println("initialization done.");

  // open the file. note that only one file can be open at a time,
  // so you have to close this one before opening another.
  myFile = SD.open("test.txt", FILE_WRITE);

  // if the file opened okay, write to it:
  if (myFile) {
    Serial.print("Writing to test.txt...");
    myFile.println("testing 1, 2, 3.");
    // close the file:
    myFile.close();
    Serial.println("done.");
  } else {
    // if the file didn't open, print an error:
    Serial.println("error opening test.txt");
  }

  
}
// Pressure Sensor 

void setupPressureSensor() {
  analogReference(DEFAULT); // set 3.3V as V_ref | not needed for SAMD21 boards
}

double readPressure(int input_pin) { 
  int adc_val; float volt_calc;  // sensor read variables
  float pres_approx;
  adc_val    = analogRead(input_pin); // read ADC data
  volt_calc  = (adc_val/(ADC_res-1.0))*V_ref; // convert ADC to voltage
  String prnt_str, str_buf; // string buffers for printing to serial port
  str_buf = "ADC val: ";
  prnt_str = str_buf+String(adc_val)+", Voltage: "; // adc value string
  str_buf = prnt_str + String(volt_calc,2)+" V, Pressure: "; // add voltage
  pres_approx = ((P_max-P_min)/(V_max-V_min))*abs((volt_calc-((V_max+V_min)/2.0)));
  prnt_str = str_buf+String(pres_approx,2)+" kPa"; // add pressure approx [kPa]
  Serial.println(prnt_str); // uncomment to print ADC, volt, and press to serial port

  float veloc; // velocity variable
  veloc = sqrt((2.0*(pres_approx*1000.0))/rho); // velocity approximation from pitot tube
  prnt_str = "Velocity: "+String(veloc,2)+" m/s"; // string for velocity printout
  Serial.println(prnt_str); // print velocity to serial port
  delay(50); // wait between readings for valid sensor reading [1ms]

  return veloc;
  
  // re-open the file for reading:
  // myFile = SD.open("cylinder_wall.csv");
  // if (myFile) {
  //   Serial.println("test.csv:");

  //   // read from the file until there's nothing else in it:
  //   while (myFile.available()) {
  //     Serial.write(myFile.read());
  //   }
  //   // close the file:
  //   myFile.close();
  // } else {
  //   // if the file didn't open, print an error:
  //   Serial.println("error opening test.txt");
  // }
}


// MPU6050 Accelerometer 

void checkSettings()
{
  Serial.println();
  
  Serial.print(" * Sleep Mode:            ");
  Serial.println(mpu.getSleepEnabled() ? "Enabled" : "Disabled");
  
  Serial.print(" * Clock Source:          ");
  switch(mpu.getClockSource())
  {
    case MPU6050_CLOCK_KEEP_RESET:     Serial.println("Stops the clock and keeps the timing generator in reset"); break;
    case MPU6050_CLOCK_EXTERNAL_19MHZ: Serial.println("PLL with external 19.2MHz reference"); break;
    case MPU6050_CLOCK_EXTERNAL_32KHZ: Serial.println("PLL with external 32.768kHz reference"); break;
    case MPU6050_CLOCK_PLL_ZGYRO:      Serial.println("PLL with Z axis gyroscope reference"); break;
    case MPU6050_CLOCK_PLL_YGYRO:      Serial.println("PLL with Y axis gyroscope reference"); break;
    case MPU6050_CLOCK_PLL_XGYRO:      Serial.println("PLL with X axis gyroscope reference"); break;
    case MPU6050_CLOCK_INTERNAL_8MHZ:  Serial.println("Internal 8MHz oscillator"); break;
  }
  
  Serial.print(" * Accelerometer:         ");
  switch(mpu.getRange())
  {
    case MPU6050_RANGE_16G:            Serial.println("+/- 16 g"); break;
    case MPU6050_RANGE_8G:             Serial.println("+/- 8 g"); break;
    case MPU6050_RANGE_4G:             Serial.println("+/- 4 g"); break;
    case MPU6050_RANGE_2G:             Serial.println("+/- 2 g"); break;
  }  

  Serial.print(" * Accelerometer offsets: ");
  Serial.print(mpu.getAccelOffsetX());
  Serial.print(" / ");
  Serial.print(mpu.getAccelOffsetY());
  Serial.print(" / ");
  Serial.println(mpu.getAccelOffsetZ());
  
  Serial.println();
}

void setupAccSensor() {
  Serial.println("Initialize MPU6050");
  while(!mpu.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_2G))
  {
    Serial.println("Could not find a valid MPU6050 sensor, check wiring!");
    delay(500);
  }
  Serial.println("Initialized MPU6050");

    // If you want, you can set accelerometer offsets
  // mpu.setAccelOffsetX(2650);
  // mpu.setAccelOffsetY(400);
  // mpu.setAccelOffsetZ(2000);
  //checkSettings();
}
 
double readAcceleration() {
  //Vector rawAccel = mpu.readRawAccel();
  Vector normAccel = mpu.readNormalizeAccel();

  // Serial.print(" Xraw = ");
  // Serial.print(rawAccel.XAxis);
  // Serial.print(" Yraw = ");
  // Serial.print(rawAccel.YAxis);
  // Serial.print(" Zraw = ");

  // Serial.println(rawAccel.ZAxis);
  // Serial.print(" Xnorm = ");
  // Serial.print(normAccel.XAxis);
  // Serial.print(" Ynorm = ");
  // Serial.print(normAccel.YAxis);
  // Serial.print(" Znorm = ");
  // Serial.println(normAccel.ZAxis);
  return normAccel.ZAxis;
}

void writeData(double pitotInner, double pitotOuter, double gForce) {
  Serial.println(myFile);
  Serial.println("initialization done.");
  Serial.println("Outer: " + String(pitotOuter));
  Serial.println("Inner: " + String(pitotInner));
  Serial.println("gForce: " + String(gForce));
  // if the file opened okay, write to it:
  if (!SD.begin(10)) {
    Serial.println("initialization failed!");
    // while (1);
  }

  Serial.println("FAN SETUP COMPLETE");
    myFile = SD.open("data.txt", FILE_WRITE); // IF YOU USE CAPITAL LETTERS I FUCK YOU IN ASS
    Serial.println(myFile);

  // myFile.close();
  // Serial.println(myFile);
  // myFile = SD.open("flightData.txt", FILE_WRITE);
  // Serial.println(myFile);
  if (myFile) {
    // num_data++;
    // if(num_data != 50) {
      Serial.print("Writing Sensor Values");
      myFile.println(String(gForce) + "," + String(pitotInner) + "," + String(pitotOuter));
      // close the file:
      myFile.close();
      Serial.println("done.");
    // } else {
      myFile.close();
    // }
  } else {
    // if the file didn't open, print an error:
    Serial.println("error opening flightData.txt");
  }

}

void setup() {
  Serial.begin(9600); // start serial 
  while (!Serial){;}; // wait for serial to start (required for some boards)
  Serial.println("SETUP !!!! bitch work ");
    setupAccSensor();
  setupPressureSensor();

  // open the file. note that only one file can be open at a time,
  // so you have to close this one before opening another.
  // myFile = SD.open("cylinder_wall.txt", FILE_WRITE);
  myFile.close();
  //control fan//
  pinMode(3,OUTPUT);
  // pinMode(10, );
  analogWrite(3,0); //setting speed to zero


  boolean notLaunched = true;
  while(notLaunched) {
    double zVal = readAcceleration();
    // Serial.println(zVal);
    if(zVal <= 9.5) {
      notLaunched = false;
      analogWrite(3,50);
    } 
  }







}

void loop() {
  
  double outerPressure = readPressure(A6);
  double innerPressure = readPressure(A7);
  Serial.println("Inner outoutouotuot: " + String(innerPressure));
  double gFroce = readAcceleration();
  writeData(innerPressure,outerPressure,gFroce);
  delay(2000);
}


