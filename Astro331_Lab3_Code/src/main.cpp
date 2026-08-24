/* Astro331_Lab3_Code.ino
* ===========================================================
* Created by: Lt Col Wyatt Harris, Spring 2026
* Description: Use this code IAW Astro 331 Lab 3 instructions.
* Project: Electrical Power Subsystem (EPS) Lab
* Libraries:
*	    adafruit/Adafruit INA237 and INA238 Library@^1.0.3
*	    greiman/SdFat@^2.3.1
* =========================================================== */

/*---------------------------------------------------------------------------------------------*/
// Library includes:
/*---------------------------------------------------------------------------------------------*/
#include <Arduino.h>          // Main Arduino library
#include <Wire.h>             // libray for I2C communication
#include <SPI.h>              // SPI communication library
#include <SdFat.h>            // SD Card library
#include <Adafruit_INA238.h>
#include "definitions.h"     // Project definitions (this directory)
#include "sd_functions.h"     //SD helper functions (this directory)

/*---------------------------------------------------------------------------------------------*/
// Globals:
/*---------------------------------------------------------------------------------------------*/
// Objects:
FsFile dataFile;   // data file object

// extern FsFile dataFile;
Adafruit_INA238 ina238 = Adafruit_INA238();

// Variables:
uint32_t timeNext_testPoint; // time of next test point (ms)
uint32_t interval_testPoint = 100; // time interval between test points (ms)
uint32_t timeLastMenuDisplay; // time of last menu display (ms)
uint32_t interval_menuDisplay = 10000; // interval between menu displays (ms)

const int num_samples_per_testpoint = 40; // number of samples per testpoint to average over

/*---------------------------------------------------------------------------------------------*/
// Function Prototypes (see defintiions after loop()):
/*---------------------------------------------------------------------------------------------*/
void displayMenu();
void runTest();
void initINA238();

/////////////////////////////////////////////////////////////////////////////////////////////////
// SETUP:
/////////////////////////////////////////////////////////////////////////////////////////////////
void setup() {
  Serial.begin(115200); // Begin Serial communication with computer
  while (!Serial) {delay(10);} // Wait for user to open Serial monitor before proceeding
  Serial.println("[INFO] Hello World!");

  Wire.begin(); // Initialize I2C communication

  // Initialize built-in RGB LED (WS2812)
  #define RGB_BUILTIN  2
  pinMode(RGB_BUILTIN, OUTPUT);
  neopixelWrite(RGB_BUILTIN, 255, 255, 0); // Set to green (R=0, G=255, B=0)

  //----------------------------------------------
  // Initialize SD Card
  //----------------------------------------------
  sd_init(SD_CS_PIN);
  //----------------------------------------------

  //----------------------------------------------
  // Initialize INA238 Voltage/Current Sensor
  // ----------------------------------------------
  initINA238();
  // ----------------------------------------------

  Serial.println("[INFO] SETUP COMPLETE.");
  neopixelWrite(RGB_BUILTIN, 0, 255, 0); // Set to green (R=0, G=255, B=0)
  
  displayMenu();
}
/////////////////////////////////////////////////////////////////////////////////////////////////


/////////////////////////////////////////////////////////////////////////////////////////////////
// MAIN LOOP:
/////////////////////////////////////////////////////////////////////////////////////////////////
void loop() {
  
  if(Serial.available()>0){ // if user sends character, parse it
    char c = Serial.read();
    while(Serial.available()>0) Serial.read(); // clear any characters in buffer
    switch(c){
      case 'S': // run test
        //--------
        Serial.println("[INFO] Beginning test.");        
        runTest();
        //--------
        break;
      case 'L': // list files
        //--------
        Serial.println("[INFO] Listing files on SD card...");
        sd_listFiles("/", 0);
        //--------
        break;
      case 'P': // print file selected by user
        //--------
        sd_printFileMenu();
        //--------
        break;
      default:
        Serial.println("[CAUTION] Invalid Input.");
        break;
    }
    displayMenu();
  }

  if (timeLastMenuDisplay + interval_menuDisplay < millis()) { // periodic menu display
    timeLastMenuDisplay = millis();
    displayMenu();
  }

}
/////////////////////////////////////////////////////////////////////////////////////////////////


/////////////////////////////////////////////////////////////////////////////////////////////////
// FUNCITON DEFINITIONS:
/////////////////////////////////////////////////////////////////////////////////////////////////

/*---------------------------------------------------------------------------------------------*/
// Display Menu:
/*---------------------------------------------------------------------------------------------*/
/**
* @brief Displays Main Menu
* @return none
*/
void displayMenu(){
  Serial.println("\n--------------------------");
  Serial.println("[MENU]");
  Serial.println("  Send 'S' to Start test.");
  Serial.println("  Send 'L' to List files on SD card.");
  Serial.println("  Send 'P' to Print file from SD.");
  Serial.println("--------------------------\n");
} // end function displayMenu()


/*---------------------------------------------------------------------------------------------*/
// Run Test:
/*---------------------------------------------------------------------------------------------*/
/**
* @brief Runs the test
* @return none
// */
void runTest(){
  neopixelWrite(RGB_BUILTIN, 0, 0, 255); // Set to blue (R=0, G=0, B=255)


  sd_createDataFile(&dataFile); // create data file on SD card
  
  while(Serial.available()>0) Serial.read(); // clear any characters in buffer
  Serial.println("[INFO] Send any key to start. ***Send 'X' to stop test.***");
  while(!Serial.available()){delay(10);} // Wait for user to start test
  while(Serial.available()>0) Serial.read(); // clear any characters in buffer
  while(true){
    
    if(Serial.available()>0){ // Check for user input
      char c = Serial.read();
      switch(c){
        case 'X':
          while(Serial.available()>0) Serial.read(); // clear any characters in buffer
          dataFile.close();
          Serial.println("[INFO] Test Complete.");
          neopixelWrite(RGB_BUILTIN, 0, 255, 0); // Set to green (R=0, G=255, B=0)
          return;
        default:
          Serial.println("[CAUTION] Invalid Input, continuing test...");
          break;
      }
    }
    
    if(millis() > timeNext_testPoint){ // Collect Test Point loop
      uint32_t startTime = millis();
      timeNext_testPoint += interval_testPoint; // Update time for next Test Point

      // Collect Test Point (each reading takes ~ 1ms):
      float testPoint_current_mA = 0.0; 
      float testPoint_voltage_V = 0.0; 
      for (int ii = 0; ii < num_samples_per_testpoint; ii++){ // sum X readings
        testPoint_current_mA += ina238.getCurrent_mA();
        testPoint_voltage_V += ina238.getBusVoltage_V() + (ina238.getShuntVoltage_mV() / 1000.0);
      }
      testPoint_current_mA /= num_samples_per_testpoint; // average readings 
      testPoint_voltage_V /= num_samples_per_testpoint; // average readings

      // Print data to file:
      dataFile.print(millis());
      dataFile.print(",");
      dataFile.print(testPoint_current_mA,6);
      dataFile.print(",");
      dataFile.println(testPoint_voltage_V,6);
      dataFile.flush(); // save file

      //Print to Serial:
      Serial.print("Current(mA):");
      Serial.print(testPoint_current_mA,6);
      Serial.print(",Voltage(V):");
      Serial.println(testPoint_voltage_V,6);
      // Serial.print(",collectTime(ms):");
      // Serial.println(millis() - startTime); //~95 ms per test point
    }
  }
} // end function runTest()


/*---------------------------------------------------------------------------------------------*/
// Initialize INA238 Sensor:
/*---------------------------------------------------------------------------------------------*/
/**
* @brief Initializes INA238 sensor
* @return none
// */
void initINA238()
{
  if (!ina238.begin()) {
    Serial.println("[ERROR] Couldn't find INA238 chip");
    while (1)
      ;
  }
  Serial.println("[INFO] Found INA238 chip");
  // set shunt resistance and max current
  ina238.setShunt(0.015, 0.5); //

  ina238.setAveragingCount(INA2XX_COUNT_128);
  uint16_t counts[] = {1, 4, 16, 64, 128, 256, 512, 1024};
  Serial.print("[INFO] Averaging counts: ");
  Serial.println(counts[ina238.getAveragingCount()]);

  // set the time over which to measure the current and bus voltage
  ina238.setVoltageConversionTime(INA2XX_TIME_150_us);
  Serial.print("[INFO] Voltage conversion time: ");
  switch (ina238.getVoltageConversionTime()) {
  case INA2XX_TIME_50_us:
    Serial.print("50");
    break;
  case INA2XX_TIME_84_us:
    Serial.print("84");
    break;
  case INA2XX_TIME_150_us:
    Serial.print("150");
    break;
  case INA2XX_TIME_280_us:
    Serial.print("280");
    break;
  case INA2XX_TIME_540_us:
    Serial.print("540");
    break;
  case INA2XX_TIME_1052_us:
    Serial.print("1052");
    break;
  case INA2XX_TIME_2074_us:
    Serial.print("2074");
    break;
  case INA2XX_TIME_4120_us:
    Serial.print("4120");
    break;
  }
  Serial.println(" uS");

  ina238.setCurrentConversionTime(INA2XX_TIME_150_us);
  Serial.print("[INFO] Current conversion time: ");
  switch (ina238.getCurrentConversionTime()) {
  case INA2XX_TIME_50_us:
    Serial.print("50");
    break;
  case INA2XX_TIME_84_us:
    Serial.print("84");
    break;
  case INA2XX_TIME_150_us:
    Serial.print("150");
    break;
  case INA2XX_TIME_280_us:
    Serial.print("280");
    break;
  case INA2XX_TIME_540_us:
    Serial.print("540");
    break;
  case INA2XX_TIME_1052_us:
    Serial.print("1052");
    break;
  case INA2XX_TIME_2074_us:
    Serial.print("2074");
    break;
  case INA2XX_TIME_4120_us:
    Serial.print("4120");
    break;
  }
  Serial.println(" uS");
} // end function initINA238()


