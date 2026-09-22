/* Astro331_Lab6_Code
* ===========================================================
* Created by: Lt Col Wyatt Harris, Spring 2026
* Description: Use this code IAW Astro 331 Lab 6 instructions.
* Project: Attitude Determination Lab
* Libraries:
*	    greiman/SdFat@^2.3.1
*     sparkfun/SparkFun MAX1704x Fuel Gauge Arduno Library@^1.0.3
*     sparkfun/SparkFun 9DoF IMU Breakout - ICM 20948 - Arduino Library@^1.3.2
* =========================================================== */

/*---------------------------------------------------------------------------------------------*/
// Library includes:
/*---------------------------------------------------------------------------------------------*/
#include <Arduino.h>          // Main Arduino library
#include <Wire.h>             // libray for I2C communication
#include <SPI.h>              // SPI communication library
#include <SdFat.h>            // SD Card library
#include "definitions.h"     // Project definitions (this directory)
#include "sd_functions.h"     //SD helper functions (this directory)
#include <SparkFun_MAX1704x_Fuel_Gauge_Arduino_Library.h> // MAX17048 fuel gauge
#include <ICM_20948.h>        //Sparkfun IMU library


/*---------------------------------------------------------------------------------------------*/
// Globals:
/*---------------------------------------------------------------------------------------------*/
// Objects:
FsFile dataFile;   // data file object
HardwareSerial Xbee(2); // Serial object for communication with XBee 
SFE_MAX1704X lipo; // SparkFun Thing Plus ESP32-WROOM onboard fuel gauge (I2C addr 0x36)
ICM_20948_I2C imu_sensor; // IMU object

// Variables:
uint32_t timeLastXbee; // time of next Xbee check
uint32_t interval_Xbee = 10; // time interval between Xbee checks (ms)
uint32_t timeLastHeartBeat; // time of last heartbeat (ms)
uint32_t interval_heartBeat = 2000; // interval between heartbeat (ms)

float sun_plusX, sun_minusX, sun_plusY, sun_minusY;
float sun_X = 0.0, sun_Y = 0.0, sun_direction = 0.0;
float S_mag;
int16_t n_sun_sensor_reads = 5; // number of readings to average for sun sensor test point

uint32_t timeNext_testPoint; // time of next test point (ms)
uint32_t interval_testPoint = 50; // time interval between test points (ms)

float gyro_Z = 0.0;
float mag_X = 0.0;
float mag_Y = 0.0;

/*---------------------------------------------------------------------------------------------*/
// Function Prototypes (see defintiions after loop()):
/*---------------------------------------------------------------------------------------------*/
void get_command_from_ground_station();
void get_sat_rssi();
void send_battery_telemetry();
void lab6_run_test();

/////////////////////////////////////////////////////////////////////////////////////////////////
// SETUP:
/////////////////////////////////////////////////////////////////////////////////////////////////
void setup() {
  Serial.begin(115200); // Begin Serial communication with computer
  while (!Serial) {delay(10);} // Wait for user to open Serial monitor before proceeding
  Serial.println("[INFO] Hello World!");

  Wire.begin(); // Initialize I2C communication

  // Initialize built-in RGB LED (WS2812) and STAT LED
  // #define RGB_BUILTIN  2
  pinMode(RGB_BUILTIN, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  neopixelWrite(RGB_BUILTIN, 255, 0, 0); // Default to red (R=255, G=0, B=0)

  //----------------------------------------------
  // Initialize SD Card
  //----------------------------------------------
  sd_init(SD_CS_PIN);
  //----------------------------------------------

  //----------------------------------------------
  // Initialize MAX17048 fuel gauge
  //----------------------------------------------
  if (!lipo.begin(Wire)) // Uses I2C address 0x36) 
  {
    Serial.println("[WARN] MAX17048 not detected on I2C (0x36). Battery telemetry (cmd 4) will be unavailable.");
  } else {
    lipo.quickStart();    // Improves initial SOC accuracy after boot. Returns 0 on success.
    Serial.println("[INFO] MAX17048 online.");
  }
  //----------------------------------------------

  //----------------------------------------------
  // Initialize ICM20948
  //----------------------------------------------
  if (imu_sensor.begin(Wire, 1) != ICM_20948_Stat_Ok) {
    Serial.println("[CAUTION] IMU not found.");
    while (1);
  } else{
    // // 1. Set to Continuous Mode for consistent sampling
    // imu_sensor.setSampleMode((ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), ICM_20948_Sample_Mode_Continuous);
    // // 2. Configure Sample Rate Divider (formula: 1125 / (1 + divider) Hz)
    // ICM_20948_smplrt_t mySmplrt;
    // mySmplrt.a = 1; // Accel divider 1: 1125 / (1+1) = ~562.5 Hz
    // mySmplrt.g = 1; // Gyro divider 1: 1125 / (1+1) = ~562.5 Hz
    // imu_sensor.setSampleRate(ICM_20948_Internal_Acc, mySmplrt);
    // imu_sensor.setSampleRate(ICM_20948_Internal_Gyr, mySmplrt);    
    Serial.println("[INFO] IMU Initialized.");
  }
  //----------------------------------------------

  //----------------------------------------------
  // Initialize Sun Sensor
  //----------------------------------------------
  analogReadResolution(12);

  //----------------------------------------------
  // Initialize Serial link with XBee
  //----------------------------------------------
  Xbee.begin(9600,SERIAL_8N1, XBEE_RX, XBEE_TX);  // Begin MCU <> XBee communication
  Xbee.setTimeout(20);
  Xbee.println("[INFO] KestrelSAT online \npress 1 for options");
  // ----------------------------------------------

  Serial.println("[INFO] SETUP COMPLETE.");
  neopixelWrite(RGB_BUILTIN, 0, 255, 0); // Set to green (R=0, G=255, B=0)
  
}
/////////////////////////////////////////////////////////////////////////////////////////////////


/////////////////////////////////////////////////////////////////////////////////////////////////
// MAIN LOOP:
/////////////////////////////////////////////////////////////////////////////////////////////////
void loop() {
  //----------------------------------------------
  // Check for data from ground station
  // ----------------------------------------------
  if (timeLastXbee + interval_Xbee < millis()) { // periodic Xbee send
    timeLastXbee = millis();
    get_command_from_ground_station();
  }

  if (timeLastHeartBeat + interval_heartBeat < millis()) { // periodic Xbee send
    timeLastHeartBeat = millis();
    Serial.println(".");
    Xbee.println("*");
  
  }

  delay(1);
}
/////////////////////////////////////////////////////////////////////////////////////////////////


/////////////////////////////////////////////////////////////////////////////////////////////////
// FUNCITON DEFINITIONS:
/////////////////////////////////////////////////////////////////////////////////////////////////

/**
 * @brief Process and execute commands received from the ground station.
 *
 * Reads a command string from Xbee (typically connected to an XBee or
 * other ground station link), parses the command into an integer code, and
 * executes the corresponding action. Supported commands:
 *   - 1 : Print a help menu to Xbee
 *   - 2 : Query and return the satellite radio RSSI via get_sat_rssi()
 *   - 3 : Toggle the onboard LED on/off and report the new state
 *   - 4 : Get Battery Information
 *   - 5 : Execute Lab 6 Test Run (collect sun sensor & IMU data, save to SD, print to Serial)
 *
 * @note Command strings are trimmed of leading/trailing whitespace before
 *       parsing. Commands are expected to be integers at the start of the
 *       string. Unrecognized commands are ignored.
 *
 * @warning This function uses blocking serial reads (Xbee.readString()) 
 *          and may delay program execution if no command is received.
 *
 * @see get_sat_rssi(), runTest()
 */
void get_command_from_ground_station() {
  if (!Xbee.available() && !Serial.available()) return;

  int do_it = 0; // default to no command
  if (Xbee.available())
  {
    String what_to_do = Xbee.readStringUntil('\n');
    what_to_do.trim();
    if (what_to_do.length() == 0) return;
    Serial.print("Received from XBee: ");
    Serial.println(what_to_do);
    do_it = what_to_do.toInt();
  }else if (Serial.available())
  {
    String what_to_do = Serial.readStringUntil('\n');
    what_to_do.trim();
    if (what_to_do.length() == 0) return;
    Serial.print("Received from Serial: ");
    Serial.println(what_to_do);
    do_it = what_to_do.toInt();
  }

  switch (do_it) {
    case 1:
      Xbee.print("1 help \n");
      Xbee.print("2 get RSSI \n");
      Xbee.print("3 toggle LED \n");
      Xbee.print("4 battery telemetry (V, SOC, dSOC/dt) \n");
      Xbee.print("5 lab 6 run test \n");
      Serial.print("1 help \n");
      Serial.print("2 get RSSI \n");
      Serial.print("3 toggle LED \n");
      Serial.print("4 battery telemetry (V, SOC, dSOC/dt) \n");
      Serial.print("5 lab 6 run test \n");
      break;

    case 2:
      get_sat_rssi();
      break;

    case 3:
      digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
      Serial.print("LED is now ");
      Serial.println(digitalRead(LED_BUILTIN) ? "LED ON" : "LED OFF");
      Xbee.println(digitalRead(LED_BUILTIN) ? " LED ON" : " LED OFF");
      break;

    case 4:
      send_battery_telemetry();
      break;

    case 5:
      lab6_run_test();
      break;

    default:
      Xbee.println("[CAUTION] Invalid input from ground station, ignoring.");
      Serial.println("[CAUTION] invalid input from ground station, ignoring.");
      break;
  }
  return;
}//end function get_command_from_ground_station()

/**
 * @brief Query the connected XBee radio for received signal strength (RSSI).
 *
 * This function places the XBee module into command mode, issues the ATDB
 * command to request the RSSI of the last received packet, parses the hex
 * response into a decimal value, and prints the result over both Serial and
 * Xbee in dBm.
 *
 * @note The function uses blocking delays and loops; it will not return until
 *       the XBee responds correctly with "OK\r".
 *
 * @warning Assumes that Xbee is connected to an XBee radio and already
 *          initialized at the correct baud rate.
 *
 * @post After the function runs, the XBee is returned to data mode using the
 *       ATCN command.
 *
 * @see Digi XBee AT Command Reference for details on ATDB and ATCN commands.
 */
void get_sat_rssi() {
  Xbee.println(" standby for RSSI");
  
  // put the radio in command mode:
  bool not_done = true;
  String ok_response = "OK\r";   //The﻿response we expect.
  String response = String("");  //Create an empty string
  Serial.println("Starting get_sat_rssi()");
  
  // Read the text of the response into the response variable
  while (not_done) {  // As long as we did not get a response from the XBee
    response = String("");
    delay(1100);
    Xbee.print("+++");  // Put the XBee 3 into 'Command Mode'
    // Serial.print("+++");   // Put the XBee 3 into 'Command Mode'

    // delay(1100);  // Wait for the XBee to finish
    while (response.length() < ok_response.length()) {
      if (Xbee.available() > 0) {

        response += (char)Xbee.read();  // Read a single character at a time
      }
    }
    not_done = !response.equals(ok_response);  // Set the not_done flag to the opposite of the result of equality check
  }
  // Serial.println(response);
      

  // If we got the right response, configure the radio and return true.   
  Xbee.print("ATDB\r");  // destination high and destination low addresses set to 0 means all messages will only go
  delay(100);               // Wait for the XBee
  response = String("");
  while (Xbee.available() > 0) {
    response += (char)Xbee.read();  //Read a single character at a time
    
  }
  Serial.println(response);
  Xbee.print("ATCN\r");  // Switch back to data mode

  String response2 = response; 
  uint32_t dec_response = strtoul(response2.c_str(), NULL, 16);

  Xbee.print("RSSI: -");
  Xbee.print(dec_response);
  Xbee.println(" dBm");
  Serial.println("sat rssi sent");
} // end function get_rssi()

/**
 * @brief Send MAX17048 battery telemetry over the XBee link.
 *
 * Reads:
 *  - Battery voltage (V)
 *  - State of Charge (SOC, %)
 *  - Change / discharge rate (dSOC/dt, %/hr; negative = discharging)
 *
 * Output format (single line):
 *   BAT,V=<volts>,SOC=<percent>,CR=<percent_per_hr>
 */
void send_battery_telemetry() {

  const float v = lipo.getVoltage();
  const float soc = lipo.getSOC();
  const float crate = lipo.getChangeRate(); // %/hr (positive=charging, negative=discharging)

  // Ground-station friendly, parseable response
  Xbee.print("BAT,V=");
  Xbee.print(v, 3);
  Xbee.print("V, SOC=");
  Xbee.print(soc, 1);
  Xbee.print("%, CR=");
  Xbee.print(crate, 3);
  Xbee.println("%/hr");

  // Also mirror to USB serial for debugging
  Serial.print("[BAT] V=");
  Serial.print(v, 3);
  Serial.print(" V, SOC=");
  Serial.print(soc, 1);
  Serial.print(" %, CR=");
  Serial.print(crate, 3);
  Serial.println(" %/hr");
} 

/*---------------------------------------------------------------------------------------------*/
// Run Test:
/*---------------------------------------------------------------------------------------------*/
/**
* @brief Runs the test for Lab 6
* @return none
*/
void lab6_run_test() {

  char file_name[20];
  if(sd_createDataFile(&dataFile)){
    // write header row
    dataFile.println("mcu time (ms),gyro Z (dps),mag X (uT),mag Y (uT),sun direction (deg),sun_plusX,sun_plusY,sun_minusX,sun_minusY");
    dataFile.flush();
    dataFile.getName(file_name, 20);
    Xbee.print("[INFO] Data file created successfully: ");
    Xbee.println(file_name);
    Serial.print("[INFO] Data file created successfully: ");
    Serial.println(file_name);
  } else {
    Xbee.println("[ERROR] Failed to create data file. Aborting test.");
    Serial.println("[ERROR] Failed to create data file. Aborting test.");
    return;
  }
  Serial.println("[INFO] Ready to start Lab 6 test, send any key to begin (send 'X' to stop test)...");
  Xbee.println("[INFO] Ready to start Lab 6 test, send any key to begin (send 'X' to stop test)...");
  
  while(!Serial.available() && !Xbee.available()){} // wait for user to send any key to start test
  delay(100); // small delay to ensure serial buffer is fully received
  while(Serial.available()) Serial.read(); // clear serial buffer
  while(Xbee.available()) Xbee.read(); // clear Xbee buffer
  
  timeNext_testPoint = millis();
  int test_point_count = 0;
  while(true){

    if(Serial.available() > 0 || Xbee.available() > 0) { // Check for user input from USB or XBee
      char c = (Serial.available() > 0) ? Serial.read() : Xbee.read();
      if(c == 'X' || c == 'x') { // If user sent 'X', stop the test
        Serial.print("[INFO] Test Complete. File ");
        Serial.print(file_name);
        Xbee.print("[INFO] Test Complete. File: ");
        Xbee.print(file_name);
        if(dataFile.close()) {
          Serial.println(" closed.");
          Xbee.println(" closed.");
        } else {
          Serial.println(" failed to close.");
          Xbee.println(" failed to close.");
        }
        return;
      } else {
        Serial.println("[CAUTION] Invalid Input, continuing test...");
        Xbee.println("[CAUTION] Invalid Input, continuing test...");
      }
    }

    uint32_t timeNow = millis();
    if(timeNow > timeNext_testPoint){ // Collect Test Point loop
      test_point_count++;
      timeNext_testPoint += interval_testPoint; // Update time for next Test Point
      
      // Collect IMU Test Point:
      imu_sensor.getAGMT();
      gyro_Z = imu_sensor.gyrZ(); //this is negative to convert measurment to KestrelSAT body frame
      mag_X = imu_sensor.magX();
      mag_Y = imu_sensor.magY();

      // Collect Sun Sensor Test Point:
      // Average readings for each analog channel
      sun_plusX = 0.0;
      sun_minusX = 0.0;
      sun_plusY = 0.0;
      sun_minusY = 0.0;
      for (int i = 0; i < n_sun_sensor_reads; i++) {
        sun_plusX += analogRead(SUN_SENSOR_PLUS_X_PIN);
        sun_minusX += analogRead(SUN_SENSOR_MINUS_X_PIN);
        sun_plusY += analogRead(SUN_SENSOR_PLUS_Y_PIN);
        sun_minusY += analogRead(SUN_SENSOR_MINUS_Y_PIN);
      }
      sun_plusX /= n_sun_sensor_reads;
      sun_minusX /= n_sun_sensor_reads;
      sun_plusY /= n_sun_sensor_reads;
      sun_minusY /= n_sun_sensor_reads;

      // ////////////* find sun direction *////////////////////////////////////////
      // // // uncomment sun_plusX & sun_plusY lines to calculate sun direction
      // // // (highlight them, CTRL-/)
      S_mag = sun_plusX + sun_minusX + sun_plusY + sun_minusY;
      // sun_X = ;
      // sun_Y = ;
      sun_direction = (atan2(sun_Y, sun_X) * RAD_TO_DEG);
      if (sun_direction < 0) {
        sun_direction += 360; // Adjust to range 0-360
      }
      /////////////////////////////////////////////////////////////////////////////

      // Print data to .csv file:
      dataFile.print(timeNow);
      dataFile.print(",");
      dataFile.print(gyro_Z,4);
      dataFile.print(",");
      dataFile.print(mag_X,4);
      dataFile.print(",");
      dataFile.print(mag_Y,4);
      dataFile.print(",");
      dataFile.print(sun_direction,4);
      dataFile.print(",");
      dataFile.print(sun_plusX);
      dataFile.print(",");
      dataFile.print(sun_plusY);
      dataFile.print(",");
      dataFile.print(sun_minusX);
      dataFile.print(",");
      dataFile.println(sun_minusY);

      // Print data to USB & XBee serial:
      String test_point_string;
      test_point_string += "t:";
      test_point_string += timeNow;
      test_point_string += ",gZ:";
      test_point_string += gyro_Z;
      test_point_string += ",mX:";
      test_point_string += mag_X;
      test_point_string += ",mY:";
      test_point_string += mag_Y;
      test_point_string += ",sDir:";
      test_point_string += sun_direction;
      test_point_string += ",s+X:";
      test_point_string += sun_plusX;
      test_point_string += ",s+Y:";
      test_point_string += sun_plusY;
      test_point_string += ",s-X:";
      test_point_string += sun_minusX;
      test_point_string += ",s-Y:";
      test_point_string += sun_minusY;
      test_point_string += "\n";

      //Print to USB Serial:
      Serial.print(test_point_string);
      //Print to XBee:
      if(test_point_count % 10 == 0){ 
        dataFile.flush(); // save file every 10 test points
        Xbee.print("tp:");
        Xbee.println(test_point_count);
        // Xbee.print(test_point_string);
      }
    }
  }  
}