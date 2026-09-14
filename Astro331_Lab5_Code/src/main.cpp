// See full_featured.ino example:
// Copyright 2021 Arducam Technology co., Ltd. All Rights Reserved.
// License: MIT License (https://en.wikipedia.org/wiki/MIT_License)
// Web: http://www.ArduCAM.com
// This program is a demo of how to use most of the functions
// of the library with ArduCAM Spi camera, and can run on Arduino platform.
// This demo was made for ArduCAM Spi Camera.
// It needs to be used in combination with PC software.
// It can test ArduCAM Spi Camerafunctions

/*---------------------------------------------------------------------------------------------*/
// Library includes:
/*---------------------------------------------------------------------------------------------*/
#include <Arduino.h>
#include "ArducamLink.h"
#include "Arducam_Mega.h"
#include "SPI.h"

/*---------------------------------------------------------------------------------------------*/
// Configuration:
/*---------------------------------------------------------------------------------------------*/
// SparkFun Thing Plus C (ESP32 WROOM) hardware SPI bus. Do not name these
// SCK/MISO/MOSI -- the board variant already defines those symbols.
#define PIN_SPI_SCK   18
#define PIN_SPI_MISO  19
#define PIN_SPI_MOSI  23

#define PIN_CAM_CS    12  // Chip select for the Arducam Mega
#define PIN_SD_CS      5  // Chip select for the on-board microSD (shares the bus)

// Arducam Mega registers used for the pre-flight bus check.
#define CAM_REG_SENSOR_ID     0x40
#define CAM_SENSOR_ID_MIN     0x81
#define CAM_SENSOR_ID_MAX     0x87

/*---------------------------------------------------------------------------------------------*/
// Globals:
/*---------------------------------------------------------------------------------------------*/
Arducam_Mega myCAM(PIN_CAM_CS);
ArducamLink myUart;

uint8_t temp             = 0xff;
uint8_t sendFlag         = TRUE;
uint8_t commandBuff[20]  = {0};
uint8_t commandLength    = 0;
uint32_t readImageLength = 0;
uint8_t jpegHeadFlag     = 0;
bool cameraReady         = false;

/*---------------------------------------------------------------------------------------------*/
// Functions:
/*---------------------------------------------------------------------------------------------*/

// Read one Mega register directly, mirroring the driver's own bus transaction.
// Used only to confirm the camera is on the bus before handing control to the
// driver, whose waitI2cIdle() spins forever if the camera never answers.
uint8_t camReadReg(uint8_t addr)
{
    digitalWrite(PIN_CAM_CS, LOW);
    SPI.transfer(addr & 0x7F);
    SPI.transfer(0x00);
    uint8_t value = SPI.transfer(0x00);
    digitalWrite(PIN_CAM_CS, HIGH);
    return value;
}

// True only if the sensor ID register reads back a value the driver recognises.
// All-zero / all-ones means nothing is driving MISO: check wiring and CS.
bool cameraOnBus(void)
{
    for (uint8_t attempt = 0; attempt < 5; attempt++) {
        uint8_t id = camReadReg(CAM_REG_SENSOR_ID);
        if (id >= CAM_SENSOR_ID_MIN && id <= CAM_SENSOR_ID_MAX) {
            return true;
        }
        delay(20);
    }
    return false;
}

uint8_t readBuffer(uint8_t* imagebuf, uint8_t length)
{
    if (imagebuf[0] == 0xff && imagebuf[1] == 0xd8) {
        jpegHeadFlag    = 1;
        readImageLength = 0;
        myUart.arducamUartWrite(0xff);
        myUart.arducamUartWrite(0xAA);
        myUart.arducamUartWrite(0x01);

        myUart.arducamUartWrite((uint8_t)(myCAM.getTotalLength() & 0xff));
        myUart.arducamUartWrite((uint8_t)((myCAM.getTotalLength() >> 8) & 0xff));
        myUart.arducamUartWrite((uint8_t)((myCAM.getTotalLength() >> 16) & 0xff));
        myUart.arducamUartWrite((uint8_t)((myCAM.getTotalLength() >> 24) & 0xff));
        myUart.arducamUartWrite(((CAM_IMAGE_PIX_FMT_JPG & 0x0f) << 4) | 0x01);
    }
    if (jpegHeadFlag == 1) {
        readImageLength += length;
        for (uint8_t i = 0; i < length; i++) {
            myUart.arducamUartWrite(imagebuf[i]);
        }
    }
    if (readImageLength == myCAM.getTotalLength()) {
        jpegHeadFlag = 0;
        myUart.arducamUartWrite(0xff);
        myUart.arducamUartWrite(0xBB);
    }
    return sendFlag;
}

void stop_preivew()
{
    readImageLength = 0;
    jpegHeadFlag    = 0;
    uint32_t len    = 9;

    myUart.arducamUartWrite(0xff);
    myUart.arducamUartWrite(0xBB);
    myUart.arducamUartWrite(0xff);
    myUart.arducamUartWrite(0xAA);
    myUart.arducamUartWrite(0x06);
    myUart.arducamUartWriteBuff((uint8_t*)&len, 4);
    myUart.printf("streamoff");
    myUart.arducamUartWrite(0xff);
    myUart.arducamUartWrite(0xBB);
}

/////////////////////////////////////////////////////////////////////////////////////////////////
// SETUP:
/////////////////////////////////////////////////////////////////////////////////////////////////
void setup() {
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, HIGH);

    // Park both chip selects inactive (HIGH) before the bus comes up, so the
    // on-board microSD card cannot drive MISO while we talk to the camera.
    pinMode(PIN_SD_CS, OUTPUT);
    digitalWrite(PIN_SD_CS, HIGH);
    pinMode(PIN_CAM_CS, OUTPUT);
    digitalWrite(PIN_CAM_CS, HIGH);
    digitalWrite(LED_BUILTIN, LOW);
    delay(10);

    myUart.arducamUartBegin(115200);
    delay(200);

    myUart.send_data_pack(7, "[INFO] Starting SPI..");
    SPI.begin(PIN_SPI_SCK, PIN_SPI_MISO, PIN_SPI_MOSI, PIN_CAM_CS);

    myUart.send_data_pack(7, "[INFO] Starting Camera..");
    cameraReady = cameraOnBus();
    if (!cameraReady) {
        // Bail out instead of calling myCAM.begin(): the driver would block in
        // waitI2cIdle() and the board would stop answering the host software.
        myUart.send_data_pack(7, "[ERROR] No response from Mega on SPI - check CS/SCK/MISO/MOSI wiring and power.");
        return;
    }

    myCAM.begin();
    myUart.send_data_pack(8, "[INFO] Mega Initialized!");
    myCAM.registerCallBack(readBuffer, 200, stop_preivew);
    digitalWrite(LED_BUILTIN, HIGH);
}

/////////////////////////////////////////////////////////////////////////////////////////////////
// MAIN LOOP:
/////////////////////////////////////////////////////////////////////////////////////////////////
void loop() {
    if (!cameraReady) {
        // Keep the link alive and blink so the fault is visible on the bench.
        static bool ledOn = false;
        ledOn = !ledOn;
        digitalWrite(LED_BUILTIN, ledOn ? HIGH : LOW);
        myUart.send_data_pack(7, "[ERROR] Camera not initialized.");
        delay(1000);
        return;
    }

    if (myUart.arducamUartAvailable()) {
        temp = myUart.arducamUartRead();
        delay(2); // changed from 5
        if (temp == 0x55) {
            while (myUart.arducamUartAvailable()) {
                commandBuff[commandLength] = myUart.arducamUartRead();
                if (commandBuff[commandLength] == 0xAA) {
                    break;
                }
                commandLength++;
            }
            myUart.arducamFlush();
            myUart.uartCommandProcessing(&myCAM, commandBuff);
            commandLength = 0;
        }
    }
    myCAM.captureThread();
}
