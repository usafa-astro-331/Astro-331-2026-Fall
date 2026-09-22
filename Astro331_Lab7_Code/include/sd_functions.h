#pragma once

#include <Arduino.h>
#include <SdFat.h>

/*---------------------------------------------------------------------------------------------*/
// Function Prototypes (see defintiions in .cpp file):
/*---------------------------------------------------------------------------------------------*/
bool sd_init(uint8_t csPin);

bool sd_createDataFile(FsFile *dataFile, const char *preamble);
void sd_listFiles(String dirName, int depth);
void sd_printFileMenu();
void sd_printFile(const char *filename);
