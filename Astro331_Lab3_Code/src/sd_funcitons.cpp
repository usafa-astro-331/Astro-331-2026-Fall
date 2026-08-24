
#include "definitions.h"     // Project definitions (this directory)
#include "sd_functions.h"

// File-scope objects (NOT in the header)
static SdFat sd;
// static FsFile dataFile;

/*---------------------------------------------------------------------------------------------*/
// Function Definitions:
/*---------------------------------------------------------------------------------------------*/


/// @brief 
/// @param csPin 
/// @return 
bool sd_init(uint8_t csPin)
{
    pinMode(SD_CS_PIN, OUTPUT);
    if (!sd.begin(SD_CS_PIN, SD_SCK_MHZ(25))) {
        while (1) { Serial.println("[ERROR] SD card initialization failed. Card present?"); delay(2000); }
    } else {Serial.println("[INFO] SD Card Initialized.");}
    return false;
}


/**
 * @brief Creates a new data file on the SD card.
 * 
 * @param dataFile  Pointer to a File object that will be opened.
 * @return true     If the file was successfully created and opened.
 * @return false    If file creation/opening failed.
 */
bool sd_createDataFile(FsFile *dataFile) {

  // Generate unique filename
  char filename[20];
  int fileNumber = 1;

  // Find next available file number
  do {
    snprintf(filename, sizeof(filename), "lab3_data%03d.csv", fileNumber);
    fileNumber++;
  } while (sd.exists(filename) && fileNumber <= 999);

  // Check if we exceeded the limit
  if (fileNumber > 999) {
    Serial.println("[ERROR] Maximum file number exceeded (999).");
    return false;
  }

  Serial.print("[INFO] Creating file: ");
  Serial.println(filename);

  *dataFile = sd.open(filename, FILE_WRITE);
  if (!*dataFile) {
    Serial.println("[ERROR] could not create file.");
    return false;
  }

  // Optional: write header row
  dataFile->println("mcu time (ms),current (mA),voltage (V)");
  dataFile->flush();
  return true;
}


/**
 * @brief Recursively lists all files and directories on the SD card.
 * 
 * Opens the given directory, prints file and folder names (with indentation
 * for hierarchy), and displays file sizes. Calls itself recursively for
 * subdirectories.
 * 
 * @param dir   The directory to list (use SD.open("/") for root).
 * @param depth Indentation level for nested directories (start with 0).
 * @return void
 */

void sd_listFiles(String dirName, int depth)
{
  FsFile dir = sd.open(dirName.c_str());
  if (!dir) {
    Serial.println("[ERROR] Could not open directory.");
    return;
  }
  
  while (true)
  {
    FsFile entry = dir.openNextFile();
    if (!entry)
    {
      // no more files
      if (depth == 0) {
        Serial.println("[INFO] Listing SD contents complete.");
      }
      break;
    }

    // indent for clarity
    for (int i = 0; i < depth; i++)
    {
      Serial.print("  ");
    }

    char tempName[32];
    entry.getName(tempName, sizeof(tempName));
    Serial.print(tempName);
    
    if (entry.isDirectory())
    {
      Serial.println("/");
      // Build full path for subdirectory
      String fullPath = dirName;
      if (!dirName.endsWith("/")) {
        fullPath += "/";
      }
      fullPath += tempName;
      sd_listFiles(fullPath, depth + 1); // recurse with full path
    }
    else
    {
      // files have sizes, directories do not
      Serial.print("\t\t");
      Serial.print(entry.size(), DEC);
      Serial.println(" bytes");
    }
    entry.close();
  }
  dir.close();
}

/**
 * @brief Lists files on the SD card with numbers and prompts user to choose one to print.
 * 
 * Scans the root directory, prints files with an index number,
 * and waits for user input of the file number.
 * 
 * @return void
 */
void sd_printFileMenu() {
  FsFile root = sd.open("/");
  
  if (!root) {
    Serial.println("[ERROR] Could not open root directory.");
    return;
  }
  
  const int MAX_FILES = 50;        // Reduced for better memory management
  String fileList[MAX_FILES];      // store filenames
  int fileCount = 0;
  Serial.println("[INFO] Files on SD card:");
  
  while (true) {
    FsFile entry = root.openNextFile();
    if (!entry) break;  // no more files

    if (!entry.isDirectory() && fileCount < MAX_FILES) {
      char tempName[64];  // Increased buffer size for longer filenames
      entry.getName(tempName, sizeof(tempName));
      
      // Ensure null termination
      tempName[sizeof(tempName) - 1] = '\0';
      
      fileList[fileCount] = String(tempName);
      Serial.print("(");
      Serial.print(fileCount + 1);
      Serial.print("): ");
      Serial.print(fileList[fileCount]);
      Serial.print(" (");
      Serial.print(entry.size());
      Serial.println(" bytes)");
      fileCount++;
    } else if (!entry.isDirectory() && fileCount >= MAX_FILES) {
      Serial.println("[CAUTION] More files exist but only showing first 50.");
      entry.close();
      break;
    }
    entry.close();
  }

  root.close();

  if (fileCount > 0) {
    Serial.println("[REQUEST] Enter the file number to print.");
    
    // Clear any existing serial input buffer
    while (Serial.available()) {
      Serial.read();
    }
    
    // Wait for user input with timeout
    unsigned long timeout = millis() + 30000; // 30 second timeout
    while (!Serial.available() && millis() < timeout) {
      delay(10);
    }
    
    if (millis() >= timeout) {
      Serial.println("[CAUTION] Input timeout, returning to Menu.");
      return;
    }
    
    int choice = Serial.parseInt();   // read number user typed
    
    // Clear remaining characters in buffer
    while (Serial.available()) {
      Serial.read();
    }
    
    if (choice > 0 && choice <= fileCount) {
      Serial.print("[INFO] You picked file #");
      Serial.println(choice);
      sd_printFile(fileList[choice - 1].c_str());
    } else {
      Serial.println("[INFO] Invalid choice, returning to Menu.");
    }
  } else {
    Serial.println("[CAUTION] No files found on SD card.");
  }
}

/**
 * @brief Prints the contents of a selected file to Serial.
 * 
 * Opens the file in read mode and sends its contents over Serial.
 * 
 * @param filename Name of the file to print.
 * @return void
 */
void sd_printFile(const char *filename) {
  FsFile file = sd.open(filename);

  if (!file) {
    Serial.print("[ERROR] Error opening file: ");
    Serial.println(filename);
    return;
  }

  // Check file size - warn if very large
  uint32_t fileSize = file.size();
  if (fileSize > 10000) { // 10KB threshold
    Serial.print("[CAUTION] File is large (");
    Serial.print(fileSize);
    Serial.println(" bytes). This may take a while to print.");
  }

  Serial.print("[INFO] ---- Contents of ");
  Serial.print(filename);
  Serial.print(" (");
  Serial.print(fileSize);
  Serial.println(" bytes) ----\n");

  // Print file contents with periodic yield for system stability
  uint32_t bytesRead = 0;
  while (file.available()) {
    Serial.write(file.read());
    bytesRead++;
    
    // Yield to system every 100 bytes to prevent watchdog issues
    if (bytesRead % 100 == 0) {
      yield();
    }
  }

  file.close();
  Serial.println("\n[INFO] ---- End of file ----");
}
