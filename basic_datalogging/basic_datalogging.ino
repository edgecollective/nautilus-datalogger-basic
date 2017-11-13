
#include <SPI.h>
#include <Adafruit_SPIFlash.h>
#include <Adafruit_SPIFlash_FatFs.h>

#define FLASH_TYPE     SPIFLASHTYPE_W25Q16BV  // Flash chip type.
                                              // If you change this be
                                              // sure to change the fatfs
                                              // object type below to match.

#define FLASH_SS       SS1                    // Flash chip SS pin.
#define FLASH_SPI_PORT SPI1                   // What SPI port is Flash on?

Adafruit_SPIFlash flash(FLASH_SS, &FLASH_SPI_PORT);     // Use hardware SPI

Adafruit_W25Q16BV_FatFs fatfs(flash);

// Configuration for the datalogging file:
#define FILE_NAME      "data1.csv"
  
#define intervalMS 1000 // delay between sensor reads, in ms

int reading_index=0;

void setup() {

   pinMode(LED_BUILTIN, OUTPUT);
   
  // Initialize serial port and wait for it to open before continuing.
  Serial.begin(115200);
  Serial.println("Adafruit SPI Flash FatFs Simple Datalogging Example");

  // Initialize flash library and check its chip ID.
  if (!flash.begin(FLASH_TYPE)) {
    Serial.println("Error, failed to initialize flash chip!");
    while(1);
  }
  Serial.print("Flash chip JEDEC ID: 0x"); Serial.println(flash.GetJEDECID(), HEX);

  // First call begin to mount the filesystem.  Check that it returns true
  // to make sure the filesystem was mounted.
  if (!fatfs.begin()) {
    Serial.println("Error, failed to mount newly formatted filesystem!");
    Serial.println("Was the flash chip formatted with the fatfs_format example?");
    while(1);
  }
  Serial.println("Mounted filesystem!");

  Serial.println("Logging data every 60 seconds...");
}

void loop() {
  // Open the datalogging file for writing.  The FILE_WRITE mode will open
  // the file for appending, i.e. it will add new data to the end of the file.
  File dataFile = fatfs.open(FILE_NAME, FILE_WRITE);
  // Check that the file opened successfully and write a line to it.
  if (dataFile) {
   
    dataFile.print(reading_index, DEC);
    dataFile.print(",");
    dataFile.print(3);
    dataFile.println();
  
    dataFile.close();
    Serial.println(reading_index);
    reading_index++;
    
    blink(1,200);
  }
  else {
    Serial.println("Failed to open data file for writing!");
  }

  delay(1000L);

}

void blink(int times, int delay_ms) {
  for (int i=0;i<times;i++) {
  digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
  delay(delay_ms);                       // wait for a second
  digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW
  delay(delay_ms);                       // wait for a second
  }
}
