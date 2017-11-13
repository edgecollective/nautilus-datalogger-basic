// Adafruit SPI Flash Total Erase
// Authors: Tony DiCola, Dan Halbert
//
// This example will perform a complete erase of ALL data on the SPI
// flash.  This is handy to reset the flash into a known empty state
// and fix potential filesystem or other corruption issues.
//
// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
// !!  NOTE: YOU WILL ERASE ALL DATA BY RUNNING THIS SKETCH!  !!
// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
//
// Usage:
// - Modify the pins and type of fatfs object in the config
//   section below if necessary (usually not necessary).
// - Upload this sketch to your M0 express board.
// - Upon starting the board Neopixel will be blue
// - About 13 seconds later, the Neopixel should starting flashing
//   green once per second. This indicates the SPI flash has been
//   erased and all is well.
// - If the Nexopixel starts flashing red two or three times a second,
//   an error has occurred.

#include <SPI.h>
#include <Adafruit_SPIFlash.h>
#include <Adafruit_SPIFlash_FatFs.h>

// Configuration of the flash chip pins.
// You don't normally need to change these if using a Feather/Metro
// M0 express board.
#define FLASH_TYPE     SPIFLASHTYPE_W25Q16BV  // Flash chip type.
                                              // If you change this be
                                              // sure to change the fatfs
                                              // object type below to match.

#if defined(ADAFRUIT_CIRCUITPLAYGROUND_M0)
  #define FLASH_SS       SS                    // Flash chip SS pin.
  #define FLASH_SPI_PORT SPI                   // What SPI port is Flash on?
  #define NEOPIN         8
#else
  #define FLASH_SS       SS1                    // Flash chip SS pin.
  #define FLASH_SPI_PORT SPI1                   // What SPI port is Flash on?
  #define NEOPIN         40
#endif

Adafruit_SPIFlash flash(FLASH_SS, &FLASH_SPI_PORT);     // Use hardware SPI

void setup() {

Serial.begin(115200);
Serial.println("erasing ....");

 Serial.print("Flash chip JEDEC ID: 0x");
  pinMode(LED_BUILTIN, OUTPUT);
   

  // Initialize flash library and check its chip ID.
  if (!flash.begin(FLASH_TYPE)) {
    blink(5, 50);
  }
  if (!flash.EraseChip()) {
    blink(2,50);
  }
  blink(2, 1000);
  Serial.println("erased!");
}

void loop() {
  // Nothing to do in the loop.
  blink(2,100);
  delay(1000);
}

void blink(int times, int delay_ms) {
  for (int i=0;i<times;i++) {
  digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
  delay(delay_ms);                       // wait for a second
  digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW
  delay(delay_ms);                       // wait for a second
  }
}

