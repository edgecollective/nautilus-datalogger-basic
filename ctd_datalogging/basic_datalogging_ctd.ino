


#include <Wire.h>
#include "MS5837.h"
#include "TSYS01.h"

#include "RTClib.h"







#include <SPI.h>
#include <Adafruit_SPIFlash.h>
#include <Adafruit_SPIFlash_FatFs.h>

#define FLASH_TYPE     SPIFLASHTYPE_W25Q16BV  // Flash chip type.
                                              // If you change this be
                                              // sure to change the fatfs
                                              // object type below to match.

#define FLASH_SS       SS1                    // Flash chip SS pin.
#define FLASH_SPI_PORT SPI1                   // What SPI port is Flash on?


#define FREQUENCY 30000 
#define CONDUCT_INPUT 5 // the input pin to the board  

#define CPU_HZ 48000000
#define TIMER_PRESCALER_DIV 1024

#define SLEEP_TIME 1 // in seconds

#define VBATPIN A7
#include <RTCZero.h>

RTC_DS3231 rtc_p;

RTCZero rtc;
int AlarmTime;

void startTimer(int frequencyHz);
void setTimerFrequency(int frequencyHz);
void TC3_Handler();

char daysOfTheWeek[7][12] = {"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};

MS5837 pressure_sensor;
TSYS01 temp_sensor;

Adafruit_SPIFlash flash(FLASH_SS, &FLASH_SPI_PORT);     // Use hardware SPI

Adafruit_W25Q16BV_FatFs fatfs(flash);

// Configuration for the datalogging file:
#define FILE_NAME      "data1.csv"
  
#define intervalMS 1000 // delay between sensor reads, in ms

#define numSamples 3 // number of conductivity samples

int reading_index=0;

bool isLEDOn = false;

void setup() {

    analogReadResolution(12);

    // input pin
  pinMode(CONDUCT_INPUT, OUTPUT);
 startTimer(FREQUENCY);
 
  Wire.begin();

rtc.begin(); // internal rtc

if (! rtc_p.begin()) { // external rtc
    if (DEBUG) Serial.println("Couldn't find RTC");
    while (1);
  }
  

  pressure_sensor.init();
    pressure_sensor.setModel(MS5837::MS5837_30BA);

  temp_sensor.init();

  
  pressure_sensor.setFluidDensity(997); // kg/m^3 (997 freshwater, 1029 for seawater)



   pinMode(LED_BUILTIN, OUTPUT);
   
  // Initialize serial port and wait for it to open before continuing.
  Serial.begin(115200);

  /*
   while (!Serial) {
    delay(100);
  }
  */
  
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

// battery
float measuredvbat = analogRead(VBATPIN);
measuredvbat *= 2;    // we divided by 2, so multiply back
measuredvbat *= 3.3;  // Multiply by 3.3V, our reference voltage
measuredvbat /= 4096; // convert to voltage

// conductivity
int conductValue=0;
for (int i=0;i<numSamples;i++) {
conductValue += analogRead(A0);
 //Serial.println(sensorValue);
 delay(200);  
}
float conductAve=float(conductValue)/numSamples;



  pressure_sensor.read();
  temp_sensor.read();

  Serial.print(reading_index);
  Serial.print(", temp=");
  Serial.print(pressure_sensor.temperature());
  Serial.print(", altitude=");
  Serial.println(pressure_sensor.altitude());


  
DateTime now = rtc_p.now();


  // Open the datalogging file for writing.  The FILE_WRITE mode will open
  // the file for appending, i.e. it will add new data to the end of the file.
  File dataFile = fatfs.open(FILE_NAME, FILE_WRITE);
  // Check that the file opened successfully and write a line to it.
  if (dataFile) {
   
    dataFile.print(reading_index, DEC);
    dataFile.print(",");
    
    dataFile.print(now.unixtime());
    dataFile.print(",");
    dataFile.print(now.year(), DEC);
    dataFile.print('/');
    dataFile.print(now.month(), DEC);
    dataFile.print('/');
    dataFile.print(now.day(), DEC);
    dataFile.print(" ");
    dataFile.print(now.hour(), DEC);
    dataFile.print(':');
    dataFile.print(now.minute(), DEC);
    dataFile.print(':');
    dataFile.print(now.second(), DEC);
    dataFile.print(",");

    dataFile.print(pressure_sensor.pressure());
    dataFile.print(",");
    dataFile.print(pressure_sensor.temperature());
    dataFile.print(",");
    dataFile.print(pressure_sensor.depth());
    dataFile.print(",");
    dataFile.print(pressure_sensor.altitude());
    dataFile.print(",");
    dataFile.print(temp_sensor.temperature());
    dataFile.print(",");
    dataFile.print(conductAve,3);
     dataFile.print(",");
    dataFile.print(measuredvbat,3);
    
    dataFile.println();
  
    dataFile.close();
    
    reading_index++;
    
    blink(1,30);
  }
  else {
    Serial.println("Failed to open data file for writing!");
  }

  //delay(1000L);

  AlarmTime += SLEEP_TIME; // Adds 5 seconds to alarm time
  AlarmTime = AlarmTime % 60; // checks for roll over 60 seconds and corrects
  rtc.setAlarmSeconds(AlarmTime); // Wakes at next alarm time, i.e. every 5 secs

}

void blink(int times, int delay_ms) {
  for (int i=0;i<times;i++) {
  digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
  delay(delay_ms);                       // wait for a second
  digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW
  delay(delay_ms);                       // wait for a second
  }
}


void setTimerFrequency(int frequencyHz) {
  int compareValue = (CPU_HZ / (TIMER_PRESCALER_DIV * frequencyHz)) - 1;
  TcCount16* TC = (TcCount16*) TC3;
  // Make sure the count is in a proportional position to where it was
  // to prevent any jitter or disconnect when changing the compare value.
  TC->COUNT.reg = map(TC->COUNT.reg, 0, TC->CC[0].reg, 0, compareValue);
  TC->CC[0].reg = compareValue;
  //Serial.println(TC->COUNT.reg);
  //Serial.println(TC->CC[0].reg);
  while (TC->STATUS.bit.SYNCBUSY == 1);
}

void startTimer(int frequencyHz) {
  REG_GCLK_CLKCTRL = (uint16_t) (GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK0 | GCLK_CLKCTRL_ID_TCC2_TC3) ;
  while ( GCLK->STATUS.bit.SYNCBUSY == 1 ); // wait for sync

  TcCount16* TC = (TcCount16*) TC3;

  TC->CTRLA.reg &= ~TC_CTRLA_ENABLE;
  while (TC->STATUS.bit.SYNCBUSY == 1); // wait for sync

  // Use the 16-bit timer
  TC->CTRLA.reg |= TC_CTRLA_MODE_COUNT16;
  while (TC->STATUS.bit.SYNCBUSY == 1); // wait for sync

  // Use match mode so that the timer counter resets when the count matches the compare register
  TC->CTRLA.reg |= TC_CTRLA_WAVEGEN_MFRQ;
  while (TC->STATUS.bit.SYNCBUSY == 1); // wait for sync

  // Set prescaler to 1024
  TC->CTRLA.reg |= TC_CTRLA_PRESCALER_DIV1024;
  while (TC->STATUS.bit.SYNCBUSY == 1); // wait for sync

  setTimerFrequency(frequencyHz);

  // Enable the compare interrupt
  TC->INTENSET.reg = 0;
  TC->INTENSET.bit.MC0 = 1;

  NVIC_EnableIRQ(TC3_IRQn);

  TC->CTRLA.reg |= TC_CTRLA_ENABLE;
  while (TC->STATUS.bit.SYNCBUSY == 1); // wait for sync
}

void TC3_Handler() {
  TcCount16* TC = (TcCount16*) TC3;
  // If this interrupt is due to the compare register matching the timer count
  // we toggle the LED.
  if (TC->INTFLAG.bit.MC0 == 1) {
    TC->INTFLAG.bit.MC0 = 1;
    // Write callback here!!!
    digitalWrite(CONDUCT_INPUT, isLEDOn);
    isLEDOn = !isLEDOn;
  }
}
