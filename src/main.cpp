
#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include "BME280_t.h"                                                   // import BME280 template library
#include "SmallNMEA2000.h"
#include "commandline.h"



const SNMEA2000ProductInfo productInfomation PROGMEM={
                                       1300,                        // N2kVersion
                                       44,                         // Manufacturer's product code
                                       "Baro",    // Manufacturer's Model ID
                                       "1.2.3.4 (2017-06-11)",     // Manufacturer's Software version code
                                       "5.6.7.8 (2017-06-11)",      // Manufacturer's Model version
                                       "0000002",                  // Manufacturer's Model serial code
                                       0,                           // SertificationLevel
                                       1                            // LoadEquivalency
};
const SNMEA2000ConfigInfo configInfo PROGMEM={
      "Pressure Monitor",
      "Luna Technical Area",
      "https://github.com/ieb/CanPressure"
};



const unsigned long txPGN[] PROGMEM = { 
    130310L, //  Outside Environmental parameters
    130311L,  // Environmental parameters
    SNMEA200_DEFAULT_TX_PGN
};
const unsigned long rxPGN[] PROGMEM = { 
  SNMEA200_DEFAULT_RX_PGN
};

const SNMEA2000DeviceInfo devInfo = SNMEA2000DeviceInfo(
  01,   // device serial number
  130,  //Atmospheric Function
  85 // Enternal Environment monitor
);

#define DEVICE_ADDRESS 25
#define SNMEA_SPI_CS_PIN PIN_PA5  

PressureMonitor pressureMonitor = PressureMonitor(DEVICE_ADDRESS, 
          &devInfo, 
          &productInfomation, 
          &configInfo, 
          &txPGN[0], 
          &rxPGN[0],
          SNMEA_SPI_CS_PIN
);
CommandLine commandLine = CommandLine(&Serial, &pressureMonitor);
BME280<> BMESensor;                                                     // instantiate sensor


bool diagnostics = false;
void setDiagnostics(bool enabled) {
  pressureMonitor.setDiagnostics(enabled);
  diagnostics = enabled;

}

#define READING_UPDATE_PERIOD 10000

void sendReading() {
  static unsigned long lastReadingUpdate=0;
  unsigned long now = millis();
  if ( now > lastReadingUpdate+READING_UPDATE_PERIOD ) {
    lastReadingUpdate = now;
    BMESensor.refresh();
    if ( diagnostics ) {
      Serial.print(F("Sending Reading pressure:"));
      Serial.print(BMESensor.pressure);
      Serial.print(F(" temperature:"));
      Serial.print(BMESensor.temperature);
      Serial.print(F(" humidity:"));
      Serial.println(BMESensor.humidity);
    }
    pressureMonitor.sendOutsideEnvironmentParameters(BMESensor.pressure, BMESensor.temperature, BMESensor.humidity);
    pressureMonitor.sendEnvironmentParameters(BMESensor.pressure, BMESensor.temperature, BMESensor.humidity);
  }
}


void setup() {
  Serial.begin(115200);
  Serial.println(F("Pressure monitor start"));
  commandLine.begin();


  Wire.begin();
  while (!BMESensor.begin()) {
    Serial.println(F("Failed to start Sensor, retry in 5s"));
    delay(5000);
  }
  Serial.println(F("Sensor Ok"));
  pressureMonitor.setSerialNumber(commandLine.getSerialNumber());
  pressureMonitor.setDeviceAddress(commandLine.getDeviceAddress());
  Serial.println(F("Opening CAN"));
  while ( !pressureMonitor.open() ) {
    Serial.println(F("Failed to start NMEA2000, retry in 5s"));
    delay(5000);
  }
  Serial.println(F("Opened, MCP2515 Operational"));
  Serial.println(F("Running..."));
}

void loop() {
  sendReading();
  pressureMonitor.processMessages();
  commandLine.checkCommand();
}


