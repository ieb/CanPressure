
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

uint8_t pgnEnableRegister = 0xff;


const unsigned long txPGN[] PROGMEM = { 
    130310L, //  Outside Environmental parameters
    130311L,  // Environmental parameters
    130313L,  // Humidity parameters
    130314L,  // Pressure parameters
    130316L,  // Temperature parameters
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

#define READING_UPDATE_PERIOD 2000

void sendReading() {
  static unsigned long nextReadingUpdate=0;
  static unsigned long next130310Update=0;
  static unsigned long next130311Update=0;
  static unsigned long next130313Update=0;
  static unsigned long next130314Update=0;
  static unsigned long next130316Update=0;
  static byte sid = 0;
  unsigned long now = millis();
  if ( now > nextReadingUpdate ) {
    nextReadingUpdate = now+READING_UPDATE_PERIOD;
    BMESensor.refresh();
    if ( diagnostics ) {
      Serial.print(F("Reading pressure:"));
      Serial.print(BMESensor.pressure);
      Serial.print(F(" temperature:"));
      Serial.print(BMESensor.temperature);
      Serial.print(F(" humidity:"));
      Serial.println(BMESensor.humidity);
    }
    sid++;
  }
#define MAIN_CABIN_TEMPERATURE 4
#define INSIDE_HUMIDITY 0
#define ATMOSPHERIC 0
  if ( commandLine.periodPGN130310 != 0 && now > next130310Update) {
    next130310Update = now + commandLine.periodPGN130310;
    pressureMonitor.sendOutsideEnvironmentParameters(sid, SNMEA2000::n2kDoubleNA,  CToKelvin(BMESensor.temperature), BMESensor.pressure);    
  }
  if ( commandLine.periodPGN130311 != 0 && now > next130311Update) {
    next130311Update = now + commandLine.periodPGN130311;
    pressureMonitor.sendEnvironmentParameters(sid, BMESensor.pressure, MAIN_CABIN_TEMPERATURE, CToKelvin(BMESensor.temperature), INSIDE_HUMIDITY, BMESensor.humidity);
  }
  if ( commandLine.periodPGN130313 != 0 && now > next130313Update) {
    next130313Update = now + commandLine.periodPGN130313;
      // NMEA2000 v3 messages, more resolution.
      pressureMonitor.sendHumidity(sid, INSIDE_HUMIDITY, 0, BMESensor.humidity);
  }
  if ( commandLine.periodPGN130314 != 0 && now > next130314Update) {
    next130314Update = now + commandLine.periodPGN130314;
    pressureMonitor.sendPressure(sid, ATMOSPHERIC, 0, BMESensor.pressure);
  }
  if ( commandLine.periodPGN130316 != 0 && now > next130316Update) {
    next130316Update = now + commandLine.periodPGN130316;
    pressureMonitor.sendTemperature(sid, MAIN_CABIN_TEMPERATURE, 0, CToKelvin(BMESensor.temperature));
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
  pressureMonitor.setSerialNumber(commandLine.serialNumber);
  pressureMonitor.setDeviceAddress(commandLine.deviceAddress);
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


