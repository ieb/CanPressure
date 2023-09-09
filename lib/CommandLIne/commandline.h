#pragma once

#include "SmallNMEA2000.h"
#include <avr/eeprom.h>
#include "BME280_t.h"


#define HR_DEVICE_ADDRESS 0
#define HR_SERIAL_NUMBER 2
#define HR_PGN_130310 4
#define HR_PGN_130311 6
#define HR_PGN_130313 8
#define HR_PGN_130314 10
#define HR_PGN_130316 12
#define HR_CRC 14
#define HR_SIZE 16


// symbols from main.cpp so we dont have to get complicated.
extern double getTemperature();
extern double getPressure();
extern double getHumidity();
extern BME280 BMESensor;
extern void setDiagnostics(bool enabled);

const uint8_t epromDefaultValues[HR_CRC] PROGMEM = {
  0x1a, 0x00,  // device address 26
  0x04, 0x00,  // serial number
  0x88, 0x13,          //Period in ms for 130310L, //  Outside Environmental parameters
  0x88, 0x13,          //Period in ms for 130311L,  // Environmental parameters
  0x88, 0x13,          //Period in ms for 130313L,  // Humidity parameters
  0x88, 0x13,          //Period in ms for 130314L,  // Pressure parameters
  0x88, 0x13          //Period in ms for 130316L,  // Temperature parameters
};




class CommandLine {
    public:
        CommandLine(UartClass * io, SNMEA2000 *nmea2000) : io{io}, nmea2000{nmea2000} {};

        unsigned char deviceAddress = 25;
        uint32_t serialNumber = 123;
        uint16_t periodPGN130310 = 5000;  //  Outside Environmental parameters
        uint16_t periodPGN130311 = 5000;  //  Environmental parameters
        uint16_t periodPGN130313 = 5000;  //  Humidity parameters
        uint16_t periodPGN130314 = 5000;  //  Pressure parameters
        uint16_t periodPGN130316 = 5000;  //  Temperature parameters

        void begin() {
            loadDefaults();
        };


        void checkCommand() {
            if (io->available()) {
                char chr = io->read();
                switch ( chr ) {
                    case 'h': showHelp(); break;
                    case 's': showStatus(); break;
                    case 'r': readSensors();  break;
                    case 'S': doSetup(); break;
                    case 'R': doReset(); break;
                    case 'F': loadDefaults(); break;
                    case 'd': toggleDiagnostics(); break;
                }
            }
        };
    private:
        UartClass * io;
        bool diagnosticsEnabled = false;
        SNMEA2000 *nmea2000;

        void doReset() {
            _PROTECTED_WRITE(RSTCTRL.SWRR, 1);
        }

        void toggleDiagnostics() {
            diagnosticsEnabled = !diagnosticsEnabled;
            setDiagnostics(diagnosticsEnabled);
        };

        /**
         * @brief crc for mdbus, polynomial = 0x8005, reverse in, reverse out, init 0xffff;
         * 
         * @param array 
         * @param length 
         * @return uint16_t 
         */
        uint16_t crc16(const uint8_t *array, uint16_t length) {
            uint16_t crc = 0xffff;
            while (length--) {
                if ((length & 0xFF) == 0) yield();  // RTOS
                uint8_t data = *array++;
                data = (((data & 0xAA) >> 1) | ((data & 0x55) << 1));
                data = (((data & 0xCC) >> 2) | ((data & 0x33) << 2));
                data =          ((data >> 4) | (data << 4));
                crc ^= ((uint16_t)data) << 8;
                for (uint8_t i = 8; i; i--) {
                if (crc & (1 << 15)) {
                    crc <<= 1;
                    crc ^= 0x8005;
                } else {
                    crc <<= 1;
                }
                }
            }
            crc = (((crc & 0XAAAA) >> 1) | ((crc & 0X5555) << 1));
            crc = (((crc & 0xCCCC) >> 2) | ((crc & 0X3333) << 2));
            crc = (((crc & 0xF0F0) >> 4) | ((crc & 0X0F0F) << 4));
            //  crc = (( crc >> 8) | (crc << 8));
            //  crc ^= endmask;
            return crc;
        };

        void readSensors() {

            io->print(F("Temperatuire="));
            io->println(BMESensor.temperature);
            io->print(F("Pressure="));
            io->println(BMESensor.pressure);
            io->print(F("Humidity="));
            io->println(BMESensor.humidity);
        }

        void loadDefaults() {
            uint8_t eepromContents[HR_SIZE];
            eeprom_read_block((void*)&eepromContents[0], (const void*)0, HR_SIZE);
            uint16_t crcv =  crc16(&eepromContents[0], HR_SIZE-2);
            uint16_t crcvs = (0xff&eepromContents[HR_CRC]) | (0xff00&(eepromContents[HR_CRC+1]<<8)); // little endian
            if (crcv != crcvs || eepromContents[HR_DEVICE_ADDRESS] == 0 ) {
                io->print(F("EPROM not initialised"));
                for(uint8_t i = 0; i < HR_SIZE-2; i++) {
                    eepromContents[i] = pgm_read_byte_near(epromDefaultValues+i);
                }
                crcv =  crc16(&eepromContents[0], HR_SIZE-2);
                eepromContents[HR_CRC] = 0xff&(crcv);
                eepromContents[HR_CRC+1] = 0xff&(crcv>>8); // little endian
                eeprom_update_block(&eepromContents[0], (void*)0, HR_SIZE);
            }
            deviceAddress = asInt16(eepromContents,HR_DEVICE_ADDRESS);
            serialNumber = asInt16(eepromContents,HR_SERIAL_NUMBER);
            periodPGN130310 = asInt16(eepromContents,HR_PGN_130310);
            periodPGN130311 = asInt16(eepromContents,HR_PGN_130311);
            periodPGN130313 = asInt16(eepromContents,HR_PGN_130313);
            periodPGN130314 = asInt16(eepromContents,HR_PGN_130314);
            periodPGN130316 = asInt16(eepromContents,HR_PGN_130316);

            showStatus();

        };


        bool readInt(int16_t *v) {
            String line = io->readStringUntil('\r');
            line.trim();
            if ( line.length() > 0 ) {
                *v = line.toInt();
                return true;
            }
            return false;
        };
        bool updateSetting(int8_t * registers, uint8_t offset, int16_t minval =  INT16_MIN, int16_t maxval = INT16_MAX) {
            int16_t value;
            if (readInt(&value)) {
                if ( value > minval && value < maxval ) {
                    registers[offset] = 0xff&value;
                    registers[offset+1] = 0xff&(value>>8);
                    io->println(F(" - changed"));
                    return true;
                } else {
                    io->println(F(" - invalid, no change"));
                }
            } else {
                io->println(F(" - no change"));
            }
            return false;
        };
        int16_t asInt16(uint8_t * registers, uint8_t offset) {
            return registers[offset] | (0xff00&(registers[offset+1]<<8));
        };

        void dumpRegisters(uint8_t * b) {
            for(int i = 0; i < HR_SIZE; i+=2 ) {
                io->print(i);
                io->print(':');
                io->println(asInt16(b,i));
            }
        }


        void doSetup() {
            showStatus();
            io->setTimeout(10000);
            uint8_t eepromContents[HR_SIZE];
            eeprom_read_block((void*)&eepromContents[0], (const void*)0, HR_SIZE);
            io->println(F("Setup: Enter new values or return to leave unchanged."));
            io->print(F("Setup - device address "));io->print(eepromContents[HR_DEVICE_ADDRESS]);io->print(F(" >"));
            bool changed = updateSetting(eepromContents, HR_DEVICE_ADDRESS, 0, 255);

            io->print(F("Setup - serial number "));io->print(asInt16(eepromContents, HR_SERIAL_NUMBER));io->print(F(" >"));
            changed = updateSetting(eepromContents, HR_SERIAL_NUMBER) || changed;

            io->println(F("Setup - PGN periods in ms, 0 disables  "));
            io->print(F(" - PGN:130310 Outside Environmental Parameters "));io->print(asInt16(eepromContents, HR_PGN_130310));io->print(F("ms >"));
            changed = updateSetting(eepromContents, HR_PGN_130310) || changed;
            io->print(F(" - PGN:130311 Environmental Parameters "));io->print(asInt16(eepromContents, HR_PGN_130311));io->print(F("ms >"));
            changed = updateSetting(eepromContents, HR_PGN_130311) || changed;
            io->print(F(" - PGN:130313 Humidity Parameters "));io->print(asInt16(eepromContents, HR_PGN_130313));io->print(F("ms >"));
            changed = updateSetting(eepromContents, HR_PGN_130313) || changed;
            io->print(F(" - PGN:130314 Pressure Parameters "));io->print(asInt16(eepromContents, HR_PGN_130314));io->print(F("ms >"));
            changed = updateSetting(eepromContents, HR_PGN_130314) || changed;
            io->print(F(" - PGN:130316 Temperature Parameters "));io->print(asInt16(eepromContents, HR_PGN_130316));io->print(F("ms >"));
            changed = updateSetting(eepromContents, HR_PGN_130316) || changed;

            if ( changed ) {
                // update CRC and save.
                uint16_t crcv =  crc16(&eepromContents[0], HR_SIZE-2);
                eepromContents[HR_CRC] = 0xff&(crcv);
                eepromContents[HR_CRC+1] = 0xff&(crcv>>8); // little endian
                eeprom_update_block((const void*)&eepromContents[0], (void*)0, HR_SIZE);
                // update device address, which may have changed.
                deviceAddress = asInt16(eepromContents,HR_DEVICE_ADDRESS);
                serialNumber = asInt16(eepromContents,HR_SERIAL_NUMBER);
                periodPGN130310 = asInt16(eepromContents,HR_PGN_130310);
                periodPGN130311 = asInt16(eepromContents,HR_PGN_130311);
                periodPGN130313 = asInt16(eepromContents,HR_PGN_130313);
                periodPGN130314 = asInt16(eepromContents,HR_PGN_130314);
                periodPGN130316 = asInt16(eepromContents,HR_PGN_130316);
                showStatus();
                io->println(F("Settings changed, device will restart in 5s "));
                delay(5000);
                doReset();
            }
            io->setTimeout(1000);
        };


        void showHelp() {
            io->println(F("Pressure Monitor - key presses"));
            io->println(F("  - 'h' or '?' to show this message"));
            io->println(F("  - 's' show status"));
            io->println(F("  - 'd' toggle diagnostics"));
            io->println(F("  - 'r' read sensor"));            
            io->println(F("  - 'S' setup"));
            io->println(F("  - 'R' reset"));
            io->println(F("  - 'F' factory reset"));
        };

        void printPeriod(int16_t period) {
            if ( period == 0) {
                io->println(F("disabled"));
            } else {
                io->print(period);
                io->println(F(" ms"));
            }
        }

        
        void showStatus() {

            io->print(F("Status\nDevice Address :"));
            io->println(deviceAddress);
            io->print(F("Serial Number  :"));
            io->println(serialNumber);
            io->print(F("PGN:130310 Outside Environmental Parameters :"));printPeriod(periodPGN130310);
            io->print(F("PGN:130311 Environmental Parameters         :"));printPeriod(periodPGN130311);
            io->print(F("PGN:130313 Environmental Parameters         :"));printPeriod(periodPGN130313);
            io->print(F("PGN:130314 Environmental Parameters         :"));printPeriod(periodPGN130314);
            io->print(F("PGN:130316 Environmental Parameters         :"));printPeriod(periodPGN130316);

            nmea2000->dumpStatus();
            io->print((F("MCU V=")));
            io->print(readMCUVoltage());
            io->print((F("mV T=")));
            int32_t t = readMCUTemperature();
            t -= 273;
            io->print(t);
            io->println((F("C")));

        };

        uint16_t readMCUVoltage() {
            analogReference(INTERNAL1V024);
            delayMicroseconds(100);
            int32_t vddmeasure = analogReadEnh(ADC_VDDDIV10, 12); // Take it at 12 bits
            vddmeasure *= 10; // since we measured 1/10th VDD
            int16_t returnval = vddmeasure >> 2; // divide by 4 to get into millivolts.
            if (vddmeasure & 0x02) {
                // if last two digits were 0b11 or 0b10 we should round up
                returnval++;
            }
            return returnval;
        }

        uint16_t readMCUTemperature() {
            int8_t sigrowOffset = SIGROW.TEMPSENSE1;
            uint8_t sigrowGain = SIGROW.TEMPSENSE0;
            analogSampleDuration(128); // must be >= 32µs * f_CLK_ADC per datasheet 30.3.3.7
            analogReference(INTERNAL1V024);
            uint32_t reading = analogRead(ADC_TEMPERATURE);
            reading -= sigrowOffset;
            reading *= sigrowGain;
            reading += 0x80; // Add 1/2 to get correct rounding on division below
            reading >>= 8; // Divide result to get Kelvin
            return reading;

        }

};


