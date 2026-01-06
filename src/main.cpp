/*
  main.cpp

  Simple test program to send three frames to the IRHAJHUS meter using
  the provided PZEM004Tv30 library and print the responses.

  Notes:
  - This example uses `Serial` for the PZEM interface. If your board
    has a separate HardwareSerial for the meter (e.g. `Serial1`), replace
    `Serial` with that instance in the `PZEM004Tv30 pzem(Serial);` line.
  - The PZEM library communicates at 9600 baud, so we begin Serial at 9600.
*/

#include <Arduino.h>
#include "PZEM004Tv30.h"

// Pins for ESP32 hardware serial (adjust if different)
#define TXD1 17
#define RXD1 16
#define RS485_DE 4

// Use default address; change if your meter uses a different one
#define METER_ADDR PZEM_DEFAULT_ADDR

// Hardware serial instance for meter (ESP32 UART1)
HardwareSerial meterSerial(1);

// Instantiate PZEM for ESP32: pass HardwareSerial&, RX, TX, addr
PZEM004Tv30 pzem(meterSerial, RXD1, TXD1, METER_ADDR);

void printElectrical()
{
    Serial.println("--- Electrical Frame ---");
    Serial.print("V1: "); Serial.println(pzem.voltage_1());
    Serial.print("I1: "); Serial.println(pzem.current_1());
    Serial.print("P1: "); Serial.println(pzem.power_1());
    Serial.print("V2: "); Serial.println(pzem.voltage_2());
    Serial.print("I2: "); Serial.println(pzem.current_2());
    Serial.print("P2: "); Serial.println(pzem.power_2());
    Serial.print("V3: "); Serial.println(pzem.voltage_3());
    Serial.print("I3: "); Serial.println(pzem.current_3());
    Serial.print("P3: "); Serial.println(pzem.power_3());
    Serial.print("Total Power: "); Serial.println(pzem.total_power());
}

void printEnergyFreq()
{
    Serial.println("--- Energy & Frequency Frame ---");
    // updateEnergyFreq() is public and reads frequency/energy
    if (pzem.updateEnergyFreq()) {
        Serial.print("Frequency: "); Serial.println(pzem.getFrequency());
        Serial.print("Energy P: "); Serial.println(pzem.getEnergyP());
        Serial.print("Energy N: "); Serial.println(pzem.getEnergyN());
    } else {
        Serial.println("Failed to read Energy/Frequency frame");
    }
}

void printTHD()
{
    Serial.println("--- THD Frame ---");
    if (pzem.updateTHD()) {
        Serial.print("THD V1: "); Serial.println(pzem.thdVoltage1());
        Serial.print("THD V2: "); Serial.println(pzem.thdVoltage2());
        Serial.print("THD V3: "); Serial.println(pzem.thdVoltage3());
        Serial.print("THD I1: "); Serial.println(pzem.thdCurrent1());
        Serial.print("THD I2: "); Serial.println(pzem.thdCurrent2());
        Serial.print("THD I3: "); Serial.println(pzem.thdCurrent3());
    } else {
        Serial.println("Failed to read THD frame");
    }
}

void setup()
{
    Serial.begin(115200);
    pinMode(RS485_DE, OUTPUT);
    digitalWrite(RS485_DE, LOW);

    Serial.println();
    Serial.println("PZEM004Tv30 3-frame test starting...");

    // Note: For ESP32 the PZEM constructor calls port.begin(...)
    // so do not call meterSerial.begin() here; the library will initialize it.
    delay(200);
}

void loop()
{
    // 1) Electrical (use public getters which call updateValues internally)
    Serial.println("Requesting Electrical values...");
    printElectrical();

    delay(100);

    // 2) Energy & Frequency
    Serial.println("Requesting Energy/Frequency values...");
    printEnergyFreq();

    delay(100);

    // 3) THD
    Serial.println("Requesting THD values...");
    printTHD();

    Serial.println("--- Cycle complete ---\n");
    delay(5000);
}