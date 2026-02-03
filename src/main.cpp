#include <Arduino.h>
#include <PZEM004Tv30.h>

// -- Hardware Configuration --

// 1. RX/TX Pins for ESP32 Hardware Serial
#define RXD2 16
#define TXD2 17

// 2. RS485 Enable Pin for Direction Control
#define RS485_EN_PIN 4 // Set to -1 if your converter has automatic direction control

// 3. PZEM Device Address
// The diagnostic search found devices at addresses 1, 2, and 100.
// We will target one of these. Change the address below to read from a different device.
#define PZEM_ADDRESS 1


// Initialize the PZEM object with the specific address you want to read from.
PZEM004Tv30 pzem(Serial2, RXD2, TXD2, RS485_EN_PIN, PZEM_ADDRESS);


void setup() {
  Serial.begin(115200);
  Serial.println("\n----------------------------------------------------------");
  Serial.println("PZEM-004T v3.0 Example - Full Parameter Read");
  Serial.println("Reading data from device at Modbus Address: " + String(PZEM_ADDRESS));
  Serial.println("----------------------------------------------------------");

  // Quick communication test to verify Modbus RTU parameters (baud/parity/address)
  Serial.println("Performing communication test...");
  bool comm = pzem.updateEnergyFreq();
  if (comm) {
    float freq = pzem.frequency();
    Serial.print("Communication OK - meter responds. Frequency: "); Serial.print(freq, 1); Serial.println(" Hz");
  } else {
    Serial.println("No response - check baud/parity/wiring/address");
  }
}

void loop() {
  Serial.println("---------------------------------------");
  Serial.println("Reading from Address: " + String(pzem.getAddress()));

  // Update all data groups from the sensor
  bool elecUpdated = pzem.updateElectrical();
  bool energyUpdated = pzem.updateEnergyFreq();
  bool thdUpdated = pzem.updateTHD();

  if(elecUpdated && energyUpdated && thdUpdated) {
    // Phase 1
    Serial.print("Phase 1: ");
    Serial.print(pzem.voltage_1(), 1); Serial.print("V, ");
    Serial.print(pzem.current_1(), 3); Serial.print("A, ");
    Serial.print(pzem.power_1(), 2); Serial.print("W, ");
    Serial.print(pzem.pf_1(), 2); Serial.print("PF, ");
    Serial.print(pzem.thdVoltage1(), 1); Serial.print("% THDV, ");
    Serial.print(pzem.thdCurrent1(), 1); Serial.println("% THDI");

    // Phase 2
    Serial.print("Phase 2: ");
    Serial.print(pzem.voltage_2(), 1); Serial.print("V, ");
    Serial.print(pzem.current_2(), 3); Serial.print("A, ");
    Serial.print(pzem.power_2(), 2); Serial.print("W, ");
    Serial.print(pzem.pf_2(), 2); Serial.print("PF, ");
    Serial.print(pzem.thdVoltage2(), 1); Serial.print("% THDV, ");
    Serial.print(pzem.thdCurrent2(), 1); Serial.println("% THDI");

    // Phase 3
    Serial.print("Phase 3: ");
    Serial.print(pzem.voltage_3(), 1); Serial.print("V, ");
    Serial.print(pzem.current_3(), 3); Serial.print("A, ");
    Serial.print(pzem.power_3(), 2); Serial.print("W, ");
    Serial.print(pzem.pf_3(), 2); Serial.print("PF, ");
    Serial.print(pzem.thdVoltage3(), 1); Serial.print("% THDV, ");
    Serial.print(pzem.thdCurrent3(), 1); Serial.println("% THDI");
    
    Serial.println();

    // System-wide values
    Serial.print("System: ");
    Serial.print(pzem.frequency(), 1); Serial.print("Hz, ");
    Serial.print(pzem.total_power(), 2); Serial.print("W (TAP), ");
    Serial.print(pzem.energyp(), 3); Serial.print("kWh (EP), ");
    Serial.print(pzem.energyn(), 3); Serial.println("kWh (EN)");
    
  } else {
    // This message will be printed if any of the communication updates fail
    Serial.println("Error reading values from PZEM. Check wiring and device address.");
    if (!elecUpdated) Serial.println(" -> Electrical values failed.");
    if (!energyUpdated) Serial.println(" -> Energy/Frequency values failed.");
    if (!thdUpdated) Serial.println(" -> THD values failed.");
  }

  // Wait 5 seconds before the next read
  delay(5000);
}
