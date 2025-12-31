#include <ModbusMaster.h>

// ===== PIN DEFINITIONS (FROM YOUR SCHEMATIC) =====
#define RS485_EN 4      // RE + DE (MAX485)
#define TXD1     17     // UART1 TX
#define RXD1     16     // UART1 RX

ModbusMaster node;

// ===== RS485 DIRECTION CONTROL =====
void preTransmission() {
  digitalWrite(RS485_EN, HIGH);   // transmit
}

void postTransmission() {
  digitalWrite(RS485_EN, LOW);    // receive
}

void setup() {
  Serial.begin(115200);
  Serial.println("IRHAJHUS Modbus Reader Starting...");

  pinMode(RS485_EN, OUTPUT);
  digitalWrite(RS485_EN, LOW);

  // UART1 for RS485
  Serial1.begin(9600, SERIAL_8N1, RXD1, TXD1);

  // Modbus slave ID = 1
  node.begin(1, Serial1);
  node.preTransmission(preTransmission);
  node.postTransmission(postTransmission);
}

void loop() {
  uint8_t result;


  result = node.readHoldingRegisters(0x0006, 6);

  if (result == node.ku8MBSuccess) {

    float voltage = node.getResponseBuffer(0) / 10.0;
    float current = node.getResponseBuffer(1) / 100.0;

    uint32_t powerRaw =
      ((uint32_t)node.getResponseBuffer(2) << 16) |
       node.getResponseBuffer(3);
    float power = powerRaw / 10.0;

    float pf = node.getResponseBuffer(4) / 100.0;

    Serial.println("------ IRHAJHUS DATA ------");
    Serial.print("Voltage : "); Serial.print(voltage); Serial.println(" V");
    Serial.print("Current : "); Serial.print(current); Serial.println(" A");
    Serial.print("Power   : "); Serial.print(power);   Serial.println(" W");
    Serial.print("PF      : "); Serial.println(pf);
    Serial.println("---------------------------");
  }
  else {
    Serial.print("Modbus error: ");
    Serial.println(result);
  }

  delay(1000);
}
