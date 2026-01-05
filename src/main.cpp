#include <Arduino.h>

#define TXD1 17
#define RXD1 16
#define RS485_DE 4

HardwareSerial modbus(1);

/* ---------- CRC16 (Modbus RTU) ---------- */
uint16_t modbusCRC(uint8_t *buf, uint16_t len)
{
  uint16_t crc = 0xFFFF;
  for (uint16_t pos = 0; pos < len; pos++) {
    crc ^= buf[pos];
    for (uint8_t i = 0; i < 8; i++) {
      if (crc & 0x0001) {
        crc >>= 1;
        crc ^= 0xA001;
      } else {
        crc >>= 1;
      }
    }
  }
  return crc;
}

/* ---------- Read multiple holding registers ---------- */
bool readRegisters(uint16_t startReg, uint16_t count, uint16_t *dest)
{
  uint8_t tx[8];

  tx[0] = 0x01;
  tx[1] = 0x04;
  tx[2] = startReg >> 8;
  tx[3] = startReg & 0xFF;
  tx[4] = count >> 8;
  tx[5] = count & 0xFF;

  uint16_t crc = modbusCRC(tx, 6);
  tx[6] = crc & 0xFF;
  tx[7] = crc >> 8;

  digitalWrite(RS485_DE, HIGH);
  delayMicroseconds(100);
  modbus.write(tx, 8);
  modbus.flush();
  delayMicroseconds(100);
  digitalWrite(RS485_DE, LOW);

  uint16_t expected = 5 + count * 2;
  uint8_t rx[128];
  uint16_t idx = 0;
  unsigned long start = millis();

  while (millis() - start < 500) {
    if (modbus.available()) {
      rx[idx++] = modbus.read();
      if (idx >= expected) break;
    }
  }

  if (idx != expected) return false;

  uint16_t crcRx = rx[idx - 2] | (rx[idx - 1] << 8);
  uint16_t crcCalc = modbusCRC(rx, idx - 2);
  if (crcRx != crcCalc) return false;

  for (uint16_t i = 0; i < count; i++) {
    dest[i] = (rx[3 + i * 2] << 8) | rx[4 + i * 2];
  }

  return true;
}

/* ---------- Register → Float (ABCD) ---------- */
float regToFloat(uint16_t hi, uint16_t lo)
{
  uint32_t raw = ((uint32_t)hi << 16) | lo;
  float f;
  memcpy(&f, &raw, sizeof(float));
  return f;
}

void setup()
{
  Serial.begin(115200);

  pinMode(RS485_DE, OUTPUT);
  digitalWrite(RS485_DE, LOW);

  modbus.begin(9600, SERIAL_8E1, RXD1, TXD1);

  Serial.println("IRHAJHUS Meter Modbus RTU (Optimized)");
}

void loop()
{
  /* ================= FRAME 1: Electrical Parameters ================= */
  // Registers: // 0x0006 → Voltage L1 (2) // 0x0008 → Voltage L2 (2) // 0x000A → Voltage L3 (2) // 0x0012 → Current L1 (2) // 0x0014 → Current L2 (2) // 0x0016 → Current L3 (2) // TOTAL = 12 registers

  uint16_t regs[12];

  Serial.println("---- Electrical Parameters ----");

  if (readRegisters(0x0006, 12, regs)) {

    float V1 = regToFloat(regs[0], regs[1]);
    float V2 = regToFloat(regs[2], regs[3]);
    float V3 = regToFloat(regs[4], regs[5]);

    float I1 = regToFloat(regs[6], regs[7]);
    float I2 = regToFloat(regs[8], regs[9]);
    float I3 = regToFloat(regs[10], regs[11]);

    Serial.printf("Voltage L1: %.2f V\n", V1);
    Serial.printf("Voltage L2: %.2f V\n", V2);
    Serial.printf("Voltage L3: %.2f V\n", V3);

    Serial.printf("Current L1: %.3f A\n", I1);
    Serial.printf("Current L2: %.3f A\n", I2);
    Serial.printf("Current L3: %.3f A\n", I3);

  } else {
    Serial.println("Parameter frame read FAILED");
  }

  /* ================= FRAME 2: THD Values ================= */
  // 0250–0255 (6 registers, INT16, scale 0.01%)

  uint16_t thdRegs[6];

  Serial.println("---- THD Values ----");

  if (readRegisters(0x0250, 6, thdRegs)) {  // 6x 2 = 12 bytes + 5  

    Serial.printf("L1 Voltage THD: %.2f %%\n", thdRegs[0] * 0.01);
    Serial.printf("L2 Voltage THD: %.2f %%\n", thdRegs[1] * 0.01);
    Serial.printf("L3 Voltage THD: %.2f %%\n", thdRegs[2] * 0.01);

    Serial.printf("L1 Current THD: %.2f %%\n", thdRegs[3] * 0.01);
    Serial.printf("L2 Current THD: %.2f %%\n", thdRegs[4] * 0.01);
    Serial.printf("L3 Current THD: %.2f %%\n", thdRegs[5] * 0.01);

  } else {
    Serial.println("THD frame read FAILED");
  }

  Serial.println("--------------------------------\n");
  delay(2000);
}