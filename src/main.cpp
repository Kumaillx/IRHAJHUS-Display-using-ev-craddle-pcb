#include <Arduino.h>
#include <PZEM004Tv30.h>

#define TXD1 17
#define RXD1 16
#define RS485_DE 4

HardwareSerial modbus(1);

/* ---------- CRC16 (Modbus RTU) ---------- */
uint16_t modbusCRC(uint8_t *buf, uint8_t len)
{
  uint16_t crc = 0xFFFF;
  for (uint8_t pos = 0; pos < len; pos++) {
    crc ^= (uint16_t)buf[pos];
    for (int i = 8; i != 0; i--) {
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

/* ---------- Read INT (1 register) ---------- */
bool readInt(uint16_t reg, int16_t &value)
{
  uint8_t tx[8];

  tx[0] = 0x01;          // Slave ID
  tx[1] = 0x03;          // Read Holding Registers
  tx[2] = reg >> 8;
  tx[3] = reg & 0xFF;
  tx[4] = 0x00;
  tx[5] = 0x01;          // 1 register

  uint16_t crc = modbusCRC(tx, 6);
  tx[6] = crc & 0xFF;
  tx[7] = crc >> 8;

  /* ---- Transmit ---- */
  digitalWrite(RS485_DE, HIGH);
  delayMicroseconds(100);
  modbus.write(tx, 8);
  modbus.flush();
  delayMicroseconds(100);
  digitalWrite(RS485_DE, LOW);

  /* ---- Receive ---- */
  uint8_t rx[7];
  uint8_t idx = 0;
  unsigned long start = millis();

  while (millis() - start < 500) {
    if (modbus.available()) {
      rx[idx++] = modbus.read();
      if (idx >= 7) break;
    }
  }

  /* ---- Validate frame ---- */
  if (idx != 7) return false;
  if (rx[0] != 0x01 || rx[1] != 0x03 || rx[2] != 0x02) return false;

  uint16_t crcRx = rx[5] | (rx[6] << 8);
  uint16_t crcCalc = modbusCRC(rx, 5);
  if (crcRx != crcCalc) return false;

  /* ---- Decode int ---- */
  value = (rx[3] << 8) | rx[4];
  return true;
}

/* ---------- Read FLOAT (2 registers) ---------- */
bool readFloat(uint16_t reg, float &value)
{
  uint8_t tx[8];

  tx[0] = 0x01;          // Slave ID
  tx[1] = 0x03;          // Read Holding Registers
  tx[2] = reg >> 8;
  tx[3] = reg & 0xFF;
  tx[4] = 0x00;
  tx[5] = 0x02;          // 2 registers

  uint16_t crc = modbusCRC(tx, 6);
  tx[6] = crc & 0xFF;
  tx[7] = crc >> 8;

  /* ---- Transmit ---- */
  digitalWrite(RS485_DE, HIGH);
  delayMicroseconds(100);
  modbus.write(tx, 8);
  modbus.flush();
  delayMicroseconds(100);
  digitalWrite(RS485_DE, LOW);

  /* ---- Receive ---- */
  uint8_t rx[9];
  uint8_t idx = 0;
  unsigned long start = millis();

  while (millis() - start < 500) {
    if (modbus.available()) {
      rx[idx++] = modbus.read();
      if (idx >= 9) break;
    }
  }

  /* ---- Validate frame ---- */
  if (idx != 9) return false;
  if (rx[0] != 0x01 || rx[1] != 0x03 || rx[2] != 0x04) return false;

  uint16_t crcRx = rx[7] | (rx[8] << 8);
  uint16_t crcCalc = modbusCRC(rx, 7);
  if (crcRx != crcCalc) return false;

  /* ---- Decode float (ABCD) ---- */
  uint32_t raw =
    ((uint32_t)rx[3] << 24) |
    ((uint32_t)rx[4] << 16) |
    ((uint32_t)rx[5] << 8)  |
    ((uint32_t)rx[6]);

  memcpy(&value, &raw, sizeof(float));
  return true;
}

void setup()
{
  Serial.begin(115200);

  pinMode(RS485_DE, OUTPUT);
  digitalWrite(RS485_DE, LOW);

  modbus.begin(9600, SERIAL_8E1, RXD1, TXD1);

  Serial.println("IRHAJHUS Meter Modbus RTU Started");
}

void Voltage(uint16_t V1, uint16_t V2, uint16_t V3)
{
  float voltage;
  
  // Read Voltage L1
  if (readFloat(V1, voltage)) {
    Serial.printf("Voltage L1 : %.2f V\n", voltage);
  } else {
    Serial.println("Voltage L1 read failed");
    return;
  }
  delay(10);
// Read Voltage L2
  if (readFloat(V2, voltage)) {
    Serial.printf("Voltage L2 : %.2f V\n", voltage);
  } else {
    Serial.println("Voltage L2 read failed");
    return;       
  }
  delay(10);
// Read Voltage L3
  if (readFloat(V3, voltage)) {
    Serial.printf("Voltage L3 : %.2f V\n", voltage);
  } else {
    Serial.println("Voltage L3 read failed");
  }
}


void Current(uint16_t I1, uint16_t I2, uint16_t I3)
{
  float current;
  
  // Read Current L1
  if (readFloat(I1, current)) {
    Serial.printf("Current L1 : %.3f A\n", current);
  } else {
    Serial.println("Current L1 read failed");
    return;
  }
  delay(10);
  // Read Current L2
  if (readFloat(I2,current))
  {
    Serial.printf("Current L2 : %.3f A\n", current);
  } else {
    Serial.println("Current L2 read failed");
    return;
  }
  delay(10);
  // Read Current L3
  if (readFloat(I3,current))
  {
    Serial.printf("Current L3 : %.3f A\n", current);
  } else {
    Serial.println("Current L3 read failed");
  }
  
}

void Distortion(uint16_t V1, uint16_t V2, uint16_t V3, uint16_t I1, uint16_t I2, uint16_t I3)
{
    int16_t thd;
    if (readInt(V1, thd)) {
        Serial.printf("L1 Voltage THD: %.2f %%\n", thd * 0.01);
    } else {
        Serial.println("L1 Voltage THD read failed");
    }
    delay(10);
    if (readInt(V2, thd)) {
        Serial.printf("L2 Voltage THD: %.2f %%\n", thd * 0.01);
    } else {
        Serial.println("L2 Voltage THD read failed");
    }
    delay(10);
    if (readInt(V3, thd)) {
        Serial.printf("L3 Voltage THD: %.2f %%\n", thd * 0.01);
    } else {
        Serial.println("L3 Voltage THD read failed");
    }
    delay(10);
    if (readInt(I1, thd)) {
        Serial.printf("L1 Current THD: %.2f %%\n", thd * 0.01);
    } else {
        Serial.println("L1 Current THD read failed");
    }
    delay(10);
    if (readInt(I2, thd)) {
        Serial.printf("L2 Current THD: %.2f %%\n", thd * 0.01);
    } else {
        Serial.println("L2 Current THD read failed");
    }
    delay(10);
    if (readInt(I3, thd)) {
        Serial.printf("L3 Current THD: %.2f %%\n", thd * 0.01);
    } else {
        Serial.println("L3 Current THD read failed");
    }
}

void loop()
{
  float power, pf;

  Serial.println("---- Meter Readings ----");
  // Voltage Values
  Voltage(0x0006, 0x0008, 0x000A);
  delay(10);
// Current Values
  Current(0x0012, 0x0014, 0x0016);
  delay(10);
  // THD Values
  Distortion(0x0250, 0x0251, 0x0252, 0x0253, 0x0254, 0x0255);
  delay(10);

  if (readFloat(0x001E, power))
    Serial.printf("Total Power: %.2f kW\n", power);
  else
    Serial.println("Power read failed");

  delay(10);

  if (readFloat(0x002A, pf))
    Serial.printf("Power Factor: %.3f\n", pf);
  else
    Serial.println("PF read failed");

  Serial.println("---------------------------");
  delay(2000);
}