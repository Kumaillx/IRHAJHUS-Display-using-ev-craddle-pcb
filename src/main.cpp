#include <Arduino.h>

#define TXD1 17
#define RXD1 16
#define RS485_DE 4

HardwareSerial modbus(1);

/* ---------- CRC16 ---------- */
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

/* ---------- Read FLOAT ---------- */
bool readFloat(uint16_t reg, float &value)
{
  uint8_t tx[8];

  tx[0] = 0x01;              // Slave ID
  tx[1] = 0x03;              // Read Holding Register
  tx[2] = reg >> 8;
  tx[3] = reg & 0xFF;
  tx[4] = 0x00;
  tx[5] = 0x02;              // 2 registers = float

  uint16_t crc = modbusCRC(tx, 6);
  tx[6] = crc & 0xFF;
  tx[7] = crc >> 8;

  digitalWrite(RS485_DE, HIGH);
  delayMicroseconds(50);
  modbus.write(tx, 8);
  modbus.flush();
  digitalWrite(RS485_DE, LOW);

  uint8_t rx[9];
  uint8_t idx = 0;
  unsigned long start = millis();

  while (millis() - start < 1000) {
    if (modbus.available()) {
      rx[idx++] = modbus.read();
      if (idx >= 9) break;
    }
  }

  if (idx != 9 || rx[1] != 0x03 || rx[2] != 0x04) {
    return false;
  }

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

  Serial.println("IRHAJHUS Meter Reading Started");
}

void loop()
{
  float voltage, current, power, pf;

  if (readFloat(0x0006, voltage))
    Serial.printf("Voltage: %.2f V\n", voltage);
  else
    Serial.println("Voltage read failed");

  if (readFloat(0x0012, current))
    Serial.printf("Current: %.3f A\n", current);
  else
    Serial.println("Current read failed");

  if (readFloat(0x0018, power))
    Serial.printf("Power: %.2f kW\n", power);
  else
    Serial.println("Power read failed");

  if (readFloat(0x002A, pf))
    Serial.printf("Power Factor: %.3f\n", pf);
  else
    Serial.println("PF read failed");

  Serial.println("------------------------");
  delay(2000);
}
