#include <Arduino.h>

#define TXD1 17
#define RXD1 16
#define RS485_DE 4   // change if needed

HardwareSerial modbus(1);

/* CRC16 (Modbus) */
uint16_t modbusCRC(uint8_t *buf, uint8_t len)
{
  uint16_t crc = 0xFFFF;

  for (uint8_t pos = 0; pos < len; pos++) {
    crc ^= (uint16_t)buf[pos];

    for (int i = 8; i != 0; i--) {
      if ((crc & 0x0001) != 0) {
        crc >>= 1;
        crc ^= 0xA001;
      } else {
        crc >>= 1;
      }
    }
  }
  return crc;
}

void setup()
{
  Serial.begin(115200);
  delay(1000);

  pinMode(RS485_DE, OUTPUT);
  digitalWrite(RS485_DE, LOW); // receive mode

  modbus.begin(9600, SERIAL_8E1, RXD1, TXD1);

  Serial.println("RS485 Modbus Test Started");
}

void loop()
{
  uint8_t request[8];

  // Modbus Read Holding Registers
  request[0] = 0x01;     // Slave ID
  request[1] = 0x03;     // Function code
  request[2] = 0x00;     // Start addr Hi
  request[3] = 0x06;     // Start addr Lo
  request[4] = 0x00;     // Quantity Hi
  request[5] = 0x02;     // Quantity Lo

  uint16_t crc = modbusCRC(request, 6);
  request[6] = crc & 0xFF;
  request[7] = crc >> 8;

  // ---- SEND ----
  digitalWrite(RS485_DE, HIGH);
  delayMicroseconds(50);

  modbus.write(request, 8);
  modbus.flush();

  digitalWrite(RS485_DE, LOW);

  Serial.print("TX: ");
  for (int i = 0; i < 8; i++) {
    Serial.printf("%02X ", request[i]);
  }
  Serial.println();

  // ---- RECEIVE ----
  unsigned long start = millis();
  Serial.print("RX: ");

  while (millis() - start < 1000) {
    if (modbus.available()) {
      uint8_t b = modbus.read();
      Serial.printf("%02X ", b);
    }
  }
  Serial.println("\n----------------------");

  delay(2000);
}
