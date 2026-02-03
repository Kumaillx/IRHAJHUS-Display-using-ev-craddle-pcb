#include <Arduino.h>

// ===== UART PINS (change if needed) =====
#define RX_PIN 16
#define TX_PIN 17
// =======================================

void setup()
{
    Serial.begin(115200);
    delay(1000);
    Serial.println("ABB M1M30 Average Voltage Reader");

    Serial2.begin(19200, SERIAL_8E1, RX_PIN, TX_PIN);
}

void loop()
{
    uint8_t request[8];

    // -------- Modbus Request --------
    request[0] = 0x01;      // Slave ID
    request[1] = 0x03;      // Read Holding Registers
    request[2] = 0x5B;      // Start address high (0x5BDC)
    request[3] = 0xDC;      // Start address low
    request[4] = 0x00;      // Quantity high
    request[5] = 0x06;      // Quantity low (6 registers)

    // -------- Inline CRC16 --------
    uint16_t crc = 0xFFFF;
    for (int i = 0; i < 6; i++)
    {
        crc ^= request[i];
        for (int b = 0; b < 8; b++)
        {
            if (crc & 0x0001)
            {
                crc >>= 1;
                crc ^= 0xA001;
            }
            else
                crc >>= 1;
        }
    }

    request[6] = crc & 0xFF;        // CRC Low
    request[7] = (crc >> 8) & 0xFF; // CRC High
    // --------------------------------

    Serial2.write(request, 8);
    Serial2.flush();

    delay(250);

    // -------- Read Response --------
    if (Serial2.available() >= 17)
    {
        uint8_t response[17];
        Serial2.readBytes(response, 17);

        if (response[1] == 0x03)
        {
            uint16_t rawL1 = (response[3] << 8) | response[4];
            uint16_t rawL2 = (response[5] << 8) | response[6];
            uint16_t rawL3 = (response[7] << 8) | response[8];

            float vL1 = rawL1 / 10.0;
            float vL2 = rawL2 / 10.0;
            float vL3 = rawL3 / 10.0;

            Serial.print("Avg Voltage L1: ");
            Serial.print(vL1);
            Serial.println(" V");

            Serial.print("Avg Voltage L2: ");
            Serial.print(vL2);
            Serial.println(" V");

            Serial.print("Avg Voltage L3: ");
            Serial.print(vL3);
            Serial.println(" V");

            Serial.println("---------------------------");
        }
        else
        {
            Serial.println("Modbus exception response");
        }
    }
    else
    {
        Serial.println("No response from meter");
    }

    delay(3000);
}
