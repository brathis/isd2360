/*
 * FlashUpdate.ino can be used with uploader.py to update the firmware
 * of Nuvoton ChipCorder ISD2360 devices.
 * Copyright (C) 2020 Mathis Dedial
 * 
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#include <ISD2360.h>

#define PIN_LED_UPDATE 8
#define PIN_RDY 9
#define PIN_SSB 10

#define PACKET_HELLO 0x01
#define PACKET_HELLO_ACK 0x02
#define PACKET_ACK 0x03
#define PACKET_ERR 0x04
#define PACKET_STOP 0x05
#define PACKET_RDY 0x06

ISD2360 isd(PIN_RDY, PIN_SSB, false);

byte n_sectors = 0;
byte response = 0;
byte chunk[128] = {0};
size_t bytes_read = 0;

void setup()
{
  Serial.begin(115200);
  pinMode(PIN_LED_UPDATE, OUTPUT);
  digitalWrite(PIN_LED_UPDATE, LOW);

  // set up chip, light up LED to signal when we're ready
  isd.begin();
  isd.reset();
  isd.powerUp();
  digitalWrite(PIN_LED_UPDATE, HIGH);
  Serial.write(PACKET_RDY);

  // handshake
  while (Serial.available() == 0)
    ;
  if (Serial.read() != PACKET_HELLO)
  {
    Serial.write(PACKET_ERR);
    digitalWrite(PIN_LED_UPDATE, LOW);
    return;
  }

  // erase chip, then acknowledge
  isd.eraseChip();

  Serial.write(PACKET_HELLO_ACK);
  while (Serial.available() == 0)
    ;
  n_sectors = Serial.read();
  Serial.write(PACKET_ACK);

  // write sectors
  for (uint8_t i = 0; i < n_sectors; ++i)
  {

    // read sector index
    while (Serial.available() == 0)
      ;
    response = Serial.read();
    if (response != i)
    {
      Serial.write(PACKET_ERR);
      digitalWrite(PIN_LED_UPDATE, LOW);
      return;
    }
    Serial.write(PACKET_ACK);

    // read sector chunks
    for (uint8_t j = 0; j < 8; ++j)
    {
      while (Serial.available() == 0)
        ;
      Serial.readBytes(chunk, 128);

      // write to chip
      isd.flashWrite(i * 0x400UL + j * 0x80UL, chunk, 128);

      Serial.write(PACKET_ACK);
    }
  }

  // phase handshake
  Serial.write(PACKET_STOP);
  while (Serial.available() == 0)
    ;
  response = Serial.read();
  if (response != PACKET_ACK)
  {
    Serial.write(PACKET_ERR);
    digitalWrite(PIN_LED_UPDATE, LOW);
    return;
  }

  // verify
  for (uint8_t i = 0; i < n_sectors; ++i)
  {
    while (Serial.available() == 0)
      ;
    response = Serial.read();
    if (response != i)
    {
      Serial.write(PACKET_ERR);
      digitalWrite(PIN_LED_UPDATE, LOW);
      return;
    }
    Serial.write(PACKET_ACK);

    for (uint8_t j = 0; j < 8; ++j)
    {
      // load sector chunks from chip
      isd.flashRead(i * 0x400UL + j * 0x80UL, chunk, 128);

      Serial.write(chunk, 128);

      while (Serial.available() == 0)
        ;
      response = Serial.read();
      if (response != PACKET_ACK)
      {
        Serial.write(PACKET_ERR);
        digitalWrite(PIN_LED_UPDATE, LOW);
        return;
      }
    }
  }

  // phase handshake
  Serial.write(PACKET_STOP);
  while (Serial.available() == 0)
    ;
  response = Serial.read();
  if (response != PACKET_ACK)
  {
    Serial.write(PACKET_ERR);
    digitalWrite(PIN_LED_UPDATE, LOW);
    return;
  }

  // cleanup
  isd.powerDown();
  digitalWrite(PIN_LED_UPDATE, LOW);
}

void loop()
{
}
