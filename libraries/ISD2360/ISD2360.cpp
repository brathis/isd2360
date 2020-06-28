/*
 * The ISD2360 Arduino library implements SPI communication 
 * with Nuvoton ChipCorder ISD2360 devices.
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

#include "ISD2360.h"

ISD2360::ISD2360(uint8_t pinRdy, uint8_t pinSsb, bool debug = false)
{
  this->pinRdy = pinRdy;
  this->pinSsb = pinSsb;
  this->debug = debug;
}

void ISD2360::begin()
{
  pinMode(this->pinRdy, INPUT);
  pinMode(11, OUTPUT); // MOSI
  digitalWrite(11, LOW);
  pinMode(13, OUTPUT); // SCK
  digitalWrite(13, HIGH);
  pinMode(this->pinSsb, OUTPUT);
  digitalWrite(this->pinSsb, HIGH);

  // MSB first
  // Master
  // Mode 3
  // 125 kHz
  SPCR = 0b01011111;

  if (this->debug)
  {
    Serial.begin(115200);
    Serial.println("ISD2360 initialized.");
  }
}

bool ISD2360::getDebug()
{
  return this->debug;
}

void ISD2360::setDebug(bool debug)
{
  this->debug = debug;
}

uint8_t ISD2360::readStatus()
{
  this->spiTransfer(ISD2360_CMD_READ_STATUS, this->data, 1);

  if (this->debug)
  {
    Serial.println("[+] READ_STATUS");

    Serial.print("\tStatus = 0x");
    Serial.println(this->deviceStatus, HEX);

    Serial.print("\t\tPD = ");
    Serial.println((this->deviceStatus & _BV(ISD2360_DEVICE_STATUS_PD)) >> ISD2360_DEVICE_STATUS_PD, BIN);
    Serial.print("\t\tDBUF_RDY = ");
    Serial.println((this->deviceStatus & _BV(ISD2360_DEVICE_STATUS_DBUF_RDY)) >> ISD2360_DEVICE_STATUS_DBUF_RDY, BIN);
    Serial.print("\t\tINT = ");
    Serial.println((this->deviceStatus & _BV(ISD2360_DEVICE_STATUS_INT)) >> ISD2360_DEVICE_STATUS_INT, BIN);
    Serial.print("\t\tCH2_BSY = ");
    Serial.println((this->deviceStatus & _BV(ISD2360_DEVICE_STATUS_CH2_BSY)) >> ISD2360_DEVICE_STATUS_CH2_BSY, BIN);
    Serial.print("\t\tCH1_BSY = ");
    Serial.println((this->deviceStatus & _BV(ISD2360_DEVICE_STATUS_CH1_BSY)) >> ISD2360_DEVICE_STATUS_CH1_BSY, BIN);
    Serial.print("\t\tCH0_BSY = ");
    Serial.println((this->deviceStatus & _BV(ISD2360_DEVICE_STATUS_CH0_BSY)) >> ISD2360_DEVICE_STATUS_CH0_BSY, BIN);
    Serial.print("\t\tDIG_BSY = ");
    Serial.println((this->deviceStatus & _BV(ISD2360_DEVICE_STATUS_DIG_BSY)) >> ISD2360_DEVICE_STATUS_DIG_BSY, BIN);

    Serial.print("\tInterrupt Status = 0x");
    Serial.println(this->response[0], HEX);
  }

  return this->deviceStatus;
}

uint8_t ISD2360::readInterrupt()
{
  this->spiTransfer(ISD2360_CMD_READ_INT, this->data, 1);

  if (this->debug)
  {
    Serial.println("[+] READ_INT");

    Serial.print("\tTALARM_INT = ");
    Serial.println((this->response[0] & _BV(ISD2360_INTERRUPT_STATUS_TALARM_INT)) >> ISD2360_INTERRUPT_STATUS_TALARM_INT, BIN);
    Serial.print("\tMPT_ERR = ");
    Serial.println((this->response[0] & _BV(ISD2360_INTERRUPT_STATUS_MPT_ERR)) >> ISD2360_INTERRUPT_STATUS_MPT_ERR, BIN);
    Serial.print("\tWR_FIN = ");
    Serial.println((this->response[0] & _BV(ISD2360_INTERRUPT_STATUS_WR_FIN)) >> ISD2360_INTERRUPT_STATUS_WR_FIN, BIN);
    Serial.print("\tCMD_ERR = ");
    Serial.println((this->response[0] & _BV(ISD2360_INTERRUPT_STATUS_CMD_ERR)) >> ISD2360_INTERRUPT_STATUS_CMD_ERR, BIN);
    Serial.print("\tOVF_ERR = ");
    Serial.println((this->response[0] & _BV(ISD2360_INTERRUPT_STATUS_OVF_ERR)) >> ISD2360_INTERRUPT_STATUS_OVF_ERR, BIN);
    Serial.print("\tCH2_FIN = ");
    Serial.println((this->response[0] & _BV(ISD2360_INTERRUPT_STATUS_CH2_FIN)) >> ISD2360_INTERRUPT_STATUS_CH2_FIN, BIN);
    Serial.print("\tCH1_FIN = ");
    Serial.println((this->response[0] & _BV(ISD2360_INTERRUPT_STATUS_CH1_FIN)) >> ISD2360_INTERRUPT_STATUS_CH1_FIN, BIN);
    Serial.print("\tCH0_FIN = ");
    Serial.println((this->response[0] & _BV(ISD2360_INTERRUPT_STATUS_CH0_FIN)) >> ISD2360_INTERRUPT_STATUS_CH0_FIN, BIN);
  }

  return this->response[0];
}

bool ISD2360::readId()
{
  this->spiTransfer(ISD2360_CMD_READ_ID, this->data, 4);

  if (this->debug)
  {
    Serial.println("[+] READ_ID");

    Serial.print("\tPART_ID = 0x");
    Serial.println(this->response[0], HEX);
    Serial.print("\tMAN_ID = 0x");
    Serial.println(this->response[1], HEX);
    Serial.print("\tMEM_TYPE = 0x");
    Serial.println(this->response[2], HEX);
    Serial.print("\tDEV_ID = 0x");
    Serial.println(this->response[3], HEX);
  }

  if (this->response[0] != ISD2360_EXP_PART_ID)
  {
    if (this->debug)
    {
      Serial.print("\tPART_ID mismatch. Expected ");
      Serial.print(ISD2360_EXP_PART_ID, HEX);
      Serial.print(", got ");
      Serial.println(this->response[0], HEX);
    }
    return false;
  }

  if (this->response[1] != ISD2360_EXP_MAN_ID)
  {
    if (this->debug)
    {
      Serial.print("\tMAN_ID mismatch. Expected ");
      Serial.print(ISD2360_EXP_MAN_ID, HEX);
      Serial.print(", got ");
      Serial.println(this->response[1], HEX);
    }
    return false;
  }

  if (this->response[2] != ISD2360_EXP_MEM_TYPE)
  {
    if (this->debug)
    {
      Serial.print("\tMEM_TYPE mismatch. Expected ");
      Serial.print(ISD2360_EXP_MEM_TYPE, HEX);
      Serial.print(", got ");
      Serial.println(this->response[2], HEX);
    }
    return false;
  }

  if (this->response[3] != ISD2360_EXP_DEV_ID)
  {
    if (this->debug)
    {
      Serial.print("\tDEV_ID mismatch. Expected ");
      Serial.print(ISD2360_EXP_DEV_ID, HEX);
      Serial.print(", got ");
      Serial.println(this->response[3], HEX);
    }
    return false;
  }

  if (this->debug)
  {
    Serial.println("\tOK!");
  }

  return true;
}

void ISD2360::powerUp()
{
  this->spiTransfer(ISD2360_CMD_PWR_UP, this->data, 0);

  if (this->debug)
  {
    Serial.println("[+] PWR_UP");
    Serial.println("\tWaiting for device to power up...");
  }

  // wait for PD to go low
  while (this->deviceStatus & _BV(ISD2360_DEVICE_STATUS_PD))
  {
    this->spiTransfer(ISD2360_CMD_READ_STATUS, this->data, 1);
  }
  if (this->debug)
  {
    Serial.println("\t\tPD -> 0");
  }

  // wait for DBUF_RDY to go high
  while (!(this->deviceStatus & _BV(ISD2360_DEVICE_STATUS_DBUF_RDY)))
  {
    this->spiTransfer(ISD2360_CMD_READ_STATUS, this->data, 1);
  }
  if (this->debug)
  {
    Serial.println("\t\tDBUF_RDY -> 1");
    Serial.println("\tWaiting for Power-Up Voice Macro to complete...");
  }

  // poll status until CHx_BSY goes low
  while (this->deviceStatus & (_BV(ISD2360_DEVICE_STATUS_CH2_BSY) | _BV(ISD2360_DEVICE_STATUS_CH1_BSY) | _BV(ISD2360_DEVICE_STATUS_CH0_BSY)))
  {
    this->spiTransfer(ISD2360_CMD_READ_STATUS, this->data, 1);
  }

  // enable RDY/BSYB functionality on GPIO4 and disable all other GPIO triggers
  this->writeConfigurationRegister(0x1E, 0);
  this->writeConfigurationRegister(0x1F, _BV(4));

  if (this->debug)
  {
    Serial.println("\tDone.");
  }
}

void ISD2360::powerDown()
{
  this->spiTransfer(ISD2360_CMD_PWR_DN, this->data, 1);
  while (!(this->readStatus() & _BV(ISD2360_DEVICE_STATUS_PD)))
    ;
  if (this->debug)
  {
    Serial.println("[+] PWR_DN");
  }
}

void ISD2360::reset()
{
  this->spiTransfer(ISD2360_CMD_RESET, this->data, 0);

  if (this->debug)
  {
    Serial.println("[+] RESET");
  }
}

void ISD2360::writeConfigurationRegister(uint8_t reg, const uint8_t *data, size_t dataLen)
{
  if (dataLen > ISD2360_BUF_SIZE - 1)
  {
    return;
  }

  // fill the data buffer
  // first byte is the target register
  // remaining bytes is data to be written to target register and potential subsequent registers
  this->data[0] = reg;
  for (uint8_t i = 0; i < dataLen; ++i)
  {
    this->data[i + 1] = data[i];
  }

  this->spiTransfer(ISD2360_CMD_WR_CFG_REG, this->data, dataLen + 1);
}

void ISD2360::writeConfigurationRegister(uint8_t reg, const uint8_t val)
{
  const uint8_t data[] = {
      (const uint8_t)reg,
      val,
  };
  this->spiTransfer(ISD2360_CMD_WR_CFG_REG, data, 2);

  if (this->debug)
  {
    Serial.print("[+] WR_CFG_REG(0x");
    Serial.print(reg, HEX);
    Serial.print(", 0x");
    Serial.print(val, HEX);
    Serial.println(")");
  }
}

void ISD2360::readConfigurationRegister(uint8_t reg, uint8_t *data, size_t dataLen)
{
  if (dataLen > ISD2360_BUF_SIZE - 1)
  {
    return;
  }

  this->data[0] = reg;

  this->spiTransfer(ISD2360_CMD_RD_CFG_REG, this->data, dataLen + 1);

  for (uint8_t i = 0; i < dataLen; ++i)
  {
    data[i] = this->response[i + 1];
  }
}

uint8_t ISD2360::readConfigurationRegister(uint8_t reg)
{
  this->data[0] = reg;

  this->spiTransfer(ISD2360_CMD_RD_CFG_REG, this->data, 2);

  if (this->debug)
  {
    Serial.print("[+] RD_CFG_REG(0x");
    Serial.print(reg, HEX);
    Serial.println(")");
    Serial.print("\t0x");
    Serial.println(this->response[1], HEX);
  }

  return this->response[1];
}

void ISD2360::flashWrite(uint32_t addr, const uint8_t *data, size_t dataLen)
{
  if (dataLen > ISD2360_BUF_SIZE - 3)
  {
    return;
  }

  this->loadAddress(addr);

  // payload
  for (uint8_t i = 0; i < dataLen; ++i)
  {
    this->data[i + 3] = data[i];
  }

  this->spiTransfer(ISD2360_CMD_DIG_WRITE, this->data, dataLen + 3);

  // wait for DIG_BSY to go low
  while (this->readStatus() & _BV(ISD2360_DEVICE_STATUS_DIG_BSY))
    ;

  // wait for WR_FIN interrupt
  while (!(this->readInterrupt() & _BV(ISD2360_INTERRUPT_STATUS_WR_FIN)))
    ;

  if (this->debug)
  {
    Serial.print("[+] DIG_WRITE(0x");
    Serial.print(addr, HEX);
    Serial.print(", ");
    Serial.print(dataLen);
    Serial.println(")");
  }
}

void ISD2360::flashRead(uint32_t addr, uint8_t *buf, size_t dataLen)
{
  if (dataLen > ISD2360_BUF_SIZE - 3)
  {
    return;
  }

  this->loadAddress(addr);

  this->spiTransfer(ISD2360_CMD_DIG_READ, this->data, 3 + dataLen);

  for (uint8_t i = 0; i < dataLen; ++i)
  {
    buf[i] = this->response[i + 3];
  }

  if (this->debug)
  {
    Serial.print("[+] DIG_READ(0x");
    Serial.print(addr, HEX);
    Serial.print(", ");
    Serial.print(dataLen);
    Serial.println(")");
  }
}

uint8_t ISD2360::flashRead(uint32_t addr)
{
  this->loadAddress(addr);

  this->spiTransfer(ISD2360_CMD_DIG_READ, this->data, 4);

  if (this->debug)
  {
    Serial.println("[+] DIG_READ");
    Serial.print("\t0x");
    Serial.println(this->response[3], HEX);
  }

  return this->response[3];
}

void ISD2360::flashDump()
{
  bool tempDebug = this->debug;
  this->debug = false;

  for (uint32_t sector = 0; sector < 256; ++sector)
  {
    Serial.println("");
    Serial.print("Sector 0x");
    Serial.println(sector, HEX);

    // print each sector as 64 rows of 16 bytes each
    for (uint32_t row = 0; row < 64UL; ++row)
    {
      Serial.println("");
      for (uint32_t col = 0; col < 16UL; ++col)
      {
        Serial.print(this->flashRead(1024UL * sector + 16UL * row + col), HEX);
        if (col < 15UL)
        {
          Serial.print("\t");
        }
      }
    }
  }

  this->debug = tempDebug;
}

void ISD2360::spiAudioWrite(const uint8_t *data, size_t dataLen)
{
  if (dataLen > ISD2360_BUF_SIZE - 1)
  {
    return;
  }

  this->spiTransfer(ISD2360_CMD_SPI_SND_DEC, data, dataLen);
}

void ISD2360::playVoicePrompt(uint16_t index)
{
  this->data[0] = (index & 0xff00) >> 8;
  this->data[1] = (index & 0x00ff) >> 0;
  this->spiTransfer(ISD2360_CMD_PLAY_VP, this->data, 2);
  // wait for playback to complete
  while (!(this->readInterrupt() & 0b111))
    ;
  if (this->debug)
  {
    Serial.print("[+] PLAY_VP(");
    Serial.print(index);
    Serial.println(")");
  }
}

void ISD2360::playbackStop()
{
  this->spiTransfer(ISD2360_CMD_STOP, this->data, 0);
  if (this->debug)
  {
    Serial.println("[+] STOP");
  }
}

void ISD2360::eraseChip()
{
  this->data[0] = 0x01;
  this->spiTransfer(ISD2360_CMD_CHIP_ERASE, this->data, 1);
  while (this->readStatus() & _BV(ISD2360_DEVICE_STATUS_DIG_BSY))
    ;
  if (this->debug)
  {
    Serial.println("[+] CHIP_ERASE");
  }
}

uint32_t ISD2360::readChecksum(uint32_t endAddr)
{
  // reset checksum
  this->writeConfigurationRegister(ISD2360_REG_CHECKSUM_RESET, _BV(ISD2360_REG_CHECKSUM_RESET_BIT));
  this->writeConfigurationRegister(ISD2360_REG_CHECKSUM_RESET, 0);

  this->loadAddress(endAddr);
  while (this->readStatus() & _BV(ISD2360_DEVICE_STATUS_DIG_BSY))
    ;
  uint32_t checksum = ((uint32_t)this->readConfigurationRegister(0x13) << 24) | ((uint32_t)this->readConfigurationRegister(0x12) << 16) | ((uint32_t)this->readConfigurationRegister(0x11) << 8) | (uint32_t)this->readConfigurationRegister(0x10);
  if (this->debug)
  {
    Serial.print("[+] CHECKSUM(0x");
    Serial.print(endAddr, HEX);
    Serial.println(")");
    Serial.print("\t0x");
    Serial.println(checksum, HEX);
  }
  return checksum;
}

void ISD2360::configureSignalPath(SignalPath signalPath)
{
  uint8_t val;

  switch (signalPath)
  {
  case PLAYBACK_TO_SPEAKER:
    // internal memory to speaker
    val = _BV(ISD2360_REG_PATH_CONTROL_DECODE) | _BV(ISD2360_REG_PATH_CONTROL_PWM_OUT);
    break;
  case SPI_PLAYBACK:
    // internal memory to SPI
    val = _BV(ISD2360_REG_PATH_CONTROL_DECODE) | _BV(ISD2360_REG_PATH_CONTROL_SPI_OUT);
    break;
  case SPI_DECODE_TO_SPEAKER:
    // SPI to speaker
    val = _BV(ISD2360_REG_PATH_CONTROL_DECODE) | _BV(ISD2360_REG_PATH_CONTROL_SPI_IN) | _BV(ISD2360_REG_PATH_CONTROL_PWM_OUT);
    break;
  default:
    return;
  }

  this->writeConfigurationRegister(ISD2360_REG_PATH_CONTROL, val);
}

void ISD2360::spiTransfer(uint8_t cmd, const uint8_t *data, size_t dataLen)
{
  this->spiTransfer(cmd, data, dataLen, this->response);
}

void ISD2360::spiTransfer(uint8_t cmd, const uint8_t *data, size_t dataLen, uint8_t *response)
{
  if (dataLen > ISD2360_BUF_SIZE)
  {
    return;
  }

  digitalWrite(this->pinSsb, LOW);

  // write command and read status
  SPDR = cmd;
  while (!(SPSR & _BV(SPIF)))
    ;
  this->deviceStatus = SPDR;

  for (uint8_t i = 0; i < dataLen; ++i)
  {
    while (digitalRead(this->pinRdy) == LOW)
      ;
    SPDR = data[i];
    while (!(SPSR & _BV(SPIF)))
      ;
    response[i] = SPDR;
  }
  digitalWrite(this->pinSsb, HIGH);
}

void ISD2360::loadAddress(uint32_t addr)
{
  this->data[0] = (addr & 0x00ff0000UL) >> 16;
  this->data[1] = (addr & 0x0000ff00UL) >> 8;
  this->data[2] = (addr & 0x000000ffUL) >> 0;
}
