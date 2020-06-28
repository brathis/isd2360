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

#ifndef _H_ISD2360_
#define _H_ISD2360_

#include "Arduino.h"

#define ISD2360_CMD_READ_ID                   0x48
#define ISD2360_CMD_READ_STATUS               0x40
#define ISD2360_CMD_READ_INT                  0x46
#define ISD2360_CMD_PWR_UP                    0x10
#define ISD2360_CMD_PWR_DN                    0x12
#define ISD2360_CMD_RESET                     0x14
#define ISD2360_CMD_WR_CFG_REG                0xB8
#define ISD2360_CMD_RD_CFG_REG                0xBA
#define ISD2360_CMD_DIG_WRITE                 0xA0
#define ISD2360_CMD_DIG_READ                  0xA2
#define ISD2360_CMD_SPI_SND_DEC               0xC0
#define ISD2360_CMD_PLAY_VP                   0xA6
#define ISD2360_CMD_STOP                      0x2A
#define ISD2360_CMD_CHIP_ERASE                0x26
#define ISD2360_CMD_CHECKSUM                  0xF2

// Device Status Register
#define ISD2360_DEVICE_STATUS_PD              7
#define ISD2360_DEVICE_STATUS_DBUF_RDY        6
#define ISD2360_DEVICE_STATUS_INT             5
#define ISD2360_DEVICE_STATUS_CH2_BSY         3
#define ISD2360_DEVICE_STATUS_CH1_BSY         2
#define ISD2360_DEVICE_STATUS_CH0_BSY         1
#define ISD2360_DEVICE_STATUS_DIG_BSY         0

// Interrupt Status Register
#define ISD2360_INTERRUPT_STATUS_TALARM_INT   7
#define ISD2360_INTERRUPT_STATUS_MPT_ERR      6
#define ISD2360_INTERRUPT_STATUS_WR_FIN       5
#define ISD2360_INTERRUPT_STATUS_CMD_ERR      4
#define ISD2360_INTERRUPT_STATUS_OVF_ERR      3
#define ISD2360_INTERRUPT_STATUS_CH2_FIN      2
#define ISD2360_INTERRUPT_STATUS_CH1_FIN      1
#define ISD2360_INTERRUPT_STATUS_CH0_FIN      0

// Path Control Register
#define ISD2360_REG_PATH_CONTROL              0x02
#define ISD2360_REG_PATH_CONTROL_DECODE       6
#define ISD2360_REG_PATH_CONTROL_SPI_IN       5
#define ISD2360_REG_PATH_CONTROL_PWM_OUT      2
#define ISD2360_REG_PATH_CONTROL_SPI_OUT      0

// Checksum Reset Register
#define ISD2360_REG_CHECKSUM_RESET            0x04
#define ISD2360_REG_CHECKSUM_RESET_BIT        4

#define ISD2360_EXP_PART_ID                   0x05
#define ISD2360_EXP_MAN_ID                    0xEF
#define ISD2360_EXP_MEM_TYPE                  0x20
#define ISD2360_EXP_DEV_ID                    0x60

#define ISD2360_BUF_SIZE                      256

enum SignalPath {
  PLAYBACK_TO_SPEAKER,
  SPI_PLAYBACK,
  SPI_DECODE_TO_SPEAKER,
};

class ISD2360 {

public:
  ISD2360(uint8_t pinRdy, uint8_t pinSsb, bool debug);
  void begin();
  bool getDebug();
  void setDebug(bool debug);

  uint8_t readStatus();
  uint8_t readInterrupt();
  bool readId();
  void powerUp();
  void powerDown();
  void reset();
  void writeConfigurationRegister(uint8_t reg, const uint8_t* data, size_t dataLen);
  void writeConfigurationRegister(uint8_t reg, const uint8_t val);
  void readConfigurationRegister(uint8_t reg, uint8_t* data, size_t dataLen);
  uint8_t readConfigurationRegister(uint8_t reg);
  void flashWrite(uint32_t addr, const uint8_t* data, size_t dataLen);
  void flashRead(uint32_t addr, uint8_t* data, size_t dataLen);
  uint8_t flashRead(uint32_t addr);
  void flashDump();
  void spiAudioWrite(const uint8_t* data, size_t dataLen);
  void playVoicePrompt(uint16_t index);
  void playbackStop();
  void eraseChip();
  uint32_t readChecksum(uint32_t endAddr);
  
  void configureSignalPath(SignalPath signalPath);

private:
  uint8_t pinRdy;
  uint8_t pinSsb;
  
  uint8_t deviceStatus;
  uint8_t data[ISD2360_BUF_SIZE] = {0};
  uint8_t response[ISD2360_BUF_SIZE] = {0};

  bool debug;

  void spiTransfer(uint8_t cmd, const uint8_t* data, size_t dataLen);
  void spiTransfer(uint8_t cmd, const uint8_t* data, size_t dataLen, uint8_t* response);
  void loadAddress(uint32_t addr);
};

#endif
