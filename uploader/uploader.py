# uploader.py can be used with FlashUpdate.ino to update the firmware
# of Nuvoton ChipCorder ISD2360 devices.
# Copyright (C) 2020 Mathis Dedial
# 
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
# 
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
# 
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <https://www.gnu.org/licenses/>.

import argparse
import math
import os.path
from typing import List

from tqdm import tqdm
import serial


BAUD_RATE = 115200
TIMEOUT = 60.
SECTOR_SIZE = 1024
CHUNK_SIZE = 128

PACKET_HELLO = 0x01
PACKET_HELLO_ACK = 0x02
PACKET_ACK = 0x03
PACKET_ERR = 0x04
PACKET_STOP = 0x05
PACKET_RDY = 0x06


def _make_packet(payload: int) -> bytes:
    return bytes([payload])


def _validate_sector(sector: int) -> None:
    if sector > 255:
        raise RuntimeError(f'Invalid sector 0x{sector:02x}')


def _expect_byte(port: serial.Serial, expected: int) -> None:
    response = port.read(1)
    if len(response) < 1:
        raise RuntimeError(f'Expected 0x{expected:02x}, did not get response')
    response_int = int.from_bytes(response, 'big')
    if response_int != expected:
        raise RuntimeError(f'Expected 0x{expected:02x}, got 0x{response_int:02x}')


def _handshake(port: serial.Serial, n_sectors: int) -> None:
    _validate_sector(n_sectors)
    port.write(_make_packet(PACKET_HELLO))
    _expect_byte(port, PACKET_HELLO_ACK)
    port.write(_make_packet(n_sectors))
    _expect_byte(port, PACKET_ACK)


def _write_sector(port: serial.Serial, sector: int, data: bytes) -> None:
    _validate_sector(sector)
    if len(data) > SECTOR_SIZE:
        raise RuntimeError(f'Sector exceeds {SECTOR_SIZE} bytes in length: {len(data)}')
    if len(data) < SECTOR_SIZE:
        data = data.ljust(SECTOR_SIZE, b'\xff')
    assert len(data) == SECTOR_SIZE

    # write sector
    port.write(_make_packet(sector))
    _expect_byte(port, PACKET_ACK)

    # write chunked sector data
    n_chunks = math.ceil(SECTOR_SIZE / CHUNK_SIZE)
    for chunk in range(n_chunks):
        start = chunk * CHUNK_SIZE
        end = start + CHUNK_SIZE
        chunk_data = data[start:end]
        assert len(chunk_data) == CHUNK_SIZE
        port.write(chunk_data)
        _expect_byte(port, PACKET_ACK)


def _read_sector(port: serial.Serial, sector: int) -> bytes:
    _validate_sector(sector)
    port.write(_make_packet(sector))
    _expect_byte(port, PACKET_ACK)
    n_chunks = math.ceil(SECTOR_SIZE / CHUNK_SIZE)
    response = bytearray()
    for chunk in range(n_chunks):
        chunk_response = port.read(CHUNK_SIZE)
        port.write(_make_packet(PACKET_ACK))
        if len(chunk_response) < CHUNK_SIZE:
            raise RuntimeError(f'Expected {CHUNK_SIZE} bytes, received {len(chunk_response)}')
        response += chunk_response
    assert len(response) == SECTOR_SIZE
    return bytes(response)


def _sectors(image: bytes) -> List[bytes]:
    image_size = len(image)
    return [image[i:min(i + SECTOR_SIZE, image_size)] for i in range(0, image_size, SECTOR_SIZE)]


def upload(serial_port_path: str, image_path: str):

    # open serial port
    port = serial.Serial(serial_port_path, BAUD_RATE, timeout=TIMEOUT)
    print(f'Opened serial port "{serial_port_path}" @ {BAUD_RATE} baud')
    
    # read the entire image file into memory
    if not os.path.exists(image_path):
        raise RuntimeError(f'Image file "{image_path}" does not exist')
    with open(image_path, 'rb') as image_file:
        image = image_file.read()
    print(f'Image file size: {len(image)} bytes')
    sectors = _sectors(image)
    print(f'# Sectors: {len(sectors)}')

    # wait for user to trigger the upload
    _continue = input(f'Start upload? [y/n] ')
    if _continue.lower() != 'y':
        print('Aborting.')
        exit(1)

    # arduino will typically reset when serial port is opened, so we wait until it's ready
    print('Waiting for ready packet from device')
    _expect_byte(port, PACKET_RDY)

    # handshake
    print('Performing handshake')
    _handshake(port, len(sectors))

    # upload the image file
    print('Uploading image')
    for i, sector in enumerate(tqdm(sectors)):
        _write_sector(port, i, sector)

    # handshake the upload phase
    print('Performing upload phase handshake')
    _expect_byte(port, PACKET_STOP)
    port.write(_make_packet(PACKET_ACK))

    # verify image
    print('Verifying image')
    for i, sector in enumerate(tqdm(sectors)):
        sector_on_device = _read_sector(port, i)
        if sector != sector_on_device:
            dump_file_path = f'dump_sector_{i:02x}.bin'
            with open(dump_file_path, 'wb') as dump_file:
                dump_file.write(sector_on_device)
            expected_file_path = f'expected_sector_{i:02x}.bin'
            with open(expected_file_path, 'wb') as expected_file:
                expected_file.write(sector)
            raise RuntimeError(f'Mismatch in sector 0x{i:02x}, content written to "{dump_file_path}"')

    # handshake the verification phase
    print('Performing verification phase handshake')
    _expect_byte(port, PACKET_STOP)
    port.write(_make_packet(PACKET_ACK))

    print('Done')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(epilog='Copyright (C) 2020 Mathis Dedial',
                                     description='A program to upload firmware images to Nuvoton ChipCorder ISD2360 devices. '
                                                 'Requires an Arduino running the "FlashUpdate.ino" sketch.')
    parser.add_argument('-s', '--serial-port',
                        dest='serial_port_path',
                        required=True,
                        help='path to the device file of the serial port of the Arduino (e.g. /dev/ttyACM0 on Linux)')
    parser.add_argument('-i', '--image-file',
                        dest='image_path',
                        required=True,
                        help='path to the image file generated with VPE-ISD2360, usually ending in .mem')
    args = parser.parse_args()
    upload(**vars(args))
