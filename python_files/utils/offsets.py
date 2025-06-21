# -*- coding: utf-8 -*-
#!/usr/bin/python

import pigpio
import math
import time

# USER BANK 0
PWR_MGMT_1      = 0x06
INT_PIN_CFG     = 0x0F
REG_BANK_SEL    = 0x7F
# USER BANK 1
XA_OFFSET_H     = 0x14
XA_OFFSET_L     = 0x15
YA_OFFSET_H     = 0x17
YA_OFFSET_L     = 0x18
ZA_OFFSET_H     = 0x1A
ZA_OFFSET_L     = 0x1B
# USER BANK 2
GYRO_CONFIG_1   = 0x01
XG_OFFS_USRH    = 0x03
YG_OFFS_USRH    = 0x05
ZG_OFFS_USRH    = 0x07
ACCEL_CONFIG    = 0x14

## GENERAL VALUES ##
SPI_CHANNEL = 0
SPI_FREQ = 7000000
SPI_MODE = 0b11

gyro_x_offset = 0.0
gyro_y_offset = 0.0
gyro_z_offset = 0.0
accel_x_offset = 0.0
accel_y_offset = 0.0
accel_z_offset = 0.0

pi = pigpio.pi()
if not pi.connected:
    raise RuntimeError("Not successfully connected to pigpiod!")

#Open SPI channel 0, 1 MHz, mode 3 (CPOL=1, CPHA=1)
ICM_SPI = pi.spi_open(SPI_CHANNEL, SPI_FREQ, SPI_MODE)


def spi_select_bank(bank):
    pi.spi_write(ICM_SPI, [REG_BANK_SEL, bank << 4])
    time.sleep(0.001)

def spi_read_byte(handle, reg):
    count, data = pi.spi_xfer(handle, [reg | 0x80, 0x00])
    return data[1]

def spi_write_byte(handle, reg, val):
    pi.spi_write(handle, [reg, val])

def spi_read_block(handle, start_reg, length):
    tx = [start_reg | 0x80] + [0x00] * length   # Command + length zero-bytes
    count, data = pi.spi_xfer(handle, tx)
    return data[1:]

def spi_write_block(handle, start_reg, values):
    tx = [start_reg & 0x7F] + list(values)
    count, data = pi.spi_xfer(handle, tx)
    return count == len(tx)     # True if all bytes returned

def twos_comp(val):
    if (val >= 0x8000):
        return -((65535 - val) + 1)
    else:
        return val

def reset_icm():
    spi_select_bank(0)
    spi_write_byte(ICM_SPI, PWR_MGMT_1, 0x80)
    time.sleep(0.100)

def set_normal_mode():
    spi_select_bank(0)
    spi_write_byte(ICM_SPI, PWR_MGMT_1, 0x01)
    time.sleep(0.010)

def set_accel_config(val):
    spi_select_bank(2)
    spi_write_byte(ICM_SPI, ACCEL_CONFIG, val)

def format_data(data):
    matrix = []
    for line in data:
        matrix.append([float(x) for x in line.strip().split(" ")])
    return matrix

def load_accel_offsets():
    fichero = open('/home/pi/repositories/dronepoint-2/accel_calib.txt','r')
    accel_offsets = format_data(fichero)[0]

    fichero.close()
    return accel_offsets[0], accel_offsets[1], accel_offsets[2], accel_offsets[3], accel_offsets[4], accel_offsets[5]

#ICM start
#reset_icm()
set_normal_mode()

try:
    spi_select_bank(2)
    gyro_offsets = spi_read_block(ICM_SPI, XG_OFFS_USRH, 6)
    gyro_x_offset = twos_comp((gyro_offsets[0] << 8) + gyro_offsets[1])
    gyro_y_offset = twos_comp((gyro_offsets[2] << 8) + gyro_offsets[3])
    gyro_z_offset = twos_comp((gyro_offsets[4] << 8) + gyro_offsets[5])
    print(f"Gyro Offsets: {gyro_x_offset}, {gyro_y_offset}, {gyro_z_offset}")

    #Values read are 2048 LSB/g, for comparison with other FS configurations (4096/8192/16384) readings multiply by 2/4/8
    spi_select_bank(1)
    accel_x_offset = spi_read_block(ICM_SPI, XA_OFFSET_H, 2)
    print(f"xh:{accel_x_offset[0]} xl:{accel_x_offset[1]}",end='')
    accel_x_offset = twos_comp((accel_x_offset[0] << 8) + accel_x_offset[1])

    accel_y_offset = spi_read_block(ICM_SPI, YA_OFFSET_H, 2)
    print(f" yh:{accel_y_offset[0]} yl:{accel_y_offset[1]}",end='')
    accel_y_offset = twos_comp((accel_y_offset[0] << 8) + accel_y_offset[1])

    accel_z_offset = spi_read_block(ICM_SPI, ZA_OFFSET_H, 2)
    print(f" zh:{accel_z_offset[0]} zl:{accel_z_offset[1]}")
    accel_z_offset = twos_comp((accel_z_offset[0] << 8) + accel_z_offset[1])

    print(f"Accel Offsets: {accel_x_offset}, {accel_y_offset}, {accel_z_offset}")

    ## spi_write_block(ICM_SPI, _A_OFFSET_H, [, ])

    accel_x_offset = spi_read_block(ICM_SPI, XA_OFFSET_H, 2)
    print(f"xh:{accel_x_offset[0]} xl:{accel_x_offset[1]}",end='')
    accel_x_offset = twos_comp((accel_x_offset[0] << 8) + accel_x_offset[1])

    accel_y_offset = spi_read_block(ICM_SPI, YA_OFFSET_H, 2)
    print(f" yh:{accel_y_offset[0]} yl:{accel_y_offset[1]}",end='')
    accel_y_offset = twos_comp((accel_y_offset[0] << 8) + accel_y_offset[1])

    accel_z_offset = spi_read_block(ICM_SPI, ZA_OFFSET_H, 2)
    print(f" zh:{accel_z_offset[0]} zl:{accel_z_offset[1]}")
    accel_z_offset = twos_comp((accel_z_offset[0] << 8) + accel_z_offset[1])

    print(f"Accel Offsets: {accel_x_offset}, {accel_y_offset}, {accel_z_offset}")

except KeyboardInterrupt:
    pass
