# -*- coding: utf-8 -*-
#!/usr/bin/python

import smbus
import math
import time
import RPi.GPIO as GPIO

#MPU9250 Address
MPU_ADD = 0x68
#MPU9250 Registers Addresses
PWR_MGMT_1 = 0x6B
INT_PIN_CFG = 0x37
XG_OFFSET_H = 0x13
XG_OFFSET_L = 0x14
YG_OFFSET_H = 0x15
YG_OFFSET_L = 0x16
ZG_OFFSET_H = 0x17
ZG_OFFSET_L = 0x18
XA_OFFSET_H = 0x77
XA_OFFSET_L = 0x78
YA_OFFSET_H = 0x7A
YA_OFFSET_L = 0x7B
ZA_OFFSET_H = 0x7D
ZA_OFFSET_L = 0x7E
#Values
BYPASS_EN = 0x02

gyro_x_offset = 0
gyro_y_offset = 0
gyro_z_offset = 0
accel_x_offset = 0
accel_y_offset = 0
accel_z_offset = 0

GPIO.setmode(GPIO.BCM)
bus = smbus.SMBus(1)

def read_byte(address, adr):
    return bus.read_byte_data(address, adr)

def write_byte(address, adr, value):
    return bus.write_byte_data(address, adr, value)

def read_block(address, adr, size):
    return bus.read_i2c_block_data(address, adr, size)

def write_block(address, adr, values):
    bus.write_i2c_block_data(address, adr, values)

def twos_comp(val):
    if (val >= 0x8000):
        return -((65535 - val) + 1)
    else:
        return val

def set_normal_mode():
    write_byte(MPU_ADD, PWR_MGMT_1, 0)

def format_data(data):
    matrix = []
    for line in data:
        matrix.append([float(x) for x in line.strip().split(" ")])
    return matrix

def load_accel_offsets():
    fichero = open('../../config/accel_calib.txt','r')
    accel_offsets = format_data(fichero)[0]

    fichero.close()
    return accel_offsets[0], accel_offsets[1], accel_offsets[2], accel_offsets[3], accel_offsets[4], accel_offsets[5]

#IMU wake-up
set_normal_mode()

try:
    gyro_offsets = read_block(MPU_ADD, XG_OFFSET_H, 6)
    gyro_x_offset = twos_comp((gyro_offsets[0] << 8) + gyro_offsets[1])
    gyro_y_offset = twos_comp((gyro_offsets[2] << 8) + gyro_offsets[3])
    gyro_z_offset = twos_comp((gyro_offsets[4] << 8) + gyro_offsets[5])
    print(f"Gyro Offsets: {gyro_x_offset}, {gyro_y_offset}, {gyro_z_offset}")

    #Values read are 2048 LSB/g, for comparison with other FS configurations (4096/8192/16384) readings multiply by 2/4/8
    accel_x_offset = read_block(MPU_ADD, XA_OFFSET_H, 2)
    print(accel_x_offset,end='')
    accel_x_offset = twos_comp((accel_x_offset[0] << 8) + accel_x_offset[1])

    accel_y_offset = read_block(MPU_ADD, YA_OFFSET_H, 2)
    print(accel_y_offset,end='')
    accel_y_offset = twos_comp((accel_y_offset[0] << 8) + accel_y_offset[1])

    accel_z_offset = read_block(MPU_ADD, ZA_OFFSET_H, 2)
    print(accel_z_offset)
    accel_z_offset = twos_comp((accel_z_offset[0] << 8) + accel_z_offset[1])

    print(f"Accel Offsets: {accel_x_offset}, {accel_y_offset}, {accel_z_offset}")

    ##write_block(MPU_ADD, _A_OFFSET_H, [, ])

    accel_x_offset = read_block(MPU_ADD, XA_OFFSET_H, 2)
    print(accel_x_offset,end='')
    accel_x_offset = twos_comp((accel_x_offset[0] << 8) + accel_x_offset[1])

    accel_y_offset = read_block(MPU_ADD, YA_OFFSET_H, 2)
    print(accel_y_offset,end='')
    accel_y_offset = twos_comp((accel_y_offset[0] << 8) + accel_y_offset[1])

    accel_z_offset = read_block(MPU_ADD, ZA_OFFSET_H, 2)
    print(accel_z_offset)
    accel_z_offset = twos_comp((accel_z_offset[0] << 8) + accel_z_offset[1])

    print(f"Accel Offsets: {accel_x_offset}, {accel_y_offset}, {accel_z_offset}")



except KeyboardInterrupt:
    GPIO.cleanup()
