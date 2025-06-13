# -*- coding: utf-8 -*-
#!/usr/bin/python

import smbus
import math
import time
from statistics import mean

#MPU9250 Address
MPU_ADD = 0x68
#AK8963 Address
MAG_ADD = 0x0C
#MPU9250 Registers Addresses
PWR_MGMT_1 = 0x6B
GYRO_XOUT_H = 0x43
ACCEL_XOUT_H = 0x3B
ACCEL_CONFIG2 = 0x1D
INT_PIN_CFG = 0x37
XG_OFFSET_H = 0x13
YG_OFFSET_H = 0x15
ZG_OFFSET_H = 0x17
XA_OFFSET_H = 0x77
XA_OFFSET_L = 0x78
YA_OFFSET_H = 0x7A
YA_OFFSET_L = 0x7B
ZA_OFFSET_H = 0x7D
ZA_OFFSET_L = 0x7E
#Values
A_FCHOICE_B = (0x00 << 2)
A_DLPF_1 = 0x01    #218.1 Hz   ACCELEROMETER
A_DLPF_2 = 0x02    #99 Hz      DIGITAL
A_DLPF_3 = 0x03    #44.8 Hz    LOW
A_DLPF_4 = 0x04    #21.2 Hz    PASS
A_DLPF_5 = 0x05    #10.2 Hz    FILTER
A_DLPF_6 = 0x06    #5.05 Hz    CONFIGURATION
BYPASS_EN = 0x02
#AK8963 Registers Addresses
ST1 = 0x02
CNTL_1 = 0x0A
ASA = 0x10
MAG_OUT = 0x03
#Values
PWDOWN_MODE = 0x00
FUSE_ROM_MODE = 0x0F
C8HZ_MODE = 0x02
C100HZ_MODE = 0x06
RES_14_BIT = 0x00
RES_16_BIT = 0x01
DRDY = 0x01

#Scaling parameters
gyro_scale = 131.0          #scaling parameter for gyroscope readings
accel_scale = 16384.0       #scaling parameter for accelerometer readings
mag_scale = 4912.0/32760.0  #scaling parameter for magnetometer readings

#Calibration parameters
mag_x_offset = -5
mag_y_offset = 10
mag_z_offset = -5

tm_array = []

bus = smbus.SMBus(1)

def read_byte(address, adr):
    return bus.read_byte_data(address, adr)

def write_byte(address, adr, value):
    return bus.write_byte_data(address, adr, value)

def read_block(address, adr, size):
    return bus.read_i2c_block_data(address, adr, size)

def write_block(address, adr, values):
    bus.write_i2c_block_data(address, adr, values)

def read_all():
    raw_data = read_block(MPU_ADD, ACCEL_XOUT_H, 14)

    accel_scaled_x = twos_comp((raw_data[0] << 8) + raw_data[1]) / accel_scale
    accel_scaled_y = twos_comp((raw_data[2] << 8) + raw_data[3]) / accel_scale
    accel_scaled_z = twos_comp((raw_data[4] << 8) + raw_data[5]) / accel_scale

    temp = twos_comp((raw_data[6] << 8) + raw_data[7]) / 321 + 21

    gyro_scaled_x = twos_comp((raw_data[8] << 8) + raw_data[9]) / gyro_scale
    gyro_scaled_y = twos_comp((raw_data[10] << 8) + raw_data[11]) / gyro_scale
    gyro_scaled_z = twos_comp((raw_data[12] << 8) + raw_data[13]) / gyro_scale

    return (gyro_scaled_x, gyro_scaled_y, gyro_scaled_z, accel_scaled_x, accel_scaled_y, accel_scaled_z, temp)

def twos_comp(val):
    if (val >= 0x8000):
        return -((65535 - val) + 1)
    else:
        return val

def dist(a, b):
    return math.sqrt((a * a) + (b * b))

def get_y_rotation(x, y, z):
    radians = math.atan2(x, dist(y, z))
    return -math.degrees(radians)

def get_x_rotation(x, y, z):
    radians = math.atan2(y, dist(x, z))
    return math.degrees(radians)

def set_normal_mode():
    write_byte(MPU_ADD, PWR_MGMT_1, 0)

def bypass_mag():
    write_byte(MPU_ADD, INT_PIN_CFG, BYPASS_EN)

def set_accel_DLPF(val):
    write_byte(MPU_ADD, ACCEL_CONFIG2, val)

def north_to_deg(x, y):
    north_rad = math.atan2(y, x) + math.pi / 2
    if (north_rad < 0):
        north_rad += 2 * math.pi
    return math.degrees(north_rad)

def format_data(data):
    matrix = []
    for line in data:
        matrix.append([float(x) for x in line.strip().split(" ")])
    return matrix

def tilt_compensation(mag_x, mag_y, mag_z, rad_pitch, rad_roll):
    comp_x = (mag_x * math.cos(rad_roll)) + (mag_z * math.sin(rad_roll))
    comp_y = (mag_x * math.sin(rad_pitch) * math.sin(rad_roll)) + (mag_y * math.cos(rad_pitch)) - (mag_z * math.sin(rad_pitch) * math.cos(rad_roll))
    return comp_x, comp_y

def load_mag_offsets():
    fichero = open('mag_calib.txt','r')
    mag_offsets = format_data(fichero)[0]

    fichero.close()
    return mag_offsets[0], mag_offsets[1], mag_offsets[2]

def load_accel_offsets():
    fichero = open('accel_calib.txt','r')
    accel_offsets = format_data(fichero)[0]

    fichero.close()
    return int(accel_offsets[0]), int(accel_offsets[1]), int(accel_offsets[2]), int(accel_offsets[3]), int(accel_offsets[4]), int(accel_offsets[5])

def write_accel_offsets_to_MPU(a_offset_xh, a_offset_xl, a_offset_yh, a_offset_yl, a_offset_zh, a_offset_zl):
    write_block(MPU_ADD, XA_OFFSET_H, [a_offset_xh, a_offset_xl])
    time.sleep(0.01)
    write_block(MPU_ADD, YA_OFFSET_H, [a_offset_yh, a_offset_yl])
    time.sleep(0.01)
    write_block(MPU_ADD, ZA_OFFSET_H, [a_offset_zh, a_offset_zl])
    time.sleep(0.01)

def write_gyro_offsets_to_MPU(g_offset_xh, g_offset_xl, g_offset_yh, g_offset_yl, g_offset_zh, g_offset_zl):
    write_block(MPU_ADD, XG_OFFSET_H, [g_offset_xh, g_offset_xl])
    time.sleep(0.01)
    write_block(MPU_ADD, YG_OFFSET_H, [g_offset_yh, g_offset_yl])
    time.sleep(0.01)
    write_block(MPU_ADD, ZG_OFFSET_H, [g_offset_zh, g_offset_zl])
    time.sleep(0.01)

#Now wake the MPU9250 up as it starts in sleep mode and set magnetomer bypass enable
set_normal_mode()
bypass_mag()
#Set the DLPF filtering frequency (BW)
set_accel_DLPF(A_DLPF_5)
time.sleep(0.01)

#set the correct accelerometer offsets
a_offset_xh, a_offset_xl, a_offset_yh, a_offset_yl, a_offset_zh, a_offset_zl = load_accel_offsets()
write_accel_offsets_to_MPU(a_offset_xh, a_offset_xl, a_offset_yh, a_offset_yl, a_offset_zh, a_offset_zl)
#print(a_offset_xh, a_offset_xl, a_offset_yh, a_offset_yl, a_offset_zh, a_offset_zl)
#time.sleep(10)

#set the correct gyroscope offsets
write_gyro_offsets_to_MPU(255,126,0,18,0,0)

prev_time = time.time()

#Read accelerometer and gyroscope raw values and get the scaled values
(gyro_scaled_x, gyro_scaled_y, gyro_scaled_z, accel_scaled_x, accel_scaled_y, accel_scaled_z, temp_raw) = read_all()

try:
    for i in range(1,500+1):
        cur_time = time.time()
        Tm = (cur_time - prev_time) #Sample time variable measures time between loop code runs
        tm_array.append(Tm)
        prev_time = cur_time        #This variable keeps current time as previous one for the next Tm assignment
        #read accelerometer and gyroscope raw values and get the scaled values
        (gyro_scaled_x, gyro_scaled_y, gyro_scaled_z, accel_scaled_x, accel_scaled_y, accel_scaled_z, temp_raw) = read_all()

finally:
    for i in range(0,len(tm_array)):
        print("%.3f ms" % (tm_array[i]*1000))

    print("Tm_mean: %.3f ms" % (mean(tm_array)*1000))
    print("Temp_raw: %.6f" % (temp_raw))
