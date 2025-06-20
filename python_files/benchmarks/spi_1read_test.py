# -*- coding: utf-8 -*-
#!/usr/bin/python

import pigpio
import math
import time
from statistics import mean

# ICM20948 I2C ADDRESS
ICM20948_ADD    = 0x68
# AK09916 I2C ADDRESS
MAG_ADD         = 0x0C

## ICM20948 REGS ADDRESSES ##
# USER BANK 0
WHO_AM_I        = 0x00
USER_CTRL       = 0x03
LP_CONFIG       = 0x05
PWR_MGMT_1      = 0x06
PWR_MGMT_2      = 0x07
INT_PIN_CFG     = 0x0F
I2C_MST_STATUS  = 0x17
DELAY_TIMEH     = 0x28
ACCEL_XOUT_H    = 0x2D
GYRO_XOUT_H     = 0x33
TEMP_OUT_H      = 0x39
DATA_RDY_STATUS = 0x74
FIFO_CFG        = 0x76
REG_BANK_SEL    = 0x7F
EXT_SLV_SENS_DATA_00 = 0x3B
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
TEMP_CONFIG     = 0x53
# USER BANK 3
I2C_MST_ODR_CONFIG  = 0x00
I2C_MST_CTRL        = 0x01
I2C_SLV0_ADDR       = 0x03
I2C_SLV0_REG        = 0x04
I2C_SLV0_CTRL       = 0x05

## ICM REGISTERS VALUES ##
A_DLPF_1 = (0x01 << 3)    # 218.1 Hz   ACCELEROMETER
A_DLPF_2 = (0x02 << 3)    # 99 Hz      DIGITAL
A_DLPF_3 = (0x03 << 3)    # 44.8 Hz    LOW
A_DLPF_4 = (0x04 << 3)    # 21.2 Hz    PASS
A_DLPF_5 = (0x05 << 3)    # 10.2 Hz    FILTER
A_DLPF_6 = (0x06 << 3)    # 5.05 Hz    CONFIGURATION
A_FS_SEL_2G     = 0x00
A_FS_SEL_16G    = 0x06
A_DLPF_ENABLE   = 0x01
BYPASS_EN       = 0x02
I2C_MST_CTRL    = 0x01

## AK09916 REGS ADDRESSES ##
MAG_WIA2    = 0x01
MAG_ST1     = 0x10
MAG_HXL     = 0x11
MAG_HXH     = 0x12
MAG_HYL     = 0x13
MAG_HYH     = 0x14
MAG_HZL     = 0x15
MAG_HZH     = 0x16
MAG_ST2     = 0x18
MAG_CNTL2   = 0x31
MAG_CNTL3   = 0x32

## AK09916 REGS VALUES ##
PWDOWN_MODE = 0x00
FUSE_ROM_MODE = 0x0F
C8HZ_MODE = 0x02
C100HZ_MODE = 0x06
RES_14_BIT = 0x00
RES_16_BIT = 0x01
DRDY = 0x01

## GENERAL VALUES ##
SPI_CHANNEL = 0
SPI_FREQ = 7000000
SPI_MODE = 0b11

#Scaling parameters
gyro_scale = 131.0          #scaling parameter for gyroscope readings
accel_scale = 16384.0       #scaling parameter for accelerometer readings
mag_scale = 4900.0/32768.0  #scaling parameter for magnetometer readings

#Calibration parameters
mag_x_offset = -5
mag_y_offset = 10
mag_z_offset = -5

tm_array = []

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

def icm_read_all():
    spi_select_bank(0)
    raw_data = spi_read_block(ICM_SPI, ACCEL_XOUT_H, 14)

    accel_scaled_x = twos_comp((raw_data[0] << 8) + raw_data[1]) / accel_scale
    accel_scaled_y = twos_comp((raw_data[2] << 8) + raw_data[3]) / accel_scale
    accel_scaled_z = twos_comp((raw_data[4] << 8) + raw_data[5]) / accel_scale

    gyro_scaled_x = twos_comp((raw_data[6] << 8) + raw_data[7]) / gyro_scale
    gyro_scaled_y = twos_comp((raw_data[8] << 8) + raw_data[9]) / gyro_scale
    gyro_scaled_z = twos_comp((raw_data[10] << 8) + raw_data[11]) / gyro_scale

    temp = twos_comp((raw_data[12] << 8) + raw_data[13]) / 321 + 21

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

def reset_icm():
    spi_select_bank(0)
    spi_write_byte(ICM_SPI, PWR_MGMT_1, 0x80)
    time.sleep(0.100)

def set_normal_mode():
    spi_select_bank(0)
    spi_write_byte(ICM_SPI, PWR_MGMT_1, 0x01)
    time.sleep(0.010)

def enable_i2c_master_ctl():
    spi_select_bank(0)
    spi_write_byte(ICM_SPI, USER_CTRL, 0x20)
    spi_write_byte(ICM_SPI, INT_PIN_CFG, 0x00)
    time.sleep(0.01)
    spi_select_bank(3)
    spi_write_byte(ICM_SPI, I2C_MST_CTRL, 0x07)

def i2c_master_read(slave_addr, slave_reg, length):
    spi_select_bank(3)
    spi_write_byte(ICM_SPI, I2C_SLV0_ADDR, slave_addr | 0x80)
    spi_write_byte(ICM_SPI, I2C_SLV0_REG, slave_reg)
    spi_write_byte(ICM_SPI, I2C_SLV0_CTRL, 0x80 | length)
    time.sleep(0.01)

def set_accel_DLPF(val):
    spi_select_bank(2)
    spi_write_byte(ICM_SPI, ACCEL_CONFIG, val)

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

def write_accel_offsets_to_IMU(a_offset_xh, a_offset_xl, a_offset_yh, a_offset_yl, a_offset_zh, a_offset_zl):
    write_block(ICM_SPI, XA_OFFSET_H, [a_offset_xh, a_offset_xl])
    time.sleep(0.01)
    write_block(ICM_SPI, YA_OFFSET_H, [a_offset_yh, a_offset_yl])
    time.sleep(0.01)
    write_block(ICM_SPI, ZA_OFFSET_H, [a_offset_zh, a_offset_zl])
    time.sleep(0.01)

def write_gyro_offsets_to_IMU(g_offset_xh, g_offset_xl, g_offset_yh, g_offset_yl, g_offset_zh, g_offset_zl):
    write_block(ICM_SPI, XG_OFFSET_H, [g_offset_xh, g_offset_xl])
    time.sleep(0.01)
    write_block(ICM_SPI, YG_OFFSET_H, [g_offset_yh, g_offset_yl])
    time.sleep(0.01)
    write_block(ICM_SPI, ZG_OFFSET_H, [g_offset_zh, g_offset_zl])
    time.sleep(0.01)

################################################################################
################################################################################
#Reset and wake up the ICM20948
reset_icm()
set_normal_mode()
#Read WHO_AM_I register and print the value
spi_select_bank(0)
who_am_i = spi_read_byte(ICM_SPI, 0x00)
print(f"WHO_AM_I = 0x{who_am_i:02X}")
#Enable the i2c master internal interface
enable_i2c_master_ctl()
#Read AK09916's WHO_AM_I(WIA2) register (i2c+spi) and print the value
i2c_master_read(MAG_ADD, MAG_WIA2, 1)

spi_select_bank(0)
mag_wia2 = spi_read_byte(ICM_SPI, EXT_SLV_SENS_DATA_00)
print(f"mag_wia2: 0x{mag_wia2:02X}")
time.sleep(0.1)
#Set the DLPF filtering frequency (BW)
set_accel_DLPF(A_DLPF_5 | A_DLPF_ENABLE)
time.sleep(0.01)

#set the correct accelerometer offsets
#a_offset_xh, a_offset_xl, a_offset_yh, a_offset_yl, a_offset_zh, a_offset_zl = load_accel_offsets()
#write_accel_offsets_to_IMU(a_offset_xh, a_offset_xl, a_offset_yh, a_offset_yl, a_offset_zh, a_offset_zl)
#print(a_offset_xh, a_offset_xl, a_offset_yh, a_offset_yl, a_offset_zh, a_offset_zl)
#time.sleep(10)

#set the correct gyroscope offsets
#write_gyro_offsets_to_IMU(255,126,0,18,0,0)

prev_time = time.time()

#Read accelerometer and gyroscope raw values and get the scaled values
(gyro_scaled_x, gyro_scaled_y, gyro_scaled_z, accel_scaled_x, accel_scaled_y, accel_scaled_z, temp_raw) = icm_read_all()
print(f"Ax:{accel_scaled_x}, Ay:{accel_scaled_y}, Az:{accel_scaled_z} \t Gx:{gyro_scaled_x}, Gy:{gyro_scaled_y}, Gz:{gyro_scaled_z}")

try:
    for i in range(1,500+1):
        cur_time = time.time()
        Tm = (cur_time - prev_time) #Sample time variable measures time between loop code runs
        tm_array.append(Tm)
        prev_time = cur_time        #This variable keeps current time as previous one for the next Tm assignment
        #read accelerometer and gyroscope raw values and get the scaled values
        (gyro_scaled_x, gyro_scaled_y, gyro_scaled_z, accel_scaled_x, accel_scaled_y, accel_scaled_z, temp_raw) = icm_read_all()

finally:
    for i in range(0,len(tm_array)):
        print("%.3f ms" % (tm_array[i]*1000))

    print("Tm_mean: %.3f ms" % (mean(tm_array)*1000))
    print("Temp_raw: %.6f" % (temp_raw))
    pi.spi_close(ICM_SPI)
    pi.stop()
