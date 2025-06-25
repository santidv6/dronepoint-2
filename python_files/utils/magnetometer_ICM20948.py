# -*- coding: utf-8 -*-
#!/usr/bin/python

import pigpio
import math
import time
from statistics import mean

# ICM20948 I2C ADDRESS
ICM20948_ADD        = 0x68
# AK09916 I2C ADDRESS
MAG_ADD             = 0x0C
## ICM20948 REGS ADDRESSES ##
# USER BANK 0
WHO_AM_I            = 0x00
USER_CTRL           = 0x03
LP_CONFIG           = 0x05
PWR_MGMT_1          = 0x06
PWR_MGMT_2          = 0x07
INT_PIN_CFG         = 0x0F
I2C_MST_STATUS      = 0x17
DELAY_TIMEH         = 0x28
ACCEL_XOUT_H        = 0x2D
GYRO_XOUT_H         = 0x33
TEMP_OUT_H          = 0x39
DATA_RDY_STATUS     = 0x74
FIFO_CFG            = 0x76
REG_BANK_SEL        = 0x7F
EXT_SLV_SENS_DATA_00 = 0x3B
# USER BANK 1
XA_OFFSET_H         = 0x14
XA_OFFSET_L         = 0x15
YA_OFFSET_H         = 0x17
YA_OFFSET_L         = 0x18
ZA_OFFSET_H         = 0x1A
ZA_OFFSET_L         = 0x1B
# USER BANK 2
GYRO_CONFIG_1       = 0x01
XG_OFFS_USRH        = 0x03
YG_OFFS_USRH        = 0x05
ZG_OFFS_USRH        = 0x07
ACCEL_CONFIG        = 0x14
TEMP_CONFIG         = 0x53
# USER BANK 3
I2C_MST_ODR_CONFIG  = 0x00
I2C_MST_CTRL        = 0x01
I2C_SLV0_ADDR       = 0x03
I2C_SLV0_REG        = 0x04
I2C_SLV0_CTRL       = 0x05
I2C_SLV0_DO         = 0x06
I2C_SLV4_ADDR       = 0x13
I2C_SLV4_REG        = 0x14
I2C_SLV4_CTRL       = 0x15
I2C_SLV4_DO         = 0x16
I2C_SLV4_DI         = 0x17

## ICM REGISTERS VALUES ##
A_DLPF_1 = (0x01 << 3)    # 218.1 Hz   ACCELEROMETER
A_DLPF_2 = (0x02 << 3)    # 99 Hz      DIGITAL
A_DLPF_3 = (0x03 << 3)    # 44.8 Hz    LOW
A_DLPF_4 = (0x04 << 3)    # 21.2 Hz    PASS
A_DLPF_5 = (0x05 << 3)    # 10.2 Hz    FILTER
A_DLPF_6 = (0x06 << 3)    # 5.05 Hz    CONFIGURATION
A_FS_SEL_2G         = 0x00
A_FS_SEL_16G        = 0x06
A_DLPF_ENABLE       = 0x01
BYPASS_EN           = 0x02
I2C_MST_CTRL        = 0x01

## AK09916 REGS ADDRESSES ##
MAG_WIA2            = 0x01
MAG_ST1             = 0x10
MAG_HXL             = 0x11
MAG_HXH             = 0x12
MAG_HYL             = 0x13
MAG_HYH             = 0x14
MAG_HZL             = 0x15
MAG_HZH             = 0x16
MAG_ST2             = 0x18
MAG_CNTL2           = 0x31
MAG_CNTL3           = 0x32

## AK09916 REGS VALUES ##
PWDOWN_MODE         = 0x00
SINGLE_MODE         = 0x01
CONT_10HZ_MODE      = 0x02
CONT_20HZ_MODE      = 0x04
CONT_50HZ_MODE      = 0x06
CONT_100HZ_MODE     = 0x08
DRDY                = 0x01
DOR                 = 0x02

## GENERAL VALUES ##
SPI_CHANNEL = 0
SPI_FREQ    = 7000000
SPI_MODE    = 0b11

RLED    = 23     #Red led pin for calibration routine
GLED    = 24     #Green led pin for calibration routine
BUTTON  = 25    #Pushbutton pin to start calibration routine

#Scaling parameters
gyro_scale  = 131.0     #scaling parameter for gyroscope readings (divided by)
accel_scale = 16384.0   #scaling parameter for accelerometer readings (divided by)
mag_scale   = 0.15      #scaling parameter for magnetometer readings (multiplied)

#Magnetometer default values for offsets
mag_x_offset = 0
mag_y_offset = 20
mag_z_offset = 35
calibrate = False    #default value for calibration mode
minx = 500
maxx = -500
miny = 500
maxy = -500
minz = 500
maxz = -500
prev_deg = 0
suma_deg = 0

tm_array = []

pi = pigpio.pi()
if not pi.connected:
    raise RuntimeError("Not successfully connected to pigpiod!")

#Open SPI channel 0, 1 MHz, mode 3 (CPOL=1, CPHA=1)
ICM_SPI = pi.spi_open(SPI_CHANNEL, SPI_FREQ, SPI_MODE)

pi.set_mode(BUTTON, pigpio.INPUT)
pi.set_pull_up_down(BUTTON, pigpio.PUD_UP)
pi.set_mode(RLED, pigpio.OUTPUT)
pi.write(RLED, 0)
pi.set_mode(GLED, pigpio.OUTPUT)
pi.write(GLED, 0)

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
    time.sleep(0.010)
    spi_select_bank(3)
    spi_write_byte(ICM_SPI, I2C_MST_CTRL, 0x07)

def i2c_master_read(slave_addr, slave_reg, length):
    spi_select_bank(3)
    spi_write_byte(ICM_SPI, I2C_SLV0_ADDR, slave_addr | 0x80)
    spi_write_byte(ICM_SPI, I2C_SLV0_REG, slave_reg)
    spi_write_byte(ICM_SPI, I2C_SLV0_CTRL, 0x80 | length)
    time.sleep(0.010)

def i2c_master_write(slave_addr, slave_reg, value):
    spi_select_bank(3)
    spi_write_byte(ICM_SPI, I2C_SLV4_ADDR, slave_addr & 0x7F)
    spi_write_byte(ICM_SPI, I2C_SLV4_REG, slave_reg)
    spi_write_byte(ICM_SPI, I2C_SLV4_DO, value)
    spi_write_byte(ICM_SPI, I2C_SLV4_CTRL, 0x80)
#    print("DO: ", spi_read_byte(ICM_SPI, I2C_SLV4_DO), "CTRL: ", spi_read_byte(ICM_SPI, I2C_SLV4_CTRL))
    time.sleep(0.010)

def set_accel_config(val):
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
    fichero = open('/home/pi/repositories/dronepoint-2/config/mag_calib.txt','r')
    mag_offsets = format_data(fichero)[0]

    fichero.close()
    return mag_offsets[0], mag_offsets[1], mag_offsets[2]

#Reset and wake up the ICM20948
reset_icm()
set_normal_mode()
enable_i2c_master_ctl()

#Reset and wait for 1 ms
i2c_master_write(MAG_ADD, MAG_CNTL3, 0x01)
time.sleep(0.001)

i2c_master_write(MAG_ADD, MAG_CNTL2, CONT_100HZ_MODE)
time.sleep(0.010)

try:
    if(pi.read(BUTTON) == False):    #if the pushbutton is pressed from the beginning go into the manual calibration mode
        pi.write(RLED, 1)    #and turn on the red led
        fichero = open('/home/pi/repositories/dronepoint-2/mag_calib.txt','w')    #open the calibration text file
		#Now there is an algorithm to detect a total turn of the drone (sensor) on the axes we will calibrate (X and Y)
        while (suma_deg < 360):
            print(suma_deg)
            i2c_master_read(MAG_ADD, MAG_HXL, 8)
            spi_select_bank(0)
            raw_mag = spi_read_block(ICM_SPI, EXT_SLV_SENS_DATA_00, 8)
            if((raw_mag[7] & 0x08) != 0x08):
                raw_mag_x = twos_comp((raw_mag[1] << 8) + raw_mag[0])
                raw_mag_y = twos_comp((raw_mag[3] << 8) + raw_mag[2])

                if raw_mag_x < minx:
                    minx = raw_mag_x
                if raw_mag_y < miny:
                    miny = raw_mag_y
                if raw_mag_x > maxx:
                    maxx = raw_mag_x
                if raw_mag_y > maxy:
                    maxy = raw_mag_y

            inc_deg = north_to_deg(round(raw_mag_x * mag_scale, 3),round(raw_mag_y * mag_scale, 3)) - prev_deg
            if(inc_deg < 0 and abs(inc_deg) > 180):
                inc_deg += 360
            if(prev_deg and inc_deg < 180):
                suma_deg += inc_deg

            prev_deg = north_to_deg(round(raw_mag_x * mag_scale, 3),round(raw_mag_y * mag_scale, 3))
            print("XY-PLANE","Suma:", suma_deg, "Deg:", prev_deg, "uT_x:", raw_mag_x * mag_scale)
            time.sleep(0.05)

        mag_x_offset = (maxx + minx) / 2
        mag_y_offset = (maxy + miny) / 2

        pi.write(RLED, 0)
        pi.write(GLED, 1)

		#Again the TTDA (Total Turn Detection Algorithm) for XZ plane (Z axis calibration)
        prev_deg = 0
        suma_deg = 0
        while (suma_deg< 360):
            i2c_master_read(MAG_ADD, MAG_HXL, 8)
            spi_select_bank(0)
            raw_mag = spi_read_block(ICM_SPI, EXT_SLV_SENS_DATA_00, 8)
            if((raw_mag[7] & 0x08) != 0x08):
                raw_mag_x = twos_comp((raw_mag[1] << 8) + raw_mag[0])
                raw_mag_z = twos_comp((raw_mag[5] << 8) + raw_mag[4])

                if raw_mag_z < minz:
                    minz = raw_mag_z
                if raw_mag_z > maxz:
                    maxz = raw_mag_z

            inc_deg = north_to_deg(round(raw_mag_x * mag_scale, 3),round(raw_mag_z * mag_scale, 3)) - prev_deg
            if(inc_deg < 0 and abs(inc_deg) > 180):
                inc_deg += 360
            if(prev_deg and inc_deg < 180):
                suma_deg += inc_deg

            prev_deg = north_to_deg(round(raw_mag_x * mag_scale, 3),round(raw_mag_z * mag_scale, 3))
            print("XZ-PLANE","Suma:", suma_deg, "Deg:", prev_deg, "uT_x:", raw_mag_x * mag_scale)
            time.sleep(0.05)

        mag_z_offset = (maxz + minz) / 2
        pi.write(GLED, 0)
        print("mag_x_offset:",mag_x_offset,"mag_y_offset:",mag_y_offset,"mag_z_offset:",mag_z_offset)

        #Save offset values on the calibration text file
        print(mag_x_offset, mag_y_offset, mag_z_offset, file = fichero)
        fichero.close()

    else:
        #if the pushbutton is not pressed, the offset values are read from the calibration text file
        load_mag_offsets()

    while True:
        i2c_master_read(MAG_ADD, MAG_HXL, 8)
        spi_select_bank(0)
        raw_mag = spi_read_block(ICM_SPI, EXT_SLV_SENS_DATA_00, 8)
        if((raw_mag[7] & 0x08) != 0x08):    #if there is a read value make the conversions
            raw_mag_x = twos_comp((raw_mag[1] << 8) + raw_mag[0])
            raw_mag_y = twos_comp((raw_mag[3] << 8) + raw_mag[2])
            raw_mag_z = twos_comp((raw_mag[5] << 8) + raw_mag[4])

            if raw_mag_x < minx:
                minx = raw_mag_x
            if raw_mag_y < miny:
                miny = raw_mag_y
            if raw_mag_x > maxx:
                maxx = raw_mag_x
            if raw_mag_y > maxy:
                maxy = raw_mag_y
            if raw_mag_z < minz:
                minz = raw_mag_z
            if raw_mag_z > maxz:
                maxz = raw_mag_z

            mag_x = round((raw_mag_x * mag_scale - mag_x_offset),3)
            mag_y = round((raw_mag_y * mag_scale - mag_y_offset),3)
            mag_z = round((raw_mag_z * mag_scale - mag_z_offset),3)

            north_deg = north_to_deg(mag_x, mag_y)
            #make a compensation of the values, necessary due to the tilt of the sensor
            (mag_x_comp, mag_y_comp) = tilt_compensation(mag_x, mag_y, mag_z, 0, 0)
            north_deg_comp = north_to_deg(mag_x_comp, mag_y_comp)

            print(f"MAG_XYZ:({mag_x:8.3f},{mag_y:8.3f}, {mag_z:8.3f} \t CMAG_XYZ: {mag_x_comp:8.3f}, {mag_y_comp:8.3f} \t ORIENTATION: {north_deg:.3f} \t Orientation_comp: {north_deg_comp:.3f}")
#            print("{0:.4f} {1:.4f}".format(mag_x, mag_y))
            time.sleep(0.005)

except KeyboardInterrupt:
    print("minx: {0}, maxx: {1}; miny: {2}, maxy: {3}; minz: {4}, maxz: {5};".format(minx,maxx,miny,maxy,minz,maxz))
    scalex = (maxx-minx)/2
    scaley = (maxy-miny)/2
    scalez = (maxz-minz)/2
    scale_average = (scalex+scaley+scalez)/3.0
    scalex = scale_average/scalex
    scaley = scale_average/scaley
    scalez = scale_average/scalez
    print(f"scalex(uT):{scalex}; scaley(uT):{scaley}; scalez(uT):{scalez};")
