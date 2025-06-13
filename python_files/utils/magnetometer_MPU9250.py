# -*- coding: utf-8 -*-
#!/usr/bin/python

import smbus
import math
import time
import RPi.GPIO as GPIO

#MPU9250 Address
MPU_ADD = 0x68
#AK8963 Address
MAG_ADD = 0x0C
#MPU9250 Registers Addresses
PWR_MGMT_1 = 0x6B
INT_PIN_CFG = 0x37
#Values
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

RLED = 0    #Red led pin for calibration routine
GLED = 1    #Green led pin for calibration routine
BUTTON = 25    #Pushbutton pin to start calibration routine

mag_scale = 4912.0/32760.0    #scaling parameter for magnetometer readings
mag_x_offset = -10    #magnetometer default values for offsets
mag_y_offset = 10
mag_z_offset = 10
calibrate = False    #default value for calibration mode
minx = 0
maxx = 0
miny = 0
maxy = 0
minz = 0
maxz = 0
prev_deg = 0
suma_deg = 0

GPIO.setmode(GPIO.BCM)
GPIO.setup(BUTTON, GPIO.IN, pull_up_down = GPIO.PUD_UP)
GPIO.setup(RLED, GPIO.OUT, initial = GPIO.LOW)
GPIO.setup(GLED, GPIO.OUT, initial = GPIO.LOW)

bus = smbus.SMBus(1)

def read_byte(address, adr):
    return bus.read_byte_data(address, adr)

def write_byte(address, adr, value):
    return bus.write_byte_data(address, adr, value)

def read_block(address, adr, size):
    return bus.read_i2c_block_data(address, adr, size)

def twos_comp(val):
    if (val >= 0x8000):
        return -((65535 - val) + 1)
    else:
        return val

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
    fichero = open('../../config/mag_calib.txt','r')
    mag_offsets = format_data(fichero)[0]

    mag_x_offset = mag_offsets[0]
    mag_y_offset = mag_offsets[1]
    mag_z_offset = mag_offsets[2]

    fichero.close()
    #print("mag_x_offset:",mag_x_offset,"mag_y_offset:",mag_y_offset,"mag_z_offset:",mag_z_offset)

#IMU wake-up and bypass enable
write_byte(MPU_ADD, PWR_MGMT_1, 0x01)
write_byte(MPU_ADD, INT_PIN_CFG, BYPASS_EN)
time.sleep(0.01)

#Power down and Fuse ROM access mode established
write_byte(MAG_ADD, CNTL_1, PWDOWN_MODE)
time.sleep(0.01)
write_byte(MAG_ADD, CNTL_1, FUSE_ROM_MODE)
time.sleep(0.01)

#Readings of magnetometer sensitivity data
cf_data = read_block(MAG_ADD, ASA, 3)
mag_x_cf = (cf_data[0] - 128) / 256.0 + 1
mag_y_cf = (cf_data[1] - 128) / 256.0 + 1
mag_z_cf = (cf_data[2] - 128) / 256.0 + 1

print("x_cf:{} y_cf: {} z_cf: {}".format(mag_x_cf, mag_y_cf, mag_z_cf))

#Power down for 10 ms
write_byte(MAG_ADD, CNTL_1, PWDOWN_MODE)
time.sleep(0.01)

#Setting 16bit resolution and continous 8Hz mode
write_byte(MAG_ADD, CNTL_1, C8HZ_MODE|(RES_16_BIT<<4))
time.sleep(0.01)

try:
    if(GPIO.input(BUTTON) == False):    #if the pushbutton is pressed from the beginning go into the manual calibration mode
        GPIO.output(RLED, GPIO.HIGH)    #and turn on the red led
        fichero = open('mag_calib.txt','w')    #open the calibration text file
		#Now there is an algorithm to detect a total turn of the drone (sensor) on the axes we will calibrate (X and Y)
        while (suma_deg < 360):
            print(suma_deg)
            raw_mag = read_block(MAG_ADD, MAG_OUT, 7)
            if((raw_mag[6] & 0x08) != 0x08):
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

            inc_deg = north_to_deg(round(raw_mag_x * mag_x_cf * mag_scale, 3),round(raw_mag_y * mag_y_cf * mag_scale, 3)) - prev_deg
            if(inc_deg < 0 and abs(inc_deg) > 180):
                inc_deg += 360
            if(prev_deg and inc_deg < 180):
                suma_deg += inc_deg

            prev_deg = north_to_deg(round(raw_mag_x * mag_x_cf * mag_scale, 3),round(raw_mag_y * mag_y_cf * mag_scale, 3))
            print("Suma:", suma_deg, "Deg:", prev_deg, "North_x:", raw_mag_x*mag_x_cf*mag_scale)
            time.sleep(0.05)

        mag_x_offset = (maxx + minx) / 2
        mag_y_offset = (maxy + miny) / 2

        GPIO.output(RLED,GPIO.LOW)
        GPIO.output(GLED,GPIO.HIGH)

		#Again the TTDA (Total Turn Detection Algorithm) for XZ plane (Z axis calibration)
        prev_deg = 0
        suma_deg = 0
        while (suma_deg< 360):
            raw_mag = read_block(MAG_ADD, MAG_OUT, 7)
            if((raw_mag[6] & 0x08) != 0x08):
                raw_mag_x = twos_comp((raw_mag[1] << 8) + raw_mag[0])
                raw_mag_z = twos_comp((raw_mag[5] << 8) + raw_mag[4])

                if raw_mag_z < minz:
                    minz = raw_mag_z
                if raw_mag_z > maxz:
                    maxz = raw_mag_z

            inc_deg = north_to_deg(round(raw_mag_x * mag_x_cf * mag_scale, 3),round(raw_mag_z * mag_z_cf * mag_scale, 3)) - prev_deg
            if(inc_deg < 0 and abs(inc_deg) > 180):
                inc_deg += 360
            if(prev_deg and inc_deg < 180):
                suma_deg += inc_deg

            prev_deg = north_to_deg(round(raw_mag_x * mag_x_cf * mag_scale, 3),round(raw_mag_z * mag_z_cf * mag_scale, 3))
            print("Suma:", suma_deg, "Deg:", prev_deg, "North_x:", raw_mag_x * mag_x_cf * mag_scale)
            time.sleep(0.05)

        mag_z_offset = (maxz + minz) / 2
        GPIO.output(GLED, GPIO.LOW)
        print("mag_x_offset:",mag_x_offset,"mag_y_offset:",mag_y_offset,"mag_z_offset:",mag_z_offset)

        #Save offset values on the calibration text file
        print(mag_x_offset, mag_y_offset, mag_z_offset, file = fichero)
        fichero.close()

    else:
        #if the pushbutton is not pressed, the offset values are read from the calibration text file
        load_mag_offsets()

    while True:
        raw_mag = read_block(MAG_ADD, MAG_OUT, 7)
        if((raw_mag[6] & 0x08) != 0x08):    #if there is a read value make the conversions
            raw_mag_x = twos_comp((raw_mag[1] << 8) + raw_mag[0])
            raw_mag_y = twos_comp((raw_mag[3] << 8) + raw_mag[2])
            raw_mag_z = twos_comp((raw_mag[5] << 8) + raw_mag[4])

            mag_x = round((raw_mag_x - mag_x_offset) * mag_x_cf * mag_scale,3)
            mag_y = round((raw_mag_y - mag_y_offset) * mag_y_cf * mag_scale,3)
            mag_z = round((raw_mag_z - mag_z_offset) * mag_z_cf * mag_scale,3)

            north_deg = north_to_deg(mag_x, mag_y)
            #make a compensation of the values, necessary due to the tilt of the sensor
            (mag_x_comp, mag_y_comp) = tilt_compensation(mag_x, mag_y, mag_z, 0, 0)
            north_deg_comp = north_to_deg(mag_x_comp, mag_y_comp)

            print('MAG_XYZ:(', mag_x, ',', mag_y, ',', mag_z, ')','CMAG_XYZ:',mag_x_comp,mag_y_comp ,',\tORIENTATION:', north_deg, 'Orientation_comp:',north_deg_comp)
#            print("{0:.4f} {1:.4f}".format(raw_mag_x, raw_mag_y))
            time.sleep(0.005)

except KeyboardInterrupt:
    fichero.close()
    GPIO.cleanup()
