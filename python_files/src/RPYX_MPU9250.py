# -*- coding: utf-8 -*-
#!/usr/bin/python

import smbus
import math
import time
import pigpio
from statistics import mean

pi = pigpio.pi()
if not pi.connected:
    exit(0)

RFMOT_PIN = 13
LFMOT_PIN = 27
LBMOT_PIN = 12
RBMOT_PIN = 22
PWM_FREQ = 2000

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

#General constants and parameters
init_thrust = 10
landing_thrust = 24
thrust = 42             #Base propulsion level of the motors
ref = [1.0, 1.0, 60]    #Reference angle values
abs_horizontal_correction = [-1.0, 1.0]
CF_alpha = 0.98

#Roll and Pitch control systems constants and parameters
Kp = 1.20 #1.00     #Proportional PID constant
Ki = 2.60 #1.50     #Integral PID constant
Kd = 0.28 #0.14     #Derivative PID constant
roll_SumaEr = 0     #Error's accumulation (integral)
pitch_SumaEr = 0    #Error's accumulation (integral)
prev_roll_error = 0
prev_pitch_error = 0
roll_pid = 0
pitch_pid = 0

#Yaw control system constants and parameters
Kpy = 2.0 #3.00 #1.8     #Proportional PID constant
Kiy = 2.0 #0.80 #0.6     #Integral PID constant
Kdy = 0.0 #0.02          #Derivative PID constant
yaw_SumaEr = 0      #Error's accumulation (integral)
prev_yaw_error = 0
yaw_pid = 0

#Scaling parameters
gyro_scale = 131.0          #scaling parameter for gyroscope readings
accel_scale = 16384.0       #scaling parameter for accelerometer readings
mag_scale = 4912.0/32760.0  #scaling parameter for magnetometer readings

#Calibration parameters
mag_x_offset = -5
mag_y_offset = 10
mag_z_offset = -5

#Printing arrays
tm_array = []
tp_array = []
cfx_array = []
cfy_array = []
heading_array = []
roll_er_array = []
pitch_er_array = []
yaw_er_array = []
roll_pid_array = []
pitch_pid_array = []
yaw_pid_array = []
ref0_array = []
ref1_array = []
ref2_array = []

pi.set_PWM_frequency(LFMOT_PIN,PWM_FREQ)
pi.set_PWM_frequency(RBMOT_PIN,PWM_FREQ)
pi.set_PWM_frequency(RFMOT_PIN,PWM_FREQ)
pi.set_PWM_frequency(LBMOT_PIN,PWM_FREQ)
pi.set_PWM_range(LFMOT_PIN,100)
pi.set_PWM_range(RBMOT_PIN,100)
pi.set_PWM_range(RFMOT_PIN,100)
pi.set_PWM_range(LBMOT_PIN,100)
pi.set_PWM_dutycycle(LFMOT_PIN,0)
pi.set_PWM_dutycycle(RBMOT_PIN,0)
pi.set_PWM_dutycycle(RFMOT_PIN,0)
pi.set_PWM_dutycycle(LBMOT_PIN,0)
#pi.hardware_PWM(RFMOT_PIN,PWM_FREQ,0) # doesn't seem to do any good in control loop time because the bottleneck seems to be in the communication with the daemon itself
#pi.hardware_PWM(LBMOT_PIN,PWM_FREQ,0) # doesn't seem to do any good in control loop time because the bottleneck seems to be in the communication with the daemon itself

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
#    print("a_x:%d|%d, a_y:%d|%d, a_z:%d|%d " % (raw_data[0], raw_data[1], raw_data[2], raw_data[3], raw_data[4], raw_data[5]), end='')
#    print("g_x:%d|%d, g_y:%d|%d, g_z:%d|%d" % (raw_data[8], raw_data[9], raw_data[10], raw_data[11], raw_data[12], raw_data[13]))

    accel_scaled_x = twos_comp((raw_data[0] << 8) + raw_data[1]) / accel_scale
    accel_scaled_y = twos_comp((raw_data[2] << 8) + raw_data[3]) / accel_scale
    accel_scaled_z = twos_comp((raw_data[4] << 8) + raw_data[5]) / accel_scale

#    temp = twos_comp((raw_data[6] << 8) + raw_data[7]) / 321 + 21

    gyro_scaled_x = twos_comp((raw_data[8] << 8) + raw_data[9]) / gyro_scale
    gyro_scaled_y = twos_comp((raw_data[10] << 8) + raw_data[11]) / gyro_scale
    gyro_scaled_z = twos_comp((raw_data[12] << 8) + raw_data[13]) / gyro_scale

    return (gyro_scaled_x, gyro_scaled_y, gyro_scaled_z, accel_scaled_x, accel_scaled_y, accel_scaled_z)

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
    fichero = open('../../config/mag_calib.txt','r')
    mag_offsets = format_data(fichero)[0]

    fichero.close()
    return mag_offsets[0], mag_offsets[1], mag_offsets[2]

def load_accel_offsets():
    fichero = open('../../config/accel_calib.txt','r')
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
#write_gyro_offsets_to_MPU(255,126,0,18,0,0)
#write_gyro_offsets_to_MPU(0,0,0,0,0,0)

prev_time = time.time()

#Read accelerometer and gyroscope raw values and get the scaled values
(gyro_scaled_x, gyro_scaled_y, gyro_scaled_z, accel_scaled_x, accel_scaled_y, accel_scaled_z) = read_all()
#print("accel_scaled_x:%.4f, accel_scaled_y:%.4f, accel_scaled_z:%.4f " % (accel_scaled_x, accel_scaled_y, accel_scaled_z),end='')
#print("gyro_scaled_x:%.4f, gyro_scaled_y:%.4f, gyro_scaled_z:%.4f" % (gyro_scaled_x, gyro_scaled_y, gyro_scaled_z))
#First complimentary filter (CF) variables assignments only with accelerometer readings
CF_x = get_x_rotation(accel_scaled_x, accel_scaled_y, accel_scaled_z)
CF_y = get_y_rotation(accel_scaled_x, accel_scaled_y, accel_scaled_z)
#print("CF_x:%lf, CF_y:%lf, dist_yz:%lf, dist_xz:%lf" % (CF_x,CF_y,dist(accel_scaled_y,accel_scaled_z),dist(accel_scaled_x,accel_scaled_z)))
#Create an offset with the first read value
gyro_offset_x = gyro_scaled_x
gyro_offset_y = gyro_scaled_y
#Sets first gyroscope values from the CF variables
gyro_total_x = CF_x
gyro_total_y = CF_y

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
#print("x_cf:{} y_cf:{} z_cf:{}".format(mag_x_cf, mag_y_cf, mag_z_cf))

#Power down for 10 ms
write_byte(MAG_ADD, CNTL_1, PWDOWN_MODE)
time.sleep(0.01)

#Setting 16bit resolution and continous 100Hz mode
write_byte(MAG_ADD, CNTL_1, C100HZ_MODE|(RES_16_BIT<<4))
time.sleep(0.01)

#print ("{0:.4f} {1:.2f} {2:.2f} {3:.2f} {4:.2f} {5:.2f} {6:.2f}".format( time.time() - initial, (CF_x), gyro_total_x, (CF_x), (CF_y), gyro_total_y, (CF_y)))
try:
    mag_x_offset, mag_y_offset, mag_z_offset = load_mag_offsets()
    #print("mag_x_offset:{} mag_y_offset:{} mag_z_offset:{}".format(mag_x_offset, mag_y_offset, mag_z_offset))

    raw_mag = read_block(MAG_ADD, MAG_OUT, 7)
    time.sleep(0.01)
    if((raw_mag[6] & 0x08) != 0x08):    #if there is a read value make the conversions
        raw_mag_x = twos_comp((raw_mag[1] << 8) + raw_mag[0])
        raw_mag_y = twos_comp((raw_mag[3] << 8) + raw_mag[2])
        raw_mag_z = twos_comp((raw_mag[5] << 8) + raw_mag[4])
#        print("raw_x:%d, raw_y:%d, raw_z:%d" % (raw_mag_x, raw_mag_y,raw_mag_z))
        mag_x = round((raw_mag_x - mag_x_offset) * mag_x_cf * mag_scale, 3)
        mag_y = round((raw_mag_y - mag_y_offset) * mag_y_cf * mag_scale, 3)
        mag_z = round((raw_mag_z - mag_z_offset) * mag_z_cf * mag_scale, 3)
#        print("mag_x:%lf, mag_y:%lf, mag_z:%lf" % (mag_x, mag_y,mag_z))
        #north_deg = north_to_deg(mag_x, mag_y)
        #make a compensation of the values, necessary due to the tilt of the sensor
        mag_x_comp, mag_y_comp = tilt_compensation(mag_x, mag_y, mag_z, math.radians(CF_y), math.radians(CF_x))
#        print("comp_x:%lf, comp_y:%lf, CF_y:%lf, CF_x:%lf" % (mag_x_comp,mag_y_comp,CF_y,CF_x))
        north_deg_comp = north_to_deg(mag_x_comp, mag_y_comp)

    ref[2] = north_deg_comp
#    print("ndegcomp:",north_deg_comp)

    CF_x_total = 0
    CF_y_total = 0

    for i in range(1,500+1):
        cur_time = time.time()
        Tm = (cur_time - prev_time) #Sample time variable measures time between loop code runs
        prev_time = cur_time        #This variable keeps current time as previous one for the next Tm assignment
        #read accelerometer and gyroscope raw values and get the scaled values
        (gyro_scaled_x, gyro_scaled_y, gyro_scaled_z, accel_scaled_x, accel_scaled_y, accel_scaled_z) = read_all()
        #substract the offset from the gyro scaled value
        gyro_scaled_x -= gyro_offset_x
        gyro_scaled_y -= gyro_offset_y
        #calculate the increment of angle
        gyro_x_delta = (gyro_scaled_x * Tm)
        gyro_y_delta = (gyro_scaled_y * Tm)
        #calculate the current angle
        gyro_total_x += gyro_x_delta
        gyro_total_y += gyro_y_delta
        #get the rotations based only on accelerometer readings
        rotation_x = get_x_rotation(accel_scaled_x, accel_scaled_y, accel_scaled_z)
        rotation_y = get_y_rotation(accel_scaled_x, accel_scaled_y, accel_scaled_z)

        #CF assignment with gyroscope readings and accelerometer readings
        #CF calculation ponderates readings from both sensors,
        #that derives into more reliable readings, from accelerometer at lower speeds
        #and from gyroscope at greater ones
        CF_x = 0.96 * (CF_x + gyro_x_delta) + (0.04 * rotation_x)
        CF_x_total += CF_x
        CF_y = 0.96 * (CF_y + gyro_y_delta) + (0.04 * rotation_y)
        CF_y_total += CF_y

    CF_x_average = CF_x_total/500.0
    CF_y_average = CF_y_total/500.0
#    print("CFX_av.{:+07.4f}, CFY_av:{:+07.4f}  ".format(CF_x_average, CF_y_average))

    ref[0] = CF_x_average + abs_horizontal_correction[0]
    ref[1] = CF_y_average + abs_horizontal_correction[1]
    step = 0
    initial_time = time.time()

    prev_d_roll_pid = 0
    prev_d_pitch_pid = 0

    while True:    #keep running until a key press
        cur_time = time.time()
        Tm = (cur_time - prev_time)    #Sample time variable measures time between loop code runs
        prev_time = cur_time        #this variable keeps current time as previous one for the next Tm assignment

        #read accelerometer and gyroscope raw values and get the scaled values
        (gyro_scaled_x, gyro_scaled_y, gyro_scaled_z, accel_scaled_x, accel_scaled_y, accel_scaled_z) = read_all()
#        print("accel_scaled_x:%.4f, accel_scaled_y:%.4f, accel_scaled_z:%.4f " % (accel_scaled_x, accel_scaled_y, accel_scaled_z),end='')
#        print("gyro_scaled_x:%.4f, gyro_scaled_y:%.4f, gyro_scaled_z:%.4f" % (gyro_scaled_x, gyro_scaled_y, gyro_scaled_z))

        #substract the offset from the gyro scaled value
        gyro_scaled_x -= gyro_offset_x
        gyro_scaled_y -= gyro_offset_y
        #calculate the increment of angle
        gyro_x_delta = (gyro_scaled_x * Tm)
        gyro_y_delta = (gyro_scaled_y * Tm)
        #calculate the current angle
        gyro_total_x += gyro_x_delta
        gyro_total_y += gyro_y_delta
        #get the rotations based only on accelerometer readings
        rotation_x = get_x_rotation(accel_scaled_x, accel_scaled_y, accel_scaled_z)
        rotation_y = get_y_rotation(accel_scaled_x, accel_scaled_y, accel_scaled_z)

        #CF assignment with gyroscope readings and accelerometer readings
        #CF calculation ponderates readings from both sensors,
        #that derives into more reliable readings, from accelerometer at lower speeds
        #and from gyroscope at greater ones
        CF_x = CF_alpha * (CF_x + gyro_x_delta) + ((1-CF_alpha) * rotation_x)
        roll_error = CF_x - ref[0]
        CF_y = CF_alpha * (CF_y + gyro_y_delta) + ((1-CF_alpha) * rotation_y)
        pitch_error = CF_y - ref[1]
        #print("Roll_Err:{:+08.4f}, Pitch_Err:{:+08.4f}  ".format(roll_error, pitch_error), end = '')

        raw_mag = read_block(MAG_ADD, MAG_OUT, 7)
        if((raw_mag[6] & 0x08) != 0x08):    #if there is a read value make the conversions
            raw_mag_x = twos_comp((raw_mag[1] << 8) + raw_mag[0])
            raw_mag_y = twos_comp((raw_mag[3] << 8) + raw_mag[2])
            raw_mag_z = twos_comp((raw_mag[5] << 8) + raw_mag[4])

            mag_x = round((raw_mag_x - mag_x_offset) * mag_x_cf * mag_scale, 3)
            mag_y = round((raw_mag_y - mag_y_offset) * mag_y_cf * mag_scale, 3)
            mag_z = round((raw_mag_z - mag_z_offset) * mag_z_cf * mag_scale, 3)

            #north_deg = north_to_deg(mag_x, mag_y)
            #make a compensation of the values, necessary due to the tilt of the sensor
            mag_x_comp, mag_y_comp = tilt_compensation(mag_x, mag_y, mag_z, math.radians(CF_y), math.radians(CF_x))
            north_deg_comp = north_to_deg(mag_x_comp, mag_y_comp)

            #print('MAG_XYZ:(', mag_x, ',', mag_y, ',', mag_z, ')','CMAG_XYZ:',mag_x_comp,mag_y_comp ,', ORIENTATION:', north_deg, 'COMP_Orientation:',north_deg_comp,end='')
            #print("%.4f %.4f}" % (raw_mag_x,raw_mag_y))
            #time.sleep(0.005)

        yaw_error = north_deg_comp - ref[2]

        if yaw_error > 180:
            yaw_error -= 360
        elif yaw_error < -180:
            yaw_error += 360

        #print("yaw_Er:", yaw_error, " ", end='')

        #set initial thrust limitation
        if (cur_time - initial_time < 0.5):
            pwm_LF = init_thrust
            pwm_RF = init_thrust
            pwm_LB = init_thrust
            pwm_RB = init_thrust

            prev_roll_error = roll_error
            prev_pitch_error = pitch_error
            prev_yaw_error = yaw_error

        else:
            ##ROLL ERROR CALCS
            roll_IncEr = roll_error - prev_roll_error       #Error increment
            #roll_SumaEr += roll_error                       #Error accumulation (integral)(Euler II)
            roll_SumaEr += (roll_error + prev_roll_error)/2 #Error accumulation (integral)(Tustin)

#            if (prev_roll_error * roll_error < 0) :         #Anti-windup mechanism
#                roll_SumaEr = 0

            #prev_roll_error = roll_error                    #Previous roll error update

            ##PITCH ERROR CALCS
            pitch_IncEr = pitch_error - prev_pitch_error    #Error increment
            #pitch_SumaEr += pitch_error                     #Error accumulation (integral)(Euler II)
            pitch_SumaEr += (pitch_error + prev_pitch_error)/2 #Error accumulation (integral)(Tustin)

#            if (prev_pitch_error * pitch_error < 0) :       #Anti-windup mechanism
#                pitch_SumaEr = 0

            #prev_pitch_error = pitch_error                  #Previous pitch error update

            ##YAW ERROR CALCS
            yaw_IncEr = yaw_error - prev_yaw_error          #Error increment
            #yaw_SumaEr += yaw_error                         #Error accumulation (integral)(Euler II)
            yaw_SumaEr += (yaw_error + prev_yaw_error)/2 #Error accumulation (integral)(Tustin)

#            if (prev_yaw_error * yaw_error < 0) :           #Anti-windup mechanism
#                yaw_SumaEr = 0

            #prev_yaw_error = yaw_error                      #Previous yaw error update

            #PID controllers calculation (parallel)
            #Roll derivative PID term filtering
            cur_d_roll_pid = Kd * roll_IncEr / Tm
            d_roll_pid = 0.5 * cur_d_roll_pid + 0.5 * prev_d_roll_pid
            prev_d_roll_pid = cur_d_roll_pid

            roll_pid = (Kp * roll_error) + d_roll_pid + (Ki * roll_SumaEr * Tm)
            #Pitch derivative PID term filtering
            cur_d_pitch_pid = Kd * pitch_IncEr / Tm
            d_pitch_pid = 0.5 * cur_d_pitch_pid + 0.5 * prev_d_pitch_pid
            prev_d_pitch_pid = cur_d_pitch_pid

            pitch_pid = (Kp * pitch_error) + d_pitch_pid + (Ki * pitch_SumaEr * Tm)

            yaw_pid = (Kpy * yaw_error) + (Kdy * yaw_IncEr / Tm) + (Kiy * yaw_SumaEr * Tm)
            #print("Roll_Pid:{:+08.4f}, Pitch_Pid:{:+08.4f}, Yaw_Pid:{:+08.4f}  ".format(roll_pid, pitch_pid, yaw_pid))

            #Anti-windup mechanism: Don't increment error integral while saturated pid values
            if(roll_pid > 100 or roll_pid < -100):
                roll_SumaEr -= (roll_error + prev_roll_error)/2

            if(pitch_pid > 100 or pitch_pid < -100):
                pitch_SumaEr -= (roll_error + prev_roll_error)/2

            if(yaw_pid > 100 or yaw_pid < -100):
                yaw_SumaEr -= (yaw_error + prev_yaw_error)/2

            prev_roll_error = roll_error
            prev_pitch_error = pitch_error
            prev_yaw_error = yaw_error

            pwm_LF = thrust + roll_pid - pitch_pid + yaw_pid -6  #PWM motorL value (left-front) - Extra 6 points compensation due to parcially burned red-blue wires motors
            pwm_RB = thrust - roll_pid + pitch_pid + yaw_pid -6  #PWM motorR value (right-back) - Extra 6 points compensation due to parcially burned red-blue wires motors
            pwm_RF = thrust - roll_pid - pitch_pid - yaw_pid +6  #PWM motorF value (right-front) - Extra 6 points compensation due to parcially burned red-blue wires motors
            pwm_LB = thrust + roll_pid + pitch_pid - yaw_pid +6  #PWM motorB value (left-back) - Extra 6 points compensation due to parcially burned red-blue wires motors


#            print(ref[0]," ",CF_x)
#            if (cur_time - initial_time >=1.5 and cur_time - initial_time < 1.6 and step == 0):
#                ref[0] -= 5
#                step = 1
#                print(ref[0]," ",CF_x)
#            elif (cur_time - initial_time >= 2.5 and cur_time - initial_time < 2.6 and step == 1):
#                ref[0] += 5
#                step = 2
#                print("step 2")

        if(pwm_LF < 2):
            pwm_LF = 2
        elif(pwm_LF > 98):      #PWM
            pwm_LF = 98         #
        if(pwm_RB < 2):         #SATURATION
            pwm_RB = 2          #
        elif(pwm_RB > 98):      #FILTER
            pwm_RB = 98

        if(pwm_RF < 2):
            pwm_RF = 2
        elif(pwm_RF > 98):      #PWM
            pwm_RF = 98         #
        if(pwm_LB < 2):         #SATURATION
            pwm_LB = 2          #
        elif(pwm_LB > 98):      #FILTER
            pwm_LB = 98

#        print("PWM_LF:{:06.4f}, PWM_RB:{:06.4f} ".format(pwm_LF, pwm_RB),end='')
#        print("PWM_RF:{:06.4f}, PWM_LB:{:06.4f}".format(pwm_RF, pwm_LB))

        #motor control pins PWM value assignments
        pi.set_PWM_dutycycle(LFMOT_PIN,pwm_LF)
        pi.set_PWM_dutycycle(RBMOT_PIN,pwm_RB)
        pi.set_PWM_dutycycle(LBMOT_PIN,pwm_LB)
        pi.set_PWM_dutycycle(RFMOT_PIN,pwm_RF)
#        pi.hardware_PWM(RFMOT_PIN,PWM_FREQ,int(pwm_RF*10000))
#        pi.hardware_PWM(LBMOT_PIN,PWM_FREQ,int(pwm_LB*10000))

#        print("elapsed:{0:.4f} Acc_x:{1:.2f} Gyr_T_x:{2:.2f} CF_x:{3:.2f} Acc_y:{4:.2f} Gyr_T_y:{5:.2f} CF_y:{6:.2f}".format(time.time()-initial_time,(rotation_x),(gyro_total_x),(CF_x),(rotation_y),(gyro_total_y),(CF_y)))
#        print("{0:.4f} {1:.2f} {2:.2f} {3:.2f} {4:.2f} {5:.2f} {6:.2f}".format(time.time()-initial_time,(rotation_x),(gyro_total_x),(CF_x),(rotation_y),(gyro_total_y),(CF_y)))
#        print("%.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f" % (cur_time-initial_time,CF_x,CF_y,north_deg_comp,roll_error,pitch_error,yaw_error,roll_pid,pitch_pid,yaw_pid,ref[0],ref[1],ref[2]))

        tm_array.append(Tm)
        tp_array.append(cur_time - initial_time)
        cfx_array.append(CF_x)
        cfy_array.append(CF_y)
        heading_array.append(north_deg_comp)
        roll_er_array.append(roll_error)
        pitch_er_array.append(pitch_error)
        yaw_er_array.append(yaw_error)
        roll_pid_array.append(roll_pid)
        pitch_pid_array.append(pitch_pid)
        yaw_pid_array.append(yaw_pid)
        ref0_array.append(ref[0])
        ref1_array.append(ref[1])
        ref2_array.append(ref[2])

except KeyboardInterrupt:
    #exception routine for a clean exit
    pi.set_PWM_dutycycle(LFMOT_PIN,landing_thrust)
    pi.set_PWM_dutycycle(RBMOT_PIN,landing_thrust)
    pi.set_PWM_dutycycle(RFMOT_PIN,landing_thrust)
    pi.set_PWM_dutycycle(LBMOT_PIN,landing_thrust)
#    pi.hardware_PWM(RFMOT_PIN,PWM_FREQ,landing_thrust*10000)
#    pi.hardware_PWM(LBMOT_PIN,PWM_FREQ,landing_thrust*10000)
    time.sleep(0.20)
    pi.set_PWM_dutycycle(LFMOT_PIN,0)
    pi.set_PWM_dutycycle(RBMOT_PIN,0)
    pi.set_PWM_dutycycle(RFMOT_PIN,0)
    pi.set_PWM_dutycycle(LBMOT_PIN,0)
#    pi.hardware_PWM(RFMOT_PIN,PWM_FREQ,0)
#    pi.hardware_PWM(LBMOT_PIN,PWM_FREQ,0)
    time.sleep(0.05)
    pi.stop()

finally:
    for i in range(0,len(tm_array)):
        print("%.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f" % (tp_array[i],cfx_array[i],cfy_array[i],heading_array[i],roll_er_array[i],pitch_er_array[i],yaw_er_array[i],roll_pid_array[i],pitch_pid_array[i],yaw_pid_array[i],ref0_array[i],ref1_array[i],ref2_array[i]))
    print("Tm_mean: %.4f" % (mean(tm_array)))
#    print("Comp time:", post_comp_time - pre_comp_time)
