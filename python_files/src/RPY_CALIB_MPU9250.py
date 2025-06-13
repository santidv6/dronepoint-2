# -*- coding: utf-8 -*-
#!/usr/bin/python

import smbus
import math
import time
import RPi.GPIO as GPIO

LMOT_PIN = 12
RMOT_PIN = 13
FMOT_PIN = 22
BMOT_PIN = 27

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
#Values
A_FCHOICE_B = 0x00 << 2
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

#General constants and paramenters
thrust = 50             #Base propulsion level of the motors
ref = [1.86,-1.33,90]   #Reference angle values

#Roll and Pitch control systems constants and parameters
Kp = 2.1 #1.9       #Proportional PID constant
Ki = 0.0 #0.6       #Integral PID constant
Kd = 0.8 #0.5       #Derivative PID constant
roll_SumaEr = 0     #Error's accumulation (integral)
pitch_SumaEr = 0    #Error's accumulation (integral)

#Yaw control system constants and parameters
Kpy = 0.71          #Proportional PID constant
Kiy = 0.25          #Integral PID constant
Kdy = 0.28          #Derivative PID constant
yaw_SumaEr = 0      #Error's accumulation (integral)
yaw_error = 0

#Scaling parameters
gyro_scale = 131.0          #scaling parameter for gyroscope readings
accel_scale = 16384.0       #scaling parameter for accelerometer readings
mag_scale = 4912.0/32760.0  #scaling parameter for magnetometer readings

#Calibration parameters
mag_x_offset = -5
mag_y_offset = 10
mag_z_offset = -5
calibrate = False
minx = 0
maxx = 0
miny = 0
maxy = 0
minz = 0
maxz = 0
prev_deg = 0
suma_deg = 0

RLED = 0    #Red led pin for calibration routine
GLED = 1    #Green led pin for calibration routine
BUTTON = 25 #Pushbutton pin to start calibration routine

GPIO.setmode(GPIO.BCM)
GPIO.setup(LMOT_PIN, GPIO.OUT, initial = GPIO.LOW)
GPIO.setup(RMOT_PIN, GPIO.OUT, initial = GPIO.LOW)
GPIO.setup(FMOT_PIN, GPIO.OUT, initial = GPIO.LOW)
GPIO.setup(BMOT_PIN, GPIO.OUT, initial = GPIO.LOW)
GPIO.setup(BUTTON, GPIO.IN, pull_up_down = GPIO.PUD_UP)
GPIO.setup(RLED, GPIO.OUT, initial = GPIO.LOW)
GPIO.setup(GLED, GPIO.OUT, initial = GPIO.LOW)
motor_L = GPIO.PWM(LMOT_PIN, 3000)
motor_R = GPIO.PWM(RMOT_PIN, 3000)
motor_F = GPIO.PWM(FMOT_PIN, 3000)
motor_B = GPIO.PWM(BMOT_PIN, 3000)
motor_L.start(0)
motor_R.start(0)
motor_F.start(0)
motor_B.start(0)

bus = smbus.SMBus(1)

def read_byte(address, adr):
    return bus.read_byte_data(address, adr)

def write_byte(address, adr, value):
    return bus.write_byte_data(address, adr, value)

def read_block(address, adr, size):
    return bus.read_i2c_block_data(address, adr, size)

def read_all():
    raw_gyro_data = read_block(MPU_ADD, GYRO_XOUT_H, 6)
    raw_accel_data = read_block(MPU_ADD, ACCEL_XOUT_H, 6)

    gyro_scaled_x = twos_comp((raw_gyro_data[0] << 8) + raw_gyro_data[1]) / gyro_scale
    gyro_scaled_y = twos_comp((raw_gyro_data[2] << 8) + raw_gyro_data[3]) / gyro_scale
    gyro_scaled_z = twos_comp((raw_gyro_data[4] << 8) + raw_gyro_data[5]) / gyro_scale

    accel_scaled_x = twos_comp((raw_accel_data[0] << 8) + raw_accel_data[1]) / accel_scale
    accel_scaled_y = twos_comp((raw_accel_data[2] << 8) + raw_accel_data[3]) / accel_scale
    accel_scaled_z = twos_comp((raw_accel_data[4] << 8) + raw_accel_data[5]) / accel_scale

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

#Now wake the MPU9250 as it starts in sleep mode and set magnetometer bypass enable
set_normal_mode()
bypass_mag()
#Set the DLPF filtering frequency (BW)
set_accel_DLPF(A_DLPF_4)
time.sleep(0.01)

prevtime = time.time()
initial = prevtime

#Read accelerometer and gyroscope raw values and get the scaled values
(gyro_scaled_x, gyro_scaled_y, gyro_scaled_z, accel_scaled_x, accel_scaled_y, accel_scaled_z) = read_all()
#First complimentary filter (CF) variables assignments only with accelerometer readings and errors calculations based on reference angles
CF_x = get_x_rotation(accel_scaled_x, accel_scaled_y, accel_scaled_z)
prev_roll_error = CF_x - ref[0]
CF_y = get_y_rotation(accel_scaled_x, accel_scaled_y, accel_scaled_z)
prev_pitch_error = CF_y - ref[1]

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
print("x_cf:{} y_cf:{} z_cf:{}".format(mag_x_cf, mag_y_cf, mag_z_cf))

#Power down for 10 ms
write_byte(MAG_ADD, CNTL_1, PWDOWN_MODE)
time.sleep(0.01)

#Setting 16 bit resolution and continous 8Hz mode
write_byte(MAG_ADD, CNTL_1, C8HZ_MODE|(RES_16_BIT<<4))
time.sleep(0.01)

#print("{0:.4f} {1:.2f} {2:.2f}".format(time.time() - initial, CF_x, CF_y))

prev_yaw_error = yaw_error

try:
    if(GPIO.input(BUTTON)==False):    #if the pushbutton is pressed from the beginning go into the manual calibration mode
        GPIO.output(RLED,GPIO.HIGH)    #and turn on the red led
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
            print("Suma:", suma_deg, "Deg:", prev_deg, "North_x:", raw_mag_x * mag_x_cf * mag_scale)
            time.sleep(0.05)

        mag_x_offset = (maxx + minx) / 2
        mag_y_offset = (maxy + miny) / 2

        GPIO.output(RLED, GPIO.LOW)
        GPIO.output(GLED, GPIO.HIGH)

        #Again the TTDA (Total Turn Detection Algorithm) for XZ plane (Z axis calibration)
        prev_deg = 0
        suma_deg = 0
        while (suma_deg < 360):
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

        #Save offset values on the calibration text file
        print(mag_x_offset, mag_y_offset, mag_z_offset, file = fichero)
        fichero.close()

    else:
        #if the pushbutton is not pressed, the offset values are read from the calibration text file
        mag_x_offset, mag_y_offset, mag_z_offset = load_mag_offsets()
        #print("mag_x_offset:{} mag_y_offset:{} mag_z_offset:{}".format(mag_x_offset, mag_y_offset, mag_z_offset))

    raw_mag = read_block(MAG_ADD, MAG_OUT, 7)
    time.sleep(0.01)
    if((raw_mag[6] & 0x08) != 0x08):    #if there is a read value make the conversions
        raw_mag_x = twos_comp((raw_mag[1] << 8) + raw_mag[0])
        raw_mag_y = twos_comp((raw_mag[3] << 8) + raw_mag[2])
        raw_mag_z = twos_comp((raw_mag[5] << 8) + raw_mag[4])

        mag_x = round((raw_mag_x - mag_x_offset) * mag_x_cf * mag_scale, 3)
        mag_y = round((raw_mag_y - mag_y_offset) * mag_y_cf * mag_scale, 3)
        mag_z = round((raw_mag_z - mag_z_offset) * mag_z_cf * mag_scale, 3)

        north_deg = north_to_deg(mag_x, mag_y)
        #make a compensation of the values, necessary due to the tilt of the sensor
        (mag_x_comp, mag_y_comp) = tilt_compensation(mag_x, mag_y, mag_z, math.radians(CF_y), math.radians(CF_x))
        north_deg_comp = north_to_deg(mag_x_comp, mag_y_comp)

    #Setting initial reference values
    ref[0] = CF_x
    ref[1] = CF_y
    #ref[2] = north_deg_comp


    while True:    #keep running until a key press
        Tm = (time.time() - prevtime)    #Sample time variable measures time between loop code runs
        prevtime = time.time()        #this variable keeps current time as previous one for the next Tm assignment
        #Set reference step
#        if (prevtime - initial > 3 and prevtime - initial < 3.2):
#            thrust = 50
        #read accelerometer and gyroscope raw values and get the scaled values
        (gyro_scaled_x, gyro_scaled_y, gyro_scaled_z, accel_scaled_x, accel_scaled_y, accel_scaled_z) = read_all()
        #print("accel_scaled_x:{:.4f}, accel_scaled_y:{:.4f}, accel_scaled_z:{:.4f} ".format(accel_scaled_x, accel_scaled_y, accel_scaled_z),end='')
        #print("gyro_scaled_x:{:.4f}, gyro_scaled_y:{:.4f}, gyro_scaled_z:{:.4f}".format(gyro_scaled_x, gyro_scaled_y, gyro_scaled_z))

        #substract the offset from the gyro scaled value
        gyro_scaled_x -= gyro_offset_x
        gyro_scaled_y -= gyro_offset_y
        #calculate de increment of angle
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
        CF_x = 0.98 * (CF_x + gyro_x_delta) + (0.02 * rotation_x)
        roll_error = CF_x - ref[0]
        CF_y = 0.98 * (CF_y + gyro_y_delta) + (0.02 * rotation_y)
        pitch_error = CF_y - ref[1]
        #print("Roll_Err:{:+08.4f}, Pitch_Err:{+08.4f}".format(roll_error, pitch_error), end = '')

        raw_mag = read_block(MAG_ADD, MAG_OUT, 7)
        if((raw_mag[6] & 0x08) != 0x08):    #if there is a read value make the conversions
            raw_mag_x = twos_comp((raw_mag[1] << 8) + raw_mag[0])
            raw_mag_y = twos_comp((raw_mag[3] << 8) + raw_mag[2])
            raw_mag_z = twos_comp((raw_mag[5] << 8) + raw_mag[4])

            mag_x = round((raw_mag_x - mag_x_offset) * mag_x_cf * mag_scale, 3)
            mag_y = round((raw_mag_y - mag_y_offset) * mag_y_cf * mag_scale, 3)
            mag_z = round((raw_mag_z - mag_z_offset) * mag_z_cf * mag_scale, 3)

            north_deg = north_to_deg(mag_x, mag_y)
            #make a compensation of the values, necessary due to the tilt of the sensor
            (mag_x_comp, mag_y_comp) = tilt_compensation(mag_x, mag_y, mag_z, math.radians(CF_y), math.radians(CF_x))
            north_deg_comp = north_to_deg(mag_x_comp, mag_y_comp)

            #print('MAG_XYZ:(', mag_x, ',', mag_y, ',', mag_z, ')','CMAG_XYZ:',mag_x_comp,mag_y_comp ,', ORIENTATION:', north_deg, 'Orientation_comp:',north_deg_comp)
            #print "{0:.4f} {1:.4f}".format(raw_mag_x,raw_mag_y)
            #time.sleep(0.005)

        yaw_error = north_deg_comp - ref[2]

        if yaw_error > 180:
            yaw_error -= 360
        elif yaw_error < -180:
            yaw_error += 360

        #print("yaw_Err:{} ".format(yaw_error), end = '')

        ##ROLL ERROR CALCS
        roll_IncEr = roll_error - prev_roll_error       #Error increment
        roll_SumaEr += roll_error                       #Error accumulation (integral)

        if(prev_roll_error * roll_error < 0):           #Anti-Windup mechanism
            roll_SumaEr = 0

        prev_roll_error = roll_error                    #Previous roll error update

        ##PITCH ERROR CALCS
        pitch_IncEr = pitch_error - prev_pitch_error    #Error increment
        pitch_SumaEr += pitch_error                     #Error accumulation (integral)

        if(prev_pitch_error * pitch_error < 0):         #Anti-Windup mechanism
            pitch_SumaEr = 0

        prev_pitch_error = pitch_error                  #Previous pitch error update

        ##YAW ERROR CALCS
        yaw_IncEr = yaw_error - prev_yaw_error          #Error increment
        yaw_SumaEr += yaw_error                         #Error accumulation (integral)

        if(prev_yaw_error * yaw_error < 0):             #Anti-Windup mechanism
            yaw_SumaEr = 0

        prev_yaw_error = yaw_error                      #Previous yaw error update

		#PID controllers calculation (parallel)
        roll_pid = (Kp * roll_error) + (Kd * roll_IncEr / Tm) + (Ki * roll_SumaEr * Tm)
        pitch_pid = (Kp * pitch_error) + (Kd * pitch_IncEr / Tm) + (Ki * pitch_SumaEr * Tm)
        yaw_pid = (Kpy * yaw_error) + (Kdy * yaw_IncEr / Tm) + (Kiy * yaw_SumaEr * Tm)
        #print("Roll_Pid:{:+08.4f}, Pitch_Pid:{:+08.4f}, Yaw_Pid:{:+08.4f}  ".format(roll_pid, pitch_pid, yaw_pid))

#        if(roll_pid > 100 - thrust):	    #PID
#            roll_pid = 100 - thrust		#SATURATION
#        elif(roll_pid < -(100 - thrust)):	#FILTER
#            roll_pid = -(100 - thrust)

#        if(pitch_pid > 100 - thrust):	    #PID
#            pitch_pid = 100 - thrust	    #SATURATION
#        elif(pitch_pid < -(100 - thrust)):	#FILTER
#            pitch_pid = -(100 - thrust)

#        if(yaw_pid > 100 - thrust):         #PID
#            yaw_pid = 100 - thrust          #SATURATION
#        elif(yaw_pid < -(100 - thrust)):	 #FILTER
#            yaw_pid = -(100 - thrust)

        pwm_L = thrust + roll_pid - yaw_pid     #PWM motorL value (left)
        pwm_R = thrust - roll_pid - yaw_pid     #PWM motorR value (right)
        pwm_F = thrust - pitch_pid + yaw_pid    #PWM motorF value (front)
        pwm_B = thrust + pitch_pid + yaw_pid    #PWM motorB value (back)

        if(pwm_L < thrust):
            pwm_L = thrust
        elif(pwm_L > 100):		#PWM
            pwm_L = 100		    #
        if(pwm_R < thrust):		#SATURATION
            pwm_R = thrust		#
        elif(pwm_R > 100):		#FILTER
            pwm_R = 100

        if(pwm_F < thrust):
            pwm_F = thrust
        elif(pwm_F > 100):		#PWM
            pwm_F = 100		    #
        if(pwm_B < thrust):		#SATURATION
            pwm_B = thrust		#
        elif(pwm_B > 100):		#FILTER
            pwm_B = 100

        #print("PWM_L:{:06.4f}, PWM_R:{:06.4f} ".format(pwm_L, pwm_R),end='')
        #print("PWM_F:{:06.4f}, PWM_B:{:06.4f}".format(pwm_F, pwm_B))

        #motor control pins PWM value assignments
        motor_L.ChangeDutyCycle(pwm_L)
        motor_R.ChangeDutyCycle(pwm_R)
        motor_F.ChangeDutyCycle(pwm_F)
        motor_B.ChangeDutyCycle(pwm_B)

        #print("elapsed:{0:.4f} Acc_x:{1:.2f} Gyr_T_x:{2:.2f} CF_x:{3:.2f} Acc_y:{4:.2f} Gyr_T_y:{5:.2f} CF_y:{6:.2f}".format(time.time()-prevtime,(rotation_x),(gyro_total_x),(CF_x),(rotation_y),(gyro_total_y),(CF_y)))
        #print("{0:.4f} {1:.2f} {2:.2f} {3:.2f} {4:.2f} {5:.2f} {6:.2f}".format(time.time()-initial,(rotation_x),(gyro_total_x),(CF_x),(rotation_y),(gyro_total_y),(CF_y)))
        print("{:.4f} {:.4f} {:.4f} {:.4f} {:.4f} {:.4f} {:.4f} {:.4f} {:.4f} {:.4f} {} {} {}".format(time.time()-initial,CF_x,CF_y,north_deg_comp,roll_error,pitch_error,yaw_error,roll_pid,pitch_pid,yaw_pid,ref[0],ref[1],ref[2]))

except KeyboardInterrupt:
    #exception routine for a clean exit
    motor_L.stop()
    motor_R.stop()
    motor_F.stop()
    motor_B.stop()
    fichero.close()
    time.sleep(1)
    GPIO.cleanup()
