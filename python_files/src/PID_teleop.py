# -*- coding: utf-8 -*-
#!/usr/bin/python

import smbus
import math
import time
import RPi.GPIO as GPIO

#teleop imports
import sys
import termios
import tty
from select import select
import threading


LMOT_PIN = 12
RMOT_PIN = 13
FMOT_PIN = 22
BMOT_PIN = 27

#MPU9250 Address
MPU_ADD = 0x68
#MPU9250 Registers Addresses
PWR_MGMT_1 = 0x6B
GYRO_XOUT_H = 0x43
ACCEL_XOUT_H = 0x3B
ACCEL_CONFIG2 = 0x1D
INT_PIN_CFG = 0x37
#Values
A_FCHOICE_B = (0x00 << 2)
A_DLPF_1 = 0x01    #218.1 Hz   ACCELEROMETER
A_DLPF_2 = 0x02    #99 Hz      DIGITAL
A_DLPF_3 = 0x03    #44.8 Hz    LOW
A_DLPF_4 = 0x04    #21.2 Hz    PASS
A_DLPF_5 = 0x05    #10.2 Hz    FILTER
A_DLPF_6 = 0x06    #5.05 Hz    CONFIGURATION

gyro_scale = 131.0    #scaling parameter for gyroscope readings
accel_scale = 16384.0    #scaling parameter for accelerometer readings

Kp = 1.9 #1.5    #Proportional PID constant
Ki = 0.6 #0.08    #Integral PID constant
Kd = 0.48 #0.5    #Derivative PID constant
roll_SumaEr = 0    #Error's accumulation (integral)
pitch_SumaEr = 0    #Error's accumulation (integral)
thrust = 50    #Base propulsion level of the motors
ref = [5.5,-0.5]    #reference angle values

GPIO.setmode(GPIO.BCM)
GPIO.setup(LMOT_PIN,GPIO.OUT,initial=GPIO.LOW)
GPIO.setup(RMOT_PIN,GPIO.OUT,initial=GPIO.LOW)
GPIO.setup(FMOT_PIN,GPIO.OUT,initial=GPIO.LOW)
GPIO.setup(BMOT_PIN,GPIO.OUT,initial=GPIO.LOW)
motor_L = GPIO.PWM(LMOT_PIN,3000)
motor_R = GPIO.PWM(RMOT_PIN,3000)
motor_F = GPIO.PWM(FMOT_PIN,3000)
motor_B = GPIO.PWM(BMOT_PIN,3000)
motor_L.start(0)
motor_R.start(0)
motor_F.start(0)
motor_B.start(0)

bus = smbus.SMBus(1)

## teleop variables
ref_increments = {
    'a':(1,0),
    'd':(-1,0),
    'w':(0,-1),
    's':(0,1)
    }

def read_byte(address, adr):
    return bus.read_byte_data(address, adr)

def write_byte(address, adr, value):
    return bus.write_byte_data(address, adr, value)

def read_block(address, adr, size):
    return bus.read_i2c_block_data(address,adr,size)

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

def get_y_rotation(x,y,z):
    radians = math.atan2(x, dist(y,z))
    return -math.degrees(radians)

def get_x_rotation(x,y,z):
    radians = math.atan2(y, dist(x,z))
    return math.degrees(radians)

def set_normal_mode():
    write_byte(MPU_ADD, PWR_MGMT_1, 0)

def set_accel_DLPF(val):
    write_byte(MPU_ADD, ACCEL_CONFIG2, val)

## --- teleop functions --- ##
def saveTerminalSettings():
    return termios.tcgetattr(sys.stdin)

def restoreTerminalSettings(old_settings):
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)

def getKey(settings, timeout):
    tty.setraw(sys.stdin.fileno())
    # sys.stdin.read() returns a string on Linux
    rlist = select([sys.stdin], [], [], timeout)[0]
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def getKey_thread(id):
    global ref, shutdown_motors
    while True:
        #Tm = time.time() - prevtime
        #print(f"pre_getKey ref: {ref}")
        teleop_key = getKey(settings,1)
        print("Teleop thread - key: {}".format(teleop_key))
        #time.sleep(0.1)
        if teleop_key in ref_increments.keys():
            ref[0] += ref_increments[teleop_key][0]
            ref[1] += ref_increments[teleop_key][1]
        elif teleop_key == 'p':
            ref = (0,0)
        elif (teleop_key == ''):
            print("Resetted key!")
            continue
        elif teleop_key == 'f':
            shutdown_motors = True
            break
        elif teleop_key == '\x03':
            shutdown_motors = True
            break

        print(f"post_getKey ref: {ref}")

##############################

#Now wake the MPU9250 up as it starts in sleep mode
set_normal_mode()
#And set the DLPF filtering frequency (BW)
set_accel_DLPF(A_DLPF_4)

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

#print ("{0:.4f} {1:.2f} {2:.2f} {3:.2f} {4:.2f} {5:.2f} {6:.2f}".format( time.time() - initial, (CF_x), gyro_total_x, (CF_x), (CF_y), gyro_total_y, (CF_y)))

shutdown_motors = False
settings = saveTerminalSettings()
teleop_thread = threading.Thread(target=getKey_thread,args=(1,))
teleop_thread.start()

try:
    while True:    #keep running until a key press
        Tm = (time.time() - prevtime)    #Sample time variable measures time between loop code runs
        prevtime = time.time()        #this variable keeps current time as previous one for the next Tm assignment
        if (shutdown_motors == True):
            print("ABORT!!")
            motor_L.stop()
            motor_R.stop()
            motor_F.stop()
            motor_B.stop()
            time.sleep(1)
            GPIO.cleanup()
            break

        #set reference step
#        if (prevtime - initial > 10 and prevtime - initial < 11):
#            ref[1] = -20
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
        roll_error=CF_x-ref[0]
        CF_y = 0.98 * (CF_y + gyro_y_delta) + (0.02 * rotation_y)
        pitch_error=CF_y-ref[1]
        #print("Roll_Err:{:+08.4f}, Pitch_Err:{:+08.4f}  ".format(roll_error, pitch_error),end='')

        roll_IncEr = roll_error - prev_roll_error    #Error increment
        roll_SumaEr += roll_error                    #Error accumulation (integral)
        if (prev_roll_error * roll_error < 0) :      #Anti-windup mechanism
            roll_SumaEr = 0

        prev_roll_error = roll_error                 #Previous roll error update

        pitch_IncEr = pitch_error - prev_pitch_error    #Error increment
        pitch_SumaEr += pitch_error                     #Error accumulation (integral)
        if (prev_pitch_error * pitch_error < 0) :       #Anti-windup mechanism
            pitch_SumaEr = 0

        prev_pitch_error = pitch_error                  #Previous pitch error update

        #PID controllers calculation (parallel)
        roll_pid = Kp*roll_error + Kd*roll_IncEr/Tm + Ki*roll_SumaEr*Tm
        pitch_pid = Kp*pitch_error + Kd*pitch_IncEr/Tm + Ki*pitch_SumaEr*Tm
        #print("Roll_Pid:{:+08.4f}, Pitch_Pid:{:+08.4f}  ".format(roll_pid, pitch_pid),end='')

#        if(roll_pid>100-thrust):	    #PID
#            roll_pid=100-thrust		#SATURATION
#        elif(roll_pid<-(100-thrust)):	#FILTER
#            roll_pid=-(100-thrust)

#        if(pitch_pid>100-thrust):	    #PID
#            pitch_pid=100-thrust	    #SATURATION
#        elif(pitch_pid<-(100-thrust)):	#FILTER
#            pitch_pid=-(100-thrust)

        pwm_L = thrust + roll_pid	    #PWM motorL value (left)
        pwm_R = thrust - roll_pid	    #PWM motorR value (right)
        pwm_F = thrust - pitch_pid      #PWM motorF value (front)
        pwm_B = thrust + pitch_pid      #PWM motorB value (back)

        if(pwm_L < 5):
            pwm_L = 5
        elif(pwm_L > 100):      #PWM
            pwm_L = 100	        #
        if(pwm_R < 5):	        #SATURATION
            pwm_R = 5		    #
        elif(pwm_R > 100):		#FILTER
            pwm_R = 100

        if(pwm_F < 5):
            pwm_F = 5
        elif(pwm_F > 100):		#PWM
            pwm_F = 100	        #
        if(pwm_B < 5):		    #SATURATION
            pwm_B = 5		    #
        elif(pwm_B > 100):		#FILTER
            pwm_B = 100

        #print("PWM_L:{:06.4f}, PWM_R:{:06.4f} ".format(pwm_L, pwm_R),end='')
        #print("PWM_F:{:06.4f}, PWM_B:{:06.4f}".format(pwm_F, pwm_B))

        #motor control pins PWM value assignments
        motor_L.ChangeDutyCycle(pwm_L)
        motor_R.ChangeDutyCycle(pwm_R)
        motor_F.ChangeDutyCycle(pwm_F)
        motor_B.ChangeDutyCycle(pwm_B)

        #print ("elapsed:{0:.4f} Acc_x:{1:.2f} Gyr_T_x:{2:.2f} CF_x:{3:.2f} Acc_y:{4:.2f} Gyr_T_y:{5:.2f} CF_y:{6:.2f}".format(time.time()-prevtime,(rotation_x),(gyro_total_x),(CF_x),(rotation_y),(gyro_total_y),(CF_y)))
        #print ("{0:.4f} {1:.2f} {2:.2f} {3:.2f} {4:.2f} {5:.2f} {6:.2f}".format(time.time()-initial,(rotation_x),(gyro_total_x),(CF_x),(rotation_y),(gyro_total_y),(CF_y)))
        #print ("{:.4f} {:.4f} {:.4f} {:.4f} {:.4f} {:.4f} {:.4f} {} {}".format(time.time()-initial, CF_x, CF_y, roll_error, roll_pid, pitch_error, pitch_pid, ref[0], ref[1]))

except KeyboardInterrupt:
    #exception routine for a clean exit
    motor_L.stop()
    motor_R.stop()
    motor_F.stop()
    motor_B.stop()
    time.sleep(1)
    GPIO.cleanup()

finally:
    restoreTerminalSettings(settings)
    teleop_thread.join()
