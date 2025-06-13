import RPi.GPIO as GPIO
from time import sleep

LMOT_PIN = 12 #LB MOTOR FOR X-FRAME
RMOT_PIN = 13 #RF MOTOR FOR X-FRAME
FMOT_PIN = 22 #RB MOTOR FOR X-FRAME
BMOT_PIN = 27 #LF MOTOR FOR X-FRAME


duty = 25    #duty cycle of the PWM functions

GPIO.setmode(GPIO.BCM)    #set the GPIO pinout mode to BCM
#configure the outputs pins
GPIO.setup(LMOT_PIN,GPIO.OUT)
GPIO.setup(RMOT_PIN,GPIO.OUT)
GPIO.setup(FMOT_PIN,GPIO.OUT)
GPIO.setup(BMOT_PIN,GPIO.OUT)
#configure the PWM function on these pins with a frequency of 3kHz, getting really 2 kHz aprox.
ML=GPIO.PWM(LMOT_PIN,3000)
MR=GPIO.PWM(RMOT_PIN,3000)
MF=GPIO.PWM(FMOT_PIN,3000)
MB=GPIO.PWM(BMOT_PIN,3000)
#start the output on the pins with <duty> value
ML.start(duty)
MR.start(duty)
MF.start(duty)
MB.start(duty)
try:
    #while True:    #keep running until a key press
    sleep(5)
    ML.stop()
    MR.stop()
    MF.stop()
    MB.stop()
    sleep(1)
    GPIO.cleanup()

except KeyboardInterrupt:
    #exception routine for a clean exit
    ML.stop()
    MR.stop()
    MF.stop()
    MB.stop()
    sleep(1)
    GPIO.cleanup()
