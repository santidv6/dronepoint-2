import pigpio
from time import sleep

pi = pigpio.pi()

if not pi.connected:
    exit()

LMOT_PIN = 12 #LB MOTOR FOR X-FRAME
RMOT_PIN = 13 #RF MOTOR FOR X-FRAME
FMOT_PIN = 22 #RB MOTOR FOR X-FRAME
BMOT_PIN = 27 #LF MOTOR FOR X-FRAME

duty = 24    #duty cycle of the PWM functions (resolution of 2% with pigpiod 10us sample rate and 2kHz PWM or 5us sample rate and 4kHz PWM)

#configure the PWM function on these pins with a frequency of 2 kHz and a range of 100 (really 50 steps, executing daemon with 10 us)
pi.set_PWM_frequency(LMOT_PIN,2000)
pi.set_PWM_frequency(RMOT_PIN,2000)
pi.set_PWM_frequency(FMOT_PIN,2000)
pi.set_PWM_frequency(BMOT_PIN,2000)
pi.set_PWM_range(LMOT_PIN,100)
pi.set_PWM_range(RMOT_PIN,100)
pi.set_PWM_range(FMOT_PIN,100)
pi.set_PWM_range(BMOT_PIN,100)

#start the output on the pins with <duty> value
pi.set_PWM_dutycycle(LMOT_PIN,duty+6)
pi.set_PWM_dutycycle(RMOT_PIN,duty+6)
#pi.hardware_PWM(LMOT_PIN,10000,(duty+6)*10000);
#pi.hardware_PWM(RMOT_PIN,10000,(duty+6)*10000);

pi.set_PWM_dutycycle(FMOT_PIN,duty-6)
pi.set_PWM_dutycycle(BMOT_PIN,duty-6)

try:
    #while True:    #keep running until a key press
    sleep(3)
    pi.set_PWM_dutycycle(LMOT_PIN,0)
    pi.set_PWM_dutycycle(RMOT_PIN,0)
#    pi.hardware_PWM(LMOT_PIN,0,0);
#    pi.hardware_PWM(RMOT_PIN,0,0);

    pi.set_PWM_dutycycle(FMOT_PIN,0)
    pi.set_PWM_dutycycle(BMOT_PIN,0)
    pi.stop()
    sleep(1)

except KeyboardInterrupt:
    #exception routine for a clean exit
    pi.set_PWM_dutycycle(LMOT_PIN,0)
    pi.set_PWM_dutycycle(RMOT_PIN,0)
#    pi.hardware_PWM(LMOT_PIN,0,0);
#    pi.hardware_PWM(RMOT_PIN,0,0);

    pi.set_PWM_dutycycle(FMOT_PIN,0)
    pi.set_PWM_dutycycle(BMOT_PIN,0)
    pi.stop()
    sleep(1)
