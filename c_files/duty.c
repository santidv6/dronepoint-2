#include <stdio.h>
#include <pigpio.h>
#include <stdlib.h>
#include <signal.h>

#define LFMOTOR_PIN 27
#define RFMOTOR_PIN 13
#define LBMOTOR_PIN 12
#define RBMOTOR_PIN 22

#define PWM_FREQ 2500
#define PWM_RANGE 100

void exit_cleanup(int signum){
    gpioPWM(LFMOTOR_PIN, 0);
    gpioPWM(RFMOTOR_PIN, 0);
    gpioPWM(LBMOTOR_PIN, 0);
    gpioPWM(RBMOTOR_PIN, 0);
    gpioDelay(10000);
    gpioTerminate();
//    printf("Terminated!\n");
    exit(signum);
}

int main (int argc, char **argv){

    int duty = 50;
    int rRange, rFreq;

    gpioCfgClock(4,1,0); // 4us sample rate, PCM and ignored value
    if (gpioInitialise() < 0)
        return 1;

    signal(SIGINT, exit_cleanup); //This must be placed after gpioInitialise()!

    rFreq = gpioSetPWMfrequency(LFMOTOR_PIN, PWM_FREQ);
    gpioSetPWMfrequency(RFMOTOR_PIN, PWM_FREQ);
    gpioSetPWMfrequency(LBMOTOR_PIN, PWM_FREQ);
    gpioSetPWMfrequency(RBMOTOR_PIN, PWM_FREQ);

    rRange = gpioSetPWMrange(LFMOTOR_PIN, PWM_RANGE);
    gpioSetPWMrange(RFMOTOR_PIN, PWM_RANGE);
    gpioSetPWMrange(LBMOTOR_PIN, PWM_RANGE);
    gpioSetPWMrange(RBMOTOR_PIN, PWM_RANGE);

    gpioPWM(LFMOTOR_PIN, duty-8);
    gpioPWM(RFMOTOR_PIN, duty+8);
    gpioPWM(LBMOTOR_PIN, duty+8);
    gpioPWM(RBMOTOR_PIN, duty-8);

//    printf("GPIO %d is at %d%% duty cycle with a freq of %d and range of %d \n", RBMOTOR_PIN, duty, rFreq, rRange);
    gpioDelay(3000000);
    exit_cleanup(0);
    return 0;
}
