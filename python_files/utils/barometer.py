# -*- coding: utf-8 -*-
#!/usr/bin/python

import smbus
import time
import math

#BMP280 Address
BMP_ADD = 0x76
#BMP280 Registers Address
STATUS = 0xF3
CONTROL = 0xF4
CONFIG = 0xF5
BMP_OUT = 0xF7
TEMP_OUT = 0xFA
#Values
MEASURING = 0x04
NORMAL_PWRMODE = 0x03
OSRS_T_1 = 0x01<<5    #temperature oversampling coef = 1
OSRS_T_2 = 0x02<<5    #temperature oversampling coef = 2
OSRS_T_4 = 0x03<<5
OSRS_T_8 = 0x04<<5
OSRS_T_16 = 0x05<<5
IIR_FILTER_OFF = 0x00
IIR_FILTER_2 = 0x01 <<2
IIR_FILTER_4 = 0x02 <<2
IIR_FILTER_8 = 0x03 <<2
IIR_FILTER_16 = 0x04 <<2
T_SB_0_5 = 0x00 <<5    #standby time of 0.5ms
T_SB_62_5 = 0x01 <<5    #standby time of 62.5ms
OSRS_P_4 = 0x03 <<2    #pressure oversampling coef = 4
OSRS_P_8 = 0x04 <<2    #pressure oversampling coef = 8
OSRS_P_16 = 0x05 <<2    #pressure oversampling coef = 16

MEASURE_WAIT_PERIOD = 0.0001

bus = smbus.SMBus(1)

def read_byte(address, adr):
    return bus.read_byte_data(address, adr)

def write_byte(address, adr, value):
    return bus.write_byte_data(address, adr, value)

def read_block(address, adr, size):
    return bus.read_i2c_block_data(address, adr, size)

def write_block(address, adr, values):
    bus.write_i2c_block_data(address, adr, values)

def twos_comp_24(val):
    if (val >= 0x800000):
        return -((16777215 - val) + 1)
    else:
        return val

def twos_comp(val):
    if (val >= 0x8000):
        return -((65535 - val) + 1)
    else:
        return val

def get_word(array, index, twos):
    val = array[index] + (array[index+1]<<8)
    if twos:
        return twos_comp(val)
    else:
        return val

def calculate_32b():
    #This code is a direct translation from the datasheet

    calibration = read_block(BMP_ADD, 0x88, 24)
    dig_T1 = get_word(calibration, 0, False)
    dig_T2 = get_word(calibration, 2, True)
    dig_T3 = get_word(calibration, 4, True)
    dig_P1 = get_word(calibration, 6, False)
    dig_P2 = get_word(calibration, 8, True)
    dig_P3 = get_word(calibration, 10, True)
    dig_P4 = get_word(calibration, 12, True)
    dig_P5 = get_word(calibration, 14, True)
    dig_P6 = get_word(calibration, 16, True)
    dig_P7 = get_word(calibration, 18, True)
    dig_P8 = get_word(calibration, 20, True)
    dig_P9 = get_word(calibration, 22, True)

#    print("digitsT:\n",dig_T1,"\n",dig_T2,"\n",dig_T3,"\n")
#    print("digitsP:\n",dig_P1,"\n",dig_P2,"\n",dig_P3,"\n",dig_P4,"\n",dig_P5,"\n",dig_P6,"\n",dig_P7,"\n",dig_P8,"\n",dig_P9,"\n")

    #Calculate temperature
    var1 = (((adc_T>>3) - (dig_T1<<1)) * dig_T2) >> 11
    var2 = (((((adc_T>>4) - dig_T1) * ((adc_T>>4) - dig_T1)) >> 12) * dig_T3) >> 14
    t_fine = var1 + var2
    t = (t_fine * 5 + 128) >> 8

#    t = 2660
#    t_fine = ((t << 8)-128)//5 #Forcing temperature because sensor is not working

    #Now calculate the pressure
    var1 = (t_fine>>1) - 64000
    var2 = (((var1>>2) * (var1>>2)) >> 11) * dig_P6
    var2 = var2 + ((var1*dig_P5)<<1)
    var2 = (var2>>2)+(dig_P4<<16)
    var1 = (((dig_P3 * (((var1>>2) * (var1>>2)) >> 13 )) >> 3) + ((dig_P2 * var1)>>1))>>18
    var1 = ((32768+var1)*dig_P1)>>15

    if var1==0:
        return 0

    p = ((1048576-adc_P)-(var2>>12))*3125

    if p < 0x80000000:
        p = (p << 1) // var1
    else:
        p = (p // var1) * 2

    var1 = ((dig_P9) * ((((p>>3) * (p>>3))>>13)))>>12
    var2 = (((p>>2)) * (dig_P8))>>13
    p = (p + ((var1 + var2 + dig_P7) >> 4))

    return(t/100,p/100)


def calculate_64b():
    #This code is a direct translation from the datasheet

    calibration = read_block(BMP_ADD, 0x88, 24)
    dig_T1 = get_word(calibration, 0, False)
    dig_T2 = get_word(calibration, 2, True)
    dig_T3 = get_word(calibration, 4, True)
    dig_P1 = get_word(calibration, 6, False)
    dig_P2 = get_word(calibration, 8, True)
    dig_P3 = get_word(calibration, 10, True)
    dig_P4 = get_word(calibration, 12, True)
    dig_P5 = get_word(calibration, 14, True)
    dig_P6 = get_word(calibration, 16, True)
    dig_P7 = get_word(calibration, 18, True)
    dig_P8 = get_word(calibration, 20, True)
    dig_P9 = get_word(calibration, 22, True)
#    print("digitsT:[",dig_T1,",",dig_T2,",",dig_T3)

    #Calculate temperature
    var1 = (((adc_T>>3) - (dig_T1<<1)) * dig_T2) >> 11
    var2 = (((((adc_T>>4) - dig_T1) * ((adc_T>>4) - dig_T1)) >> 12) * dig_T3) >> 14
    t_fine = var1 + var2
    t = (t_fine * 5 + 128) >> 8

#    t = 2600
#    t_fine = ((t << 8)-128)//5 #Forcing temperature because sensor is not working

    #Now calculate the pressure
    var1 = t_fine - 128000
    var2 = var1 * var1 * dig_P6
    var2 = var2 + ((var1*dig_P5)<<17)
    var2 = var2 + (dig_P4<<35)
    var1 = ((var1 * var1 * dig_P3)>>8) + ((var1 * dig_P2)>>12)
    var1 = (((1<<47)+var1)*dig_P1)>>33

    if var1==0:
        return 0

    p = (1048576-adc_P)
    p = (((p<<31) - var2)*3125)//var1

    var1 = ((dig_P9) * (p>>13) * (p>>13))>>25
    var2 = (dig_P8 * p)>>19
    p = ((p + var1 + var2)>>8) + (dig_P7<<4)

    return(t/100,p/25600)


def calculate_float():
    #This code is a direct translation from the datasheet

    calibration = read_block(BMP_ADD, 0x88, 24)
    dig_T1 = get_word(calibration, 0, False)
    dig_T2 = get_word(calibration, 2, True)
    dig_T3 = get_word(calibration, 4, True)
    dig_P1 = get_word(calibration, 6, False)
    dig_P2 = get_word(calibration, 8, True)
    dig_P3 = get_word(calibration, 10, True)
    dig_P4 = get_word(calibration, 12, True)
    dig_P5 = get_word(calibration, 14, True)
    dig_P6 = get_word(calibration, 16, True)
    dig_P7 = get_word(calibration, 18, True)
    dig_P8 = get_word(calibration, 20, True)
    dig_P9 = get_word(calibration, 22, True)
#    print("digitsT:[",dig_T1,",",dig_T2,",",dig_T3)

    #Calculate temperature
    var1 = (adc_T/16384 - dig_T1/1024) * dig_T2
    var2 = (adc_T/131072 - dig_T1/8192) * (adc_T/131072 - dig_T1/8192) * dig_T3
    t_fine = var1 + var2
    t = t_fine/5120

#    t = 26.00
#    t_fine = t*5120 #Forcing temperature because sensor is not working

    #Now calculate the pressure
    var1 = (t_fine/2) - 64000
    var2 = var1 * var1 * dig_P6 / 32768
    var2 = var2 + (var1*dig_P5*2)
    var2 = (var2/4) + (dig_P4*65536)
    var1 = (dig_P3 * var1 * var1 / 524288 + (dig_P2 * var1)) / 524288
    var1 = (1 + var1/32768) * dig_P1

    if var1==0:
        return 0

    p = 1048576 - adc_P
    p = (p - var2/4096) * 6250 / var1

    var1 = dig_P9 * p * p / 2147483648
    var2 = p * dig_P8 / 32768
    p = p + (var1 + var2 + dig_P7) / 16

    return(t,p/100)

def altitude(p):
    return 44330 * (1.0 - math.pow(p / sea_level_pressure, (1/5.255)))

sea_level_pressure = 1015.28
take_off_altitude = 0
tm_array = [0]
calc_dur_array = [0]
prev_time = time.time()

try:

    write_byte(BMP_ADD, 0xF4, OSRS_T_4 | OSRS_P_16 | NORMAL_PWRMODE)

    while True:
        cur_time = time.time()
        tm = cur_time - prev_time
        prev_time = cur_time
        tm_array.append(tm*1000)
        #Read raw temperature and pressure values from forced mode
#        write_byte(BMP_ADD, 0xF4, 0x5E)     # Tell the sensor to take a measure
    #    measure_dur = time.time()
#        while(read_byte(BMP_ADD,STATUS) == MEASURING):
#            time.sleep(MEASURE_WAIT_PERIOD)    # Wait for the conversion to take place
    #    measure_dur = time.time() - measure_dur
    #    print("MEASURE_DUR_MS:",measure_dur*1000)

        adc = read_block(BMP_ADD,BMP_OUT,6)
        adc_P = (adc[0]<<12) + (adc[1]<<4) + (adc[2]>>4)
        adc_T = (adc[3]<<12) + (adc[4]<<4) + (adc[5]>>4)

#        print(f"adc_T:[{adc[3]},{adc[4]},{adc[5]}]")
#        print("Raw_Pressure:",adc_P,"Raw_Temperature:",adc_T)

#        init_time = time.time()
        temperature, pressure = calculate_32b()

        if (take_off_altitude == 0):
            take_off_altitude = altitude(pressure)
        print(take_off_altitude)
#        finish_time = time.time()
#        calc_duration_ms = (finish_time - init_time)*1000
#        calc_dur_array.append(calc_duration_ms)
        altitude_inc = round(altitude(pressure) - take_off_altitude,2)
        print(f"T:{round(temperature,2)}, P:{round(pressure,2)}, Alt:{altitude_inc}") #Printing pressure in hPa/mbar
        time.sleep(0.001)

except KeyboardInterrupt:
    pass
#    for i in range(0,len(tm_array)+1):
#        print("tm:",tm_array[i]," calc_dur:",calc_dur_array[i])
