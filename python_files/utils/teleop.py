# -*- coding: utf-8 -*-
#!/usr/bin/python

import sys
import termios
import tty
from select import select
import time
import threading

ref = [0,0]
ref_increments = {
    'a':(-5,0),
    'd':(5,0),
    'w':(0,5),
    's':(0,-5)
    }



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

settings = saveTerminalSettings()
prevtime = time.time()

def getKey_thread(id):
    while True:
        #Tm = time.time() - prevtime
        global ref
        teleop_key = getKey(settings,0.5)
        print("Thread {} - teleop_key: {}".format(id,teleop_key))
        #time.sleep(0.2)
        if teleop_key in ref_increments.keys():
            ref[0] += ref_increments[teleop_key][0]
            ref[1] += ref_increments[teleop_key][1]
        elif teleop_key == 'p':
            ref = (0,0)
        elif (teleop_key == 'g'):
            print("ES G!!")
            break
        elif (teleop_key == '\x03'):
            break

        #print(f"ref: {ref}")

try:

    initial = time.time()
    k_thread = threading.Thread(target=getKey_thread,args=(1,))
    k_thread.start()

    while True:
        Tm = time.time() - prevtime
##        teleop_key = getKey(settings,0.5)
        #print ("Time_main: ",time.time()-initial)
        #a= 3.24324234324234*453545.345345/(223.12323+456745.342435345)
        time.sleep(0.05)
##        print("Tm: {} teleop_key: {}".format(Tm,teleop_key))
##        if teleop_key in ref_increments.keys():
##            ref[0] += ref_increments[teleop_key][0]
##            ref[1] += ref_increments[teleop_key][1]
##        elif teleop_key == 'p':
##            ref = (0,0)
##        elif (teleop_key == 'g'):
##           print("ES G!!")
##            break
##        elif (teleop_key == '\x03'):
##            break

        print(f"ref: {ref}")


except Exception as e:
    print(e)
    k_thread.join()

finally:
    restoreTerminalSettings(settings)
    k_thread.join()
