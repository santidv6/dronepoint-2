import pigpio as pi
import numpy as np
import time

resolution = 3600

init_time = time.time()

angles = np.linspace(0, 2*np.pi, resolution)
sin_lut = np.sin(angles)
cos_lut= np.cos(angles)

post_calc_time = time.time()

print("Calculation time was: ", post_calc_time - init_time)
