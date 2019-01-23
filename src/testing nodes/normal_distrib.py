import numpy as np
import scipy.stats as stats
import pylab as pl

h = []

f1 = open("/home/mustafaismail/Documents/GP/catkin_ws/src/gps_road_estimation/src/error_data/gps_no_shift_1meter.txt", "r")
text1 = f1.read()
parsed_text1 = text1.split(',')

for num in parsed_text1:
	num = float(num) - 1.75
	h.append(num)

f2 = open("/home/mustafaismail/Documents/GP/catkin_ws/src/gps_road_estimation/src/error_data/gps_no_shift_2meters.txt", "r")
text2 = f2.read()
parsed_text2 = text2.split(',')

for num in parsed_text2:
	num = float(num) - 2.0
	h.append(num)

h = sorted(h)
# print h
print(np.mean(h))
print(np.std(h))
# fit = stats.norm.pdf(h, np.mean(h), np.std(h))  #this is a fitting indeed
# pl.plot(h, fit, '-o')
# pl.hist(h, normed=True)      #use this to draw histogram of your data
# pl.show()                   #use may also need add this 
