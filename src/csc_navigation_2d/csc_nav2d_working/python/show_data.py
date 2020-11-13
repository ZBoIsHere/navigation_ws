import numpy as np
import matplotlib.pyplot as plt
import math
dir_list = []#direction
control_z = []
control_x = []#velocity
vel_left_wheel = []
vel_right_wheel = []


mCurrentDirection=-1.0
step=0.05;

while mCurrentDirection<= 1.0:
    dir_list.append(mCurrentDirection)
    x = math.sin( mCurrentDirection*math.pi )
    y = (math.cos(mCurrentDirection*math.pi) +1 )
    r = (x*x + y*y)/(2*x)
    abs_r = math.sqrt(r*r)
    velocity = 0.8  # 0.5m/s
    velocity = velocity/(1+(1.0/abs_r))
    control_z.append(-1.0/r*velocity)
    control_x.append(velocity)
    mCurrentDirection = mCurrentDirection + step

    vel_left_wheel.append(control_x[-1] - control_z[-1]/2.0)
    vel_right_wheel.append(control_x[-1] + control_z[-1] / 2.0)




plt.subplot(3,1,1)
plt.plot(dir_list, control_z, '*')

plt.xlabel('mCurrentDirection')
plt.ylabel('control_z')
plt.grid()


plt.subplot(3,1,2)
plt.plot(dir_list, control_x, '*')
plt.xlabel('mCurrentDirection')
plt.ylabel('control_x')
plt.grid()

plt.subplot(3,1,3)
plt.plot(dir_list, vel_left_wheel, 'r*')
plt.plot(dir_list, vel_right_wheel, 'g*')
plt.xlabel('mCurrentDirection')
plt.ylabel('wheelspeed,red is left .green is right.')
plt.grid()

plt.show()
