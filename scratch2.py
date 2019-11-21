
import numpy as np
import matplotlib.pyplot as plt
import math

try:
    import colored_traceback.always
except:
    pass

from PythonRobotics.PathPlanning.CubicSpline import cubic_spline_planner as planner


def calc_speed_profile(cx, cy, cyaw, target_speed):

    speed_profile = [target_speed] * len(cx)

    direction = 1.0

    # Set stop point
    for i in range(len(cx) - 1):
        dyaw = cyaw[i + 1] - cyaw[i]
        switch = math.pi / 4.0 <= dyaw < math.pi / 2.0

        if switch:
            direction *= -1

        if direction != 1.0:
            speed_profile[i] = - target_speed
        else:
            speed_profile[i] = target_speed

        if switch:
            speed_profile[i] = 0.0

    speed_profile[-1] = 0.0

    return speed_profile


# ax = [0.0, 6.0, 12.5, 5.0, 7.5, 3.0, -1.0]
# ay = [0.0, 0.0, 5.0, 6.5, 3.0, 5.0, -2.0]

ax = [0.0, 10.0]
ay = [0.0, 0.0]


cx, cy, cyaw, ck, s = planner.calc_spline_course(ax, ay, ds=0.1)

sp = planner.Spline2D(ax,ay)
x,y = sp.calc_position(1)
# sp.calc_yaw()

target_speed = 10.0 / 3.6

sp = calc_speed_profile(cx, cy, cyaw, target_speed)


plt.figure()
plt.plot(ax,ay)
plt.plot(cx,cy)
plt.axes().set_aspect('equal','datalim')
plt.grid(True)
plt.show()