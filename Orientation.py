from vpython import *
import numpy as np
from math import *
import ahrs

scene.range = 5
# scene.axis = vector(1, -1, -1)
# scene.camera.axis = vector(1, 1, 1)
scene.forward = vector(.5, -1, -1)
scene.background = color.gray(.2)
arrowL = 4
arrowT = .05
ball_r = 3
point_ball_r = ball_r/30


mySphere = sphere(radius=ball_r, color=vector(1,1,1), opacity=.5)
xArrow = arrow(axis=vector(1, 0, 0), color=vector(1, 0, 0), length=arrowL, shaftwidth=arrowT)
yArrow = arrow(axis=vector(0, 1, 0), color=vector(0, 1, 0), length=arrowL, shaftwidth=arrowT)
zArrow = arrow(axis=vector(0, 0, 1), color=vector(0, 0, 1), length=arrowL, shaftwidth=arrowT)

frontArrow = arrow(axis=vector(1, 0, 0), color=vector(1, 0.5, 0.5), length=arrowL*2, shaftwidth=arrowT)
upArrow = arrow(axis=vector(0, 1, 0), color=vector(0.5, 1, 0.5), length=arrowL*1.2, shaftwidth=arrowT)
sideArrow = arrow(axis=vector(0, 0, 1), color=vector(0.5, 0.5, 1), length=arrowL*1.2, shaftwidth=arrowT)

f_ball_point = sphere(make_trail=True, trail_color=frontArrow.color, radius=point_ball_r,
                      color=frontArrow.color, pos=frontArrow.axis, opacity=.5)
u_ball_point = sphere(make_trail=True, trail_color=upArrow.color, radius=point_ball_r,
                      color=upArrow.color, pos=upArrow.axis, opacity=.5)
s_ball_point = sphere(make_trail=True, trail_color=sideArrow.color, radius=point_ball_r,
                      color=sideArrow.color, pos=sideArrow.axis, opacity=.5)


arduino_board = box(length=2, width=.2, heght=.1, opacity=.8, pos=vector(0, 0, 0))
chip = box(length=.5, width=.05, heght=.05, pos=vector(-.5, .1+.05, 0), color=color.blue)
arduino = compound([arduino_board, chip])


theta = 45
theta = radians(theta)
# while True:
#     # XY rotation
#     for theta in np.linspace(0, 360, 180):
#         rate(100)
#         theta = np.radians(theta)
#         x = arrowL * np.cos(theta)
#         y = arrowL * np.sin(theta)
#         z = 0
#         direction = vector(x, y, z)
#         pntArrow.axis = direction
#         f_ball_point.trail_color = vector(1, 0, 0)
#         f_ball_point.pos = direction
#
#     # XZ rotation
#     for theta in np.linspace(0, 450, 180):
#         rate(100)
#         theta = np.radians(theta)
#         x = arrowL * np.cos(theta)
#         y = 0
#         z = arrowL * np.sin(theta)
#         direction = vector(x, y, z)
#         pntArrow.axis = direction
#         f_ball_point.pos = direction
#         f_ball_point.trail_color = vector(0, 0, 1)
#     # ZY rotation
#     for theta in np.linspace(0, 360, 180):
#         rate(100)
#         theta = np.radians(theta)
#         x = 0
#         y = arrowL * np.sin(theta)
#         z =  arrowL * np.cos(theta)
#         direction = vector(x, y, z)
#         pntArrow.axis = direction
#         f_ball_point.pos = direction
#         f_ball_point.trail_color = vector(0, 1, 0)
#     # XZ rotation (back in place to restart)
#     for theta in np.linspace(90, 360, 180):
#         rate(100)
#         theta = np.radians(theta)
#         x = arrowL * np.cos(theta)
#         y = 0
#         z = arrowL * np.sin(theta)
#         direction = vector(x, y, z)
#         pntArrow.axis = direction
#         f_ball_point.pos = direction
#         f_ball_point.trail_color = vector(0, 0, 1)

while True:
    pitch = theta
    for yaw in np.arange(0, 2 * pi, 0.1):
        rate(50)
        k = vector(cos(yaw) * cos(pitch), sin(pitch), sin(yaw) * cos(pitch))

        y = vector(0, 1, 0)
        s = cross(k, y)
        v = cross(s, k)

        frontArrow.axis = k
        upArrow.axis = v
        sideArrow.axis = s
        arduino.axis = k
        arduino.up = v
        frontArrow.length = arrowL * 2
        upArrow.length = arrowL * 1.2
        sideArrow.length = arrowL * 1.2
        f_ball_point.pos = frontArrow.axis
        u_ball_point.pos = upArrow.axis
        s_ball_point.pos = sideArrow.axis
