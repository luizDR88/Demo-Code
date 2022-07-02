from vpython import *
import numpy as np

Cyl1_length = 1
glassBulb = sphere(radius=1.25, color=color.white, opacity=.3)
glassCylinder = cylinder(radius=0.65, length=6, color=color.white, opacity=.3, pos=vector(1,0,0))
mercurybulb = sphere(radius=1, color=color.red, opacity=1)
mercuryBar = cylinder(radius=0.35, length=Cyl1_length, color=color.red, opacity=1)

Cyl2_length = 1
glassBulb2 = sphere(radius=1.25, color=color.white, opacity=.3, pos=vector(0,5,0))
glassCylinder2 = cylinder(radius=0.65, length=6, color=color.white, opacity=.3, pos=vector(1,5,0))
mercurybulb2 = sphere(radius=1, color=color.red, opacity=1, pos=vector(0,5,0))
mercuryBar2 = cylinder(radius=0.35, length=Cyl1_length, color=color.red, opacity=1, pos=vector(0,5,0))


for tick in np.linspace(1, 6, 10):
    box(size=vector(.1, .5, .1), pos=vector(tick, 0, 0.65), color=color.gray(.3))
    label(text=str(round(tick*10, 1)), pos=vector(tick, .5, 0.65), color=color.gray(.3), opacity=.5)

Cyl1_delta = .1
Cyl2_delta = .2
while True:
    rate(100)
    # for this_length in np.linspace(2, 5.5, 1000):
    #     rate(250)
    #     mercuryBar.length = this_length
    # for this_length in np.linspace(5.5, 2, 1000):
    #     rate(250)
    #     mercuryBar.length = this_length

    Cyl1_length += Cyl1_delta
    Cyl2_length += Cyl2_delta

    if Cyl1_length >= 6 or Cyl1_length <= 1:
        Cyl1_delta = Cyl1_delta * -1

    if Cyl2_length >= 6 or Cyl2_length <= 1:
        Cyl2_delta = Cyl2_delta * -1

    mercuryBar.length = Cyl1_length
    mercuryBar2.length = Cyl2_length