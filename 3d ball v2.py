from vpython import *
import numpy as np

myShpere = sphere(radius=1, color=color.red, opacity=.25)

while True:

    for thisRadius in np.linspace(.1, 3, 1000):
        rate(250)
        myShpere.radius = thisRadius
    for thisRadius in np.linspace(3, .1, 1000):
        rate(250)
        myShpere.radius = thisRadius