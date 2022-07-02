from vpython import *

myShpere = sphere(radius=1, color=vector(1,0,0))
r = 1
g = 1
b = 0

r_inc = 0.001
g_inc = -0.001
b_inc = 0.001


while True:
    rate(200)
    r += r_inc
    g += g_inc
    b += b_inc

    if r <= 1:
        r_apply = r
    else:
        r_apply = 1

    if g <= 1:
        g_apply = g
    else:
        g_apply = 1

    if b <= 1:
        b_apply = b
    else:
        b_apply = 1

    myShpere.color = vector(r_apply, g_apply, b_apply)

    if r >= 1.5 or r <= 0:
        r_inc = r_inc * -1
    if g >= 1.5 or g <= 0:
        g_inc = g_inc*-1
    if b >= 1.5 or b <= 0:
        b_inc = b_inc * -1
    print(r_apply+g_apply+b_apply)

