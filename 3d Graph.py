from vpython import *
from time import *
import random



# ball = sphere(color=color.cyan)
# sleep(5)
# ball.color = color.green

# myBox = box(color=color.magenta, length=12, width=1, height=.2)

# myTube = cylinder(color=color.magenta, length=12, radius=1)
# myTube = cylinder(color=color.magenta, length=12, width=1, height=.5)

room_size = 10
wall_thickness = room_size/100
wall_delta = room_size/2
opacity = .25


floor = box(color=color.gray(.5), opacity=opacity, size=vector(room_size, wall_thickness, room_size),
            pos=vector(0, -wall_delta, 0))
ceiling = box(color=color.gray(.5), opacity=opacity, size=vector(room_size, wall_thickness, room_size),
              pos=vector(0, wall_delta, 0))
front_wall = box(color=color.gray(.5), opacity=opacity*0.5, length=room_size, height=room_size, width=room_size/100,
                 pos=vector(0, 0, wall_delta))
back_wall = box(color=color.gray(.5), opacity=opacity, length=room_size, height=room_size, width=room_size/100,
                pos=vector(0, 0, -wall_delta))
wall_left = box(color=color.gray(.5), opacity=opacity, size=vector(wall_thickness, room_size, room_size),
                pos=vector(-wall_delta, 0, 0))
wall_right = box(color=color.gray(.5), opacity=opacity, size=vector(wall_thickness, room_size, room_size),
                 pos=vector(wall_delta, 0, 0))

initial_ball_radius = 1
ball = sphere(color=color.cyan, radius=initial_ball_radius)

deltaX = .2
deltaY = .2
deltaZ = .2
# deltaX = random.uniform(0, 1)
# deltaY = random.uniform(0, 1)
# deltaZ = random.uniform(0, 1)

xPos = 0
yPos = 1
zPos = 0
# print(random.randint(0,9))

run = 0
def runRadio(x):
    global run
    print(x.checked)
    if x.checked:
        run = 1
    else:
        run = 0


radio(bind=runRadio, text='Run')
scene.append_to_caption('\n\n')

def changeColor(vector):
    ball.color = vector

button(bind=lambda: changeColor(vector(1,0,0)), text='Red', color=color.black, background=vector(1,0,0))
scene.append_to_caption('  ')
button(bind=lambda: changeColor(vector(0,1,0)), text='Green', color=color.black, background=vector(0,1,0))
scene.append_to_caption('  ')
button(bind=lambda: changeColor(vector(0,0,1)), text='Blue', color=color.black, background=vector(0,0,1))
scene.append_to_caption('  ')
button(bind=lambda: changeColor(vector(1,1,1)), text='White', color=color.black, background=vector(1,1,1))
scene.append_to_caption('  ')
button(bind=lambda: changeColor(color.cyan), text='Original', color=color.black, background=color.cyan)

scene.append_to_caption('\n\n')

ball_radius = initial_ball_radius


def ballSize(x):
    global ball_radius
    if x.checked:
        ball_radius = initial_ball_radius * 2
    else:
        ball_radius = initial_ball_radius


checkbox(bind=ballSize, text='Big Ball')

scene.append_to_caption('\n\n')

run_speed = 20
def changeSpeed(x):
    global run_speed
    run_speed = x.value

wtext(text='Change ball Speed')
scene.append_to_caption('\n')
slider(bind=changeSpeed, vertical=False, min=5, max=100, value=20)

scene.append_to_caption('\n\n')


def changeOpacity(x):
    # ball.opacity = x.value
    floor.opacity = x.value
    ceiling.opacity = x.value
    front_wall.opacity = x.value
    back_wall.opacity = x.value
    wall_left.opacity = x.value
    wall_right.opacity = x.value


wtext(text='Change wall opacity')
scene.append_to_caption('\n')
slider(bind=changeOpacity, vertical=False, min=0, max=1, value=.5)

scene.append_to_caption('\n\n')

# menu(bind=boxShape, choices=['1', '2', '3', '4'])


while True:
    ball.radius = ball_radius
    rate(run_speed)
    xPos += deltaX * run
    yPos += deltaY * run
    zPos += deltaZ * run


    ball_edge_left = xPos-ball_radius
    ball_edge_right = xPos+ball_radius
    inner_left_wall_pos = -wall_delta+wall_thickness/2
    inner_right_wall_pos = wall_delta-wall_thickness/2
    if ball_edge_right >= inner_right_wall_pos or ball_edge_left <= inner_left_wall_pos:
        deltaX = deltaX * -1

    ball_edge_top = yPos + ball_radius
    ball_edge_bottom = yPos - ball_radius
    inner_ceiling_wall_pos = wall_delta - wall_thickness / 2
    inner_floor_pos = -wall_delta + wall_thickness / 2
    if ball_edge_top >= inner_ceiling_wall_pos or ball_edge_bottom <= inner_floor_pos:
        deltaY = deltaY * -1

    ball_edge_back = zPos + ball_radius
    ball_edge_front = zPos - ball_radius
    inner_back_wall_pos = wall_delta - wall_thickness / 2
    inner_invisible_front_pos = -wall_delta + wall_thickness / 2
    if ball_edge_back >= inner_back_wall_pos or ball_edge_front <= inner_invisible_front_pos:
        deltaZ = deltaZ * -1

    ball.pos = vector(xPos, yPos, zPos)


