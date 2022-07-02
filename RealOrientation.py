import pandas as pd
from math import *
import numpy as np
import os
import matplotlib
# matplotlib.use('GTKAgg')
from matplotlib import pyplot as plt
from matplotlib.animation import FuncAnimation
import matplotlib.animation as animation

import time
# from time import *

from vpython import *
import ahrs

folder_path = os.getcwd()
files= ['ball_IMU_tabela_final_fdt6_cropped.csv', 'ball_IMU_tabela_final_fdt9.csv']
selected_file = 1
file_path = os.path.join(folder_path, 'files', files[selected_file])
ball_IMU = pd.read_csv(file_path)

ball_IMU.head()

data_acc = ball_IMU[['elapsed_time', 'ax', 'ay','az']]
data_gyr = ball_IMU[['elapsed_time', 'gx', 'gy','gz']]
data_mag = ball_IMU[['elapsed_time', 'mx', 'my','mz']]

plt.style.use('ggplot')  # matplotlib visual style setting

# time.sleep(1) # wait for mpu9250 sensor to settle

l_second = 0 # in seconds
u_second = 120 # in seconds

l_bound = ball_IMU.iloc[(ball_IMU['elapsed_time']-l_second).abs().argsort()[:1]].index.values[0]
u_bound = ball_IMU.iloc[(ball_IMU['elapsed_time']-u_second).abs().argsort()[:1]].index.values[0]


ii = len(ball_IMU['elapsed_time'])  # number of points
ii = u_bound
t1 = time.time() * 0 + l_bound  # for calculating sample rate

# prepping for visualization
mpu6050_str = ['accel-x', 'accel-y', 'accel-z', 'gyro-x', 'gyro-y', 'gyro-z']
AK8963_str = ['mag-x', 'mag-y', 'mag-z']
mpu6050_vec, AK8963_vec, t_vec = [], [], []

print(f'recording data from row {l_bound} to row {u_bound}')
for ii in range(l_bound, ii):

    try:
        ax, ay, az, wx, wy, wz = ball_IMU['ax'][ii], ball_IMU['ay'][ii], ball_IMU['az'][ii], ball_IMU['gx'][ii], \
                                 ball_IMU['gy'][ii], ball_IMU['gz'][ii]
        mx, my, mz = ball_IMU['mx'][ii], ball_IMU['my'][ii], ball_IMU['mz'][ii]
    except:
        continue
    t_vec.append(ball_IMU['elapsed_time'][ii])  # capture timestamp
    AK8963_vec.append([mx, my, mz])
    mpu6050_vec.append([ax, ay, az, wx, wy, wz])

print('sample rate accel: {} Hz'.format(ii / ((time.time() * 0 + u_bound) - t1)))  # print the sample rate
# t_vec = np.subtract(t_vec,t_vec[0])
# print(f'time {t_vec}')

# plot the resulting data in 3-subplots, with each data axis
fig, axs = plt.subplots(3, 1, figsize=(12, 7), sharex=True)
cmap = plt.cm.Set1

ax = axs[0]  # plot accelerometer data
for zz in range(0, np.shape(mpu6050_vec)[1] - 3):
    data_vec = [ii[zz] for ii in mpu6050_vec]
    ax.plot(t_vec, data_vec, label=mpu6050_str[zz], color=cmap(zz))
ax.legend(bbox_to_anchor=(1.12, 0.9))
ax.set_ylabel('Acceleration [g]', fontsize=12)

ax2 = axs[1]  # plot gyroscope data
for zz in range(3, np.shape(mpu6050_vec)[1]):
    data_vec = [ii[zz] for ii in mpu6050_vec]
    ax2.plot(t_vec, data_vec, label=mpu6050_str[zz], color=cmap(zz))
ax2.legend(bbox_to_anchor=(1.12, 0.9))
ax2.set_ylabel('Angular Vel. [dps]', fontsize=12)

ax3 = axs[2]  # plot magnetometer data
for zz in range(0, np.shape(AK8963_vec)[1]):
    data_vec = [ii[zz] for ii in AK8963_vec]
    ax3.plot(t_vec, data_vec, label=AK8963_str[zz], color=cmap(zz + 6))
ax3.legend(bbox_to_anchor=(1.12, 0.9))
ax3.set_ylabel('Magn. Field [Î¼T]', fontsize=12)
ax3.set_xlabel('Time [s]', fontsize=14)

fig.align_ylabels(axs)
plt.show()

# https://toptechboy.com/9-axis-imu-lesson-19-vpython-visualization-of-pitch-and-yaw/

acc = data_acc
gyr = data_gyr
mag = data_mag

thetaFold = 0.0
phiFold = 0.0
thetaG = 0.0
phiG = 0.0
millisOld = 0

thetaM = np.zeros(len(acc)).astype(float)
phiM = np.zeros(len(acc)).astype(float)

theta = np.zeros(len(acc)).astype(float)
phi = np.zeros(len(acc)).astype(float)
psi = np.zeros(len(acc)).astype(float)

for i in range(len(acc)):
    thetaM[i] = -atan2(acc['ax'][i] / 9.8, acc['az'][i] / 9.8) / 2 / pi * 360
    phiM[i] = -atan2(acc['ay'][i] / 9.8, acc['az'][i] / 9.8) / 2 / pi * 360

    phiFnew = .95 * phiFold + .05 * phiM[i]

    thetaFnew = .95 * thetaFold + .05 * thetaM[i]

    millis = acc['elapsed_time'][i]
    dt = (millis - millisOld) / 1000.
    millisOld = millis
    theta[i] = (theta[i] + gyr['gy'][i] * dt) * .95 + thetaM[i] * .05
    phi[i] = (phi[i] - gyr['gx'][i] * dt) * .95 + phiM[i] * .05
    thetaG = thetaG + gyr['gy'][i] * dt
    phiG = phiG - gyr['gx'][i] * dt
    phiRad = radians(phi[i])
    thetaRad = radians(theta[i])

    Xm = mag['mx'][i] * cos(thetaRad) - mag['my'][i] * sin(phiRad) * sin(thetaRad) + mag['mz'][i] * cos(phiRad) * sin(
        thetaRad)
    Ym = mag['my'][i] * cos(phiRad) + mag['mz'][i] * sin(phiRad)

    psi[i] = atan2(Ym, Xm) / (2 * 3.14) * 360

    #     print(acc['ax'][i]/9.8)
    #     print(acc['ay'][i]/9.8)
    #     print(acc['az'][i]/9.8)
    #     print(thetaM)
    #     print(phiM)
    #     print(thetaFnew)
    #     print(phiFnew)
    #     print(thetaG)
    #     print(phiG)
    #     print(theta[i])
    #     print(phi[i])
    #     print(psi[i])

    phiFold = phiFnew
    thetaFold = thetaFnew

# myIMU.getCalibration(&system, &gyro, &accel, &mg);
# imu::Vector<3> acc =myIMU.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
# imu::Vector<3> gyr =myIMU.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
# imu::Vector<3> mag =myIMU.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);
# thetaM=-atan2(acc.x()/9.8,acc.z()/9.8)/2/3.141592654*360;
# phiM=-atan2(acc.y()/9.8,acc.z()/9.8)/2/3.141592654*360;
# phiFnew=.95*phiFold+.05*phiM;
# thetaFnew=.95*thetaFold+.05*thetaM;

# dt=(millis()-millisOld)/1000.;
# millisOld=millis();
# theta=(theta+gyr.y()*dt)*.95+thetaM*.05;
# phi=(phi-gyr.x()*dt)*.95+ phiM*.05;
# thetaG=thetaG+gyr.y()*dt;
# phiG=phiG-gyr.x()*dt;

# phiRad=phi/360*(2*3.14);
# thetaRad=theta/360*(2*3.14);

# Xm=mag.x()*cos(thetaRad)-mag.y()*sin(phiRad)*sin(thetaRad)+mag.z()*cos(phiRad)*sin(thetaRad);
# Ym=mag.y()*cos(phiRad)+mag.z()*sin(phiRad);

# psi=atan2(Ym,Xm)/(2*3.14)*360;


print(theta, '\n', phi, '\n', psi)

angles = np.array([theta, phi, psi])
angles = np.transpose(angles)
print(angles)
euler_df = pd.DataFrame(data=angles, columns=['theta', 'phi', 'psi'])

# https://www.meccanismocomplesso.org/en/hamiltons-quaternions-and-3d-rotation-with-python/
def q_conjugate(q):
    w, x, y, z = q
    return (w, -x, -y, -z)


def qv_mult(q1, v1):
    q2 = (0.0,) + v1
    return q_mult(q_mult(q1, q2), q_conjugate(q1))[1:]


def q_mult(q1, q2):
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
    x = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
    y = w1 * y2 + y1 * w2 + z1 * x2 - x1 * z2
    z = w1 * z2 + z1 * w2 + x1 * y2 - y1 * x2
    return w, x, y, z


def euler_to_quaternion(phi, theta, psi):
    qw = cos(phi / 2) * cos(theta / 2) * cos(psi / 2) + sin(phi / 2) * sin(theta / 2) * sin(psi / 2)
    qx = sin(phi / 2) * cos(theta / 2) * cos(psi / 2) - cos(phi / 2) * sin(theta / 2) * sin(psi / 2)
    qy = cos(phi / 2) * sin(theta / 2) * cos(psi / 2) + sin(phi / 2) * cos(theta / 2) * sin(psi / 2)
    qz = cos(phi / 2) * cos(theta / 2) * sin(psi / 2) - sin(phi / 2) * sin(theta / 2) * cos(psi / 2)

    return [qw, qx, qy, qz]

def quaternion_to_euler(w, x, y, z):
    t0 = 2 * (w * x + y * z)
    t1 = 1 - 2 * (x * x + y * y)
    X = atan2(t0, t1)

    t2 = 2 * (w * y - z * x)
    t2 = 1 if t2 > 1 else t2
    t2 = -1 if t2 < -1 else t2
    Y = asin(t2)

    t3 = 2 * (w * z + x * y)
    t4 = 1 - 2 * (y * y + z * z)
    Z = atan2(t3, t4)

    return X, Y, Z

v1 = (1,0,0)
phi_ex = pi/2
theta_ex = pi/4
psi_ex = pi/2
q = euler_to_quaternion(phi_ex, theta_ex, psi_ex)
print("w =", q[0])
print("x =", q[1])
print("y =", q[2])
print("z =", q[3])

v2 = qv_mult(q,v1)
print(np.round(v2, decimals=2))

v3 = quaternion_to_euler(q[0], q[1], q[2], q[3])
v3_deg_phi = degrees(v3[0])
v3_deg_theta = degrees(v3[1])
v3_deg_psi = degrees(v3[2])

print(np.round(v3, decimals=2))
print(f'Original angles: {round(degrees(phi_ex), 2)}, {round(degrees(theta_ex),2)}, {round(degrees(psi_ex),2)}')
print(f'Reconverted angles: {round(v3_deg_phi, 2)}, {round(v3_deg_theta,2)}, {round(v3_deg_psi,2)}')

i=5
q = euler_to_quaternion(euler_df['phi'][i], euler_df['theta'][i], euler_df['psi'][i])
print("w =", q[0])
print("x =", q[1])
print("y =", q[2])
print("z =", q[3])

quaternions = []
for row in range(len(acc)):
    q = euler_to_quaternion(euler_df['phi'][row], euler_df['theta'][row], euler_df['psi'][row])
    quaternions.append(q)

print("w =", quaternions[i][0])
print("x =", quaternions[i][1])
print("y =", quaternions[i][2])
print("z =", quaternions[i][3])

scene.forward = vector(.5, -1, -1)
scene.background = color.gray(.2)
arrowL = 4
arrowT = .05
ball_r = 3
point_ball_r = ball_r / 30


# ---------------------- VPHYTON VIZUALIZATION

show_path = False


mySphere = sphere(radius=ball_r, color=vector(1, 1, 0), opacity=.5)
xArrow = arrow(axis=vector(1, 0, 0), color=vector(1, 1, 1), length=arrowL, shaftwidth=arrowT)
yArrow = arrow(axis=vector(0, 1, 0), color=vector(1, 1, 1), length=arrowL, shaftwidth=arrowT)
zArrow = arrow(axis=vector(0, 0, 1), color=vector(1, 1, 1), length=arrowL, shaftwidth=arrowT)

frontArrow = arrow(axis=vector(1, 0, 0), color=vector(1, 0.5, 0.5), length=arrowL * 2, shaftwidth=arrowT)
upArrow = arrow(axis=vector(0, 1, 0), color=vector(0.5, 1, 0.5), length=arrowL * 1.2, shaftwidth=arrowT)
sideArrow = arrow(axis=vector(0, 0, 1), color=vector(0.5, 0.5, 1), length=arrowL * 1.2, shaftwidth=arrowT)

f_ball_point = sphere(make_trail=show_path, trail_color=frontArrow.color, radius=point_ball_r,
                      color=frontArrow.color, pos=frontArrow.axis, opacity=.5)
u_ball_point = sphere(make_trail=show_path, trail_color=upArrow.color, radius=point_ball_r,
                      color=upArrow.color, pos=upArrow.axis, opacity=.5)
s_ball_point = sphere(make_trail=show_path, trail_color=sideArrow.color, radius=point_ball_r,
                      color=sideArrow.color, pos=sideArrow.axis, opacity=.5)

arduino_board = box(length=2, width=.2, heght=.1, opacity=.8, pos=vector(0, 0, 0))
chip = box(length=.5, width=.05, heght=.05, pos=vector(-.5, .1 + .05, 0), color=color.blue)
arduino = compound([arduino_board, chip])

run = False
def runRadio(x):
    global run
    print(x.checked)
    if x.checked:
        run = True
    else:
        run = False


radio(bind=runRadio, text='Run')
scene.append_to_caption('\n\n')

def pathShow(x):
    global show_path
    print(x.checked)
    if x.checked:
        show_path = True
    else:
        show_path = False



checkbox(bind=pathShow, text='Show trace')

scene.append_to_caption('\n\n')

frame = 0
def changeSpeed(x):
    global frame
    frame = x.value


min_index = 0
max_index = 1000

wtext(text='Change frame')
scene.append_to_caption('\n')
slider(bind=changeSpeed, vertical=False, min=min_index, max=max_index, value=20)


for i, w in enumerate(quaternions):
    rate(50)
    while not run:
        q0 = quaternions[frame][0]
        q1 = quaternions[frame][1]
        q2 = quaternions[frame][2]
        q3 = quaternions[frame][3]

        roll = atan2(2 * (q0 * q1 + q2 * q3), 1 - 2 * (q1 * q1 + q2 * q2))
        pitch = asin(2 * (q0 * q2 - q3 * q1))
        yaw = atan2(2 * (q0 * q3 + q1 * q2), 1 - 2 * (q2 * q2 + q3 * q3))

        k = vector(cos(yaw) * cos(pitch), sin(pitch), sin(yaw) * cos(pitch))

        y = vector(0, 1, 0)
        s = cross(k, y)
        v = cross(s, k)

        v_rotated = v * cos(roll) + cross(k, v) * sin(roll)

        frontArrow.axis = k
        upArrow.axis = v_rotated
        sideArrow.axis = cross(k, v_rotated)
        arduino.axis = k
        arduino.up = v_rotated

        frontArrow.length = arrowL * 2
        upArrow.length = arrowL * 1.6
        sideArrow.length = arrowL * 1.2
        f_ball_point.pos = frontArrow.axis
        u_ball_point.pos = upArrow.axis
        s_ball_point.pos = sideArrow.axis
        f_ball_point.make_trail = show_path
        u_ball_point.make_trail = show_path
        s_ball_point.make_trail = show_path


    q0 = quaternions[i][0]
    q1 = quaternions[i][1]
    q2 = quaternions[i][2]
    q3 = quaternions[i][3]

    roll = atan2(2 * (q0 * q1 + q2 * q3), 1 - 2 * (q1 * q1 + q2 * q2))
    pitch = asin(2 * (q0 * q2 - q3 * q1))
    yaw = atan2(2 * (q0 * q3 + q1 * q2), 1 - 2 * (q2 * q2 + q3 * q3))

    k = vector(cos(yaw) * cos(pitch), sin(pitch), sin(yaw) * cos(pitch))

    y = vector(0, 1, 0)
    s = cross(k, y)
    v = cross(s, k)

    v_rotated = v * cos(roll) + cross(k, v) * sin(roll)

    frontArrow.axis = k
    upArrow.axis = v_rotated
    sideArrow.axis = cross(k, v_rotated)
    arduino.axis = k
    arduino.up = v_rotated

    frontArrow.length = arrowL * 2
    upArrow.length = arrowL * 1.6
    sideArrow.length = arrowL * 1.2
    f_ball_point.pos = frontArrow.axis
    u_ball_point.pos = upArrow.axis
    s_ball_point.pos = sideArrow.axis
    f_ball_point.make_trail = show_path
    u_ball_point.make_trail = show_path
    s_ball_point.make_trail = show_path