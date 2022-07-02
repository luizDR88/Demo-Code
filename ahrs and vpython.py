import pandas as pd
from math import *
import numpy as np
import os
import matplotlib
from vpython import *

folder_path = os.getcwd()
files = [ 'pitch.csv', 'pitch and roll.csv',
        'ball_IMU_tabela_final_fdt6_cropped.csv', 'ball_IMU_tabela_final_fdt9.csv', 'ball_IMU_tabela_final_fdt8.csv',
        ]
selected_file = 1

file_path = os.path.join(folder_path, 'files', files[selected_file])
ball_IMU = pd.read_csv(file_path)


orig_columns=['Time since start in ms ',
              'ACCELEROMETER X (m/s²)', 'ACCELEROMETER Y (m/s²)','ACCELEROMETER Z (m/s²)',
              'GYROSCOPE X (rad/s)', 'GYROSCOPE Y (rad/s)', 'GYROSCOPE Z (rad/s)',
              'MAGNETIC FIELD X (μT)', 'MAGNETIC FIELD Y (μT)', 'MAGNETIC FIELD Z (μT)']
columns=['elapsed_time', 'ay', 'ax', 'az', 'gy', 'gx', 'gz', 'my', 'mx', 'mz']


for i, col in enumerate(columns):
    ball_IMU[col] = ball_IMU[orig_columns[i]]

rows = len(ball_IMU)

data_mag = ball_IMU.loc[:,['elapsed_time', 'mx', 'my','mz']]
medians = data_mag.median()
columns=['elapsed_time' ,'mx', 'my', 'mz']
calibrated_mag = np.ndarray(shape=(rows, 4), dtype=float)
calibrated_mag = pd.DataFrame(data=calibrated_mag, columns=columns)

calibrated_mag.loc[:,'elapsed_time']=data_mag.loc[:,'elapsed_time']

for i, col in enumerate(columns):
    print(col, medians[i]*-1, data_mag[col].median())
    if i > 0 :
        calibrated_mag.loc[:, col] = data_mag.loc[:,col] + (medians[i]*-1)

data_acc = ball_IMU[['elapsed_time', 'ax', 'ay','az']]
data_gyr = ball_IMU[['elapsed_time', 'gx', 'gy','gz']]

# ----------------------------------- ADRS Filters

acc_data = np.array(data_acc.iloc[:,1:])
gyro_data = np.array(data_gyr.iloc[:,1:])
mag_data = np.array(data_mag.iloc[:,1:])

# ----------------------------------- Madgwick 1
from ahrs.filters import Madgwick
madgwick = Madgwick()

Qmadgwick = np.tile([1., 0., 0., 0.], (rows, 1))  # Allocate for quaternions
for t in range(1, rows):
    Qmadgwick[t] = madgwick.updateIMU(Qmadgwick[t - 1], gyr=gyro_data[t], acc=acc_data[t])

print(Qmadgwick[10:20])

#  *** Frequency
t = 20

sampling_rate = 1/ (ball_IMU['elapsed_time'][t] - ball_IMU['elapsed_time'][t-1])
print(f'Data Frequency {t}: {sampling_rate}')
#  ***

# ----------------------------------- Madgwick 2

madgwick2 = Madgwick(gyr=gyro_data, acc=acc_data, mag=mag_data)
zero_div = 0

Qmadgwick2 = np.tile([1., 0., 0., 0.], (rows, 1))  # Allocate for quaternions
for t in range(1, rows):
    if t > 0:
        if ball_IMU['elapsed_time'][t] - ball_IMU['elapsed_time'][t - 1] > 0:
            madgwick2.Dt = 1 / (ball_IMU['elapsed_time'][t] - ball_IMU['elapsed_time'][t - 1])
        else:
            madgwick2.Dt = 1 / 100
            zero_div += 1

    Qmadgwick2[t] = madgwick2.updateIMU(Qmadgwick2[t - 1], gyr=gyro_data[t], acc=acc_data[t])

print(f'{Qmadgwick2[10:20]}, {zero_div}')


# ----------------------------------- AQCUA
from ahrs.filters import AQUA

gyro_rad_data = np.array([[radians(gx), radians(gy), radians(gz)] for gx, gy, gz in data_gyr.iloc[:,1:].values])

alpha = 0.5
belta = 0.5
cutoff = 0.8
threshold = 0.9

aqua_quaternions = AQUA(gyr=gyro_data, acc=acc_data, mag=mag_data, alpha=alpha, belta=belta, cutoff=cutoff,
                        threshold=threshold)

Q_aqua = np.tile([1., 0., 0., 0.], (rows, 1))  # Allocate for quaternions
for t in range(1, rows):
    if t > 0:
        if ball_IMU['elapsed_time'][t] - ball_IMU['elapsed_time'][t - 1] > 0:
            aqua_quaternions.Dt = 1 / (ball_IMU['elapsed_time'][t] - ball_IMU['elapsed_time'][t - 1])
        else:
            aqua_quaternions.Dt = 1 / 100
            zero_div += 1

    Q_aqua[t] = aqua_quaternions.updateIMU(Q_aqua[t - 1], gyr=gyro_data[t], acc=acc_data[t])

# ---------------------- VPHYTON VIZUALIZATION



scene.forward = vector(1, 1, 0)
scene.background = color.gray(.2)

ball_r = 15
arrowL = ball_r + 4
arrowT = 0.5

# logo=box(length=8, width=.2, heght=.2, pos=vector(0, 0, ball_r+.1), color=color.black)
# myBall = sphere(radius=ball_r, color=vector(1,1,0), opacity=.5, pos=vector(0, 0, 0))
# myBall = box(length=ball_r, width=ball_r/2, heght=ball_r/10, opacity=.5, pos=vector(0, 0, 0) , color=vector(0.2,0.2,0.2))
# ball = compound([logo, myBall])

gArrow = arrow(axis=vector(0, -1, 0), color=vector(1, 1, 1), length=arrowL + 8, shaftwidth=arrowT)

k = vector(1, 0, 0)
s = vector(0, 0, 1)
v = vector(0, 1, 0)

frontArrow = arrow(axis=k, color=k, length=arrowL * 2, shaftwidth=arrowT)
upArrow = arrow(axis=v, color=v, length=arrowL * 1.2, shaftwidth=arrowT)
sideArrow = arrow(axis=s, color=s, length=arrowL * 1.2, shaftwidth=arrowT)

counter = label(text='Elapsed Time: ' + str(0), pos=vector(5, -ball_r - 4, 3), color=color.white, opacity=.8)
values = label(text='Quaternion: Qw=0, Qx=0, Qy=0, Qz=0', pos=vector(-10, ball_r + 10, -3), color=color.white,
               opacity=.8)
result_angles = label(text='Angles: Roll=0, Pitch=0, Yaw=0', pos=vector(-5, ball_r + 7, -3), color=color.white,
                      opacity=.8)

arduino_board = box(length=ball_r, width=ball_r/2, heght=ball_r/10, opacity=.8, pos=vector(0, 0, 0), color=vector(0.2,0.2,0.2))
chip = box(length=ball_r*.8, width=ball_r*.5, heght=.5, pos=vector(-.5, +.15, 0), color=color.blue)
arduino = compound([arduino_board, chip])


quaternians_filters = [Qmadgwick, Qmadgwick2, Q_aqua]
selected_filtered_quaternions = 1

v_rotated = v

selected_filter_quaternion = quaternians_filters[selected_filtered_quaternions]
run = False
frame = 0
rate(50)

def runRadio(x):
    global run
    print(x.checked)
    quaternion_to_run = selected_filter_quaternion[frame:]
    if x.checked:
        for i, quaternion in enumerate(selected_filter_quaternion):
            rate(50)
            update_vpython(quaternion, i)
            print(quaternion)
    else:
        run = False

radio(bind=runRadio, text='Run')
scene.append_to_caption('\n\n')


def changeSpeed(x):
    global frame
    frame = x.value
    update_vpython(selected_filter_quaternion[frame], frame)

min_index = 0
max_index = len(selected_filter_quaternion)

wtext(text='Change frame')
scene.append_to_caption('\n')
slider(bind=changeSpeed, vertical=False, min=min_index, max=max_index, value=0)

scene.append_to_caption('\n\n')

def update_vpython(quaternion, i):
    qw, qx, qy, qz = quaternion
    rate(50)
    try:
        roll = atan2(2 * (qw * qx + qy * qz), 1 - 2 * (qx ** 2 + qy ** 2))
        pitch = asin(2 * (qw * qy - qz * qx))
        yaw = atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy ** 2 + qz ** 2))
    except ValueError:
        pass

    #     try:
    #         yaw = atan2(2.0*(qy*qz + qw*qx), qw*qw - qx*qx - qy*qy + qz*qz)
    #         pitch = asin(-2.0*(qx*qz - qw*qy))
    #         roll = atan2(2.0*(qx*qy + qw*qz), qw*qw + qx*qx - qy*qy - qz*qz)
    #     except ValueError:
    #         pass

    k = vector(cos(yaw) * cos(pitch), sin(pitch), sin(yaw) * cos(pitch))
    y = vector(0, 1, 0)
    s = cross(k, y)
    v = cross(s, k)

    v_rotated = v * cos(roll) + cross(k, v) * sin(roll)

    counter.text = 'Elapsed Time: ' + str(data_acc['elapsed_time'][i]/1000) + 's'
    values.text = f'Quaternion: Qw={round(qw, 2)}, Qx={round(qx, 2)}, Qy={round(qy, 2)}, Qz={round(qz, 2)}'
    result_angles.text = f'Angles: Roll={round(roll, 2)}, Pitch={round(pitch, 2)}, Yaw={round(yaw, 2)}'

    frontArrow.axis = k
    upArrow.axis = v_rotated
    sideArrow.axis = cross(k, v_rotated)
    arduino.axis = k
    arduino.up = v_rotated
    # ball.up = v_rotated
    # ball.axis = k

    frontArrow.length = arrowL
    upArrow.length = arrowL
    sideArrow.length = arrowL


while True:
    pass





