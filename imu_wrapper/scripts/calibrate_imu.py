#!usr/bin/env python

import yaml
import qwiic_icm20948
import time, sys
import threading


stop_flag = False
imu = qwiic_icm20948.QwiicIcm20948()
# ax, ay, az, mx, my, mz, gx, gy, gz
offsets = [0.0] * 9
# ax, ay, az, mx, my, mz
scale = [0.0] * 6

# +ax, -ax, +ay, -ay, +az, -az
a_biases = [-sys.float_info.max, sys.float_info.max, -sys.float_info.max, sys.float_info.max, -sys.float_info.max, sys.float_info.max]
# +mx, -mx, +my, -my, +mz, -mz
m_biases = [-sys.float_info.max, sys.float_info.max, -sys.float_info.max, sys.float_info.max, -sys.float_info.max, sys.float_info.max]

def wait_for_enter():
    global stop_flag
    input()
    stop_flag = True

GYRO_SCALES = [131.0, 65.5, 32.8, 16.4]
ACCEL_SCALES = [16384.0, 8192.0, 4096.0, 2048.0]
MAG_SCALES = 4900.0 / 32752.0 # fixed

print("Starting IMU calibration service for Sunspark ICM 20984.")
if not imu.connected:
    print("IMU not connected. Aborting.")
    exit()

config = {}
config["imu_wrapper"] = {}
config["imu_wrapper"]["ros__parameters"] = {}

print("First, choose which ranges to use for the accelerometer and gyroscope.")
arange = input("Choose accelerometer range (0: +-2g, 1: +-4g, 2: +-8g, 3: +-16g). Default is 0: ")
grange = input("Choose gyroscope range (0: +-250dps, 1: +-500dps, 2: +-1000dps, 3: +-2000dps). Default is 0: ")
if grange not in ['0', '1', '2', '3']:
    grange = 0
if arange not in ['0', '1', '2', '3']:
    arange = 0
grange = int(grange)
arange = int(arange)
imu.begin()
imu.setFullScaleRangeAccel(arange)
imu.setFullScaleRangeGyro(grange)


config["imu_wrapper"]["ros__parameters"]["accel_range"] = arange
config["imu_wrapper"]["ros__parameters"]["gyro_range"] = grange
config["imu_wrapper"]["ros__parameters"]["accel_scale"] = ACCEL_SCALES[arange]
config["imu_wrapper"]["ros__parameters"]["gyro_scale"] = GYRO_SCALES[grange]
config["imu_wrapper"]["ros__parameters"]["mag_scale"] = MAG_SCALES 
config["imu_wrapper"]["ros__parameters"]["gravity"] = 9.81

input("Place the robot on a flat surface, then press enter to continue. Do not touch the robot while it is calibrating. This may take a few seconds.")

t1 = time.time()

gx_sum = gy_sum = gz_sum = az_sum = samples = 0

while time.time() - t1 < 5.0:
    if imu.dataReady():
            imu.getAgmt()
            samples +=1
            gx_sum += imu.gxRaw
            gy_sum += imu.gyRaw
            gz_sum += imu.gzRaw
            az_sum += imu.azRaw

offsets[6] = gx_sum / samples
offsets[7] = gy_sum / samples
offsets[8] = gz_sum / samples
a_biases[4] = az_sum / samples
print("Gyroscope offsets:")
print(f"  gx: {offsets[6]:.2f}")
print(f"  gy: {offsets[7]:.2f}")
print(f"  gz: {offsets[8]:.2f}")

print("Next, we calibrate the accelerometer. To do this, you need to rotate the robot into target positions and hold it there for 5 seconds each." \
"I will prompt you which position to do next. Try to hold the robot as steady as possible.")
input("Rotate the robot forward 90 degrees so that the front faces down. Press enter once you are in position.")
t1 = time.time()
samples = ax_sum = 0
while time.time() - t1 < 5.0:
    if imu.dataReady():
            imu.getAgmt()
            samples +=1
            ax_sum += imu.axRaw
a_biases[0] = ax_sum / samples
print("Done. It is now safe to put the robot down again.")
input("Now rotate the robot backward 90 degrees so that the front faces up. Press enter once you are in position.")
t1 = time.time()
samples = ax_sum = 0
while time.time() - t1 < 5.0:
    if imu.dataReady():
            imu.getAgmt()
            samples +=1
            ax_sum += imu.axRaw
a_biases[1] = ax_sum / samples
print("Done. It is now safe to put the robot down again.")
input("Now rotate the robot to the left 90 degrees so that the left side faces down. Press enter once you are in position.")
t1 = time.time()
samples = ay_sum = 0
while time.time() - t1 < 5.0:
    if imu.dataReady():
            imu.getAgmt()
            samples +=1
            ay_sum += imu.ayRaw
a_biases[2] = ay_sum / samples
print("Done. It is now safe to put the robot down again.")
input("Now rotate the robot to the right 90 degrees so that the right side faces down. Press enter once you are in position.")
t1 = time.time()
samples = ay_sum = 0
while time.time() - t1 < 5.0:
    if imu.dataReady():
            imu.getAgmt()
            samples +=1
            ay_sum += imu.ayRaw
a_biases[3] = ay_sum / samples
print("Done. It is now safe to put the robot down again.")
input("Now rotate the robot upside down so that the top faces down. Press enter once you are in position.")
t1 = time.time()
samples = az_sum = 0
while time.time() - t1 < 5.0:
    if imu.dataReady():
            imu.getAgmt()
            samples +=1
            az_sum += imu.azRaw
a_biases[5] = az_sum / samples
print("Done. It is now safe to put the robot down again.")
offsets[0] = (a_biases[0] + a_biases[1]) / 2.0
offsets[1] = (a_biases[2] + a_biases[3]) / 2.0
offsets[2] = (a_biases[4] + a_biases[5]) / 2.0
scale[0] = (2 * ACCEL_SCALES[arange]) / (a_biases[0] - a_biases[1])
scale[1] = (2 * ACCEL_SCALES[arange]) / (a_biases[2] - a_biases[3])
scale[2] = (2 * ACCEL_SCALES[arange]) / (a_biases[4] - a_biases[5])
print("Accelerometer offsets and scale factors:")
print(f"  ax offset: {offsets[0]:.2f}, scale: {scale[0]:.6f}")
print(f"  ay offset: {offsets[1]:.2f}, scale: {scale[1]:.6f}")
print(f"  az offset: {offsets[2]:.2f}, scale: {scale[2]:.6f}")

input("Now, slowly rotate the robot around all three axes. Make sure you cover the entire range of all axes. " \
"Press enter to start, then press enter again when you are done.")

threading.Thread(target=wait_for_enter, daemon=True).start()
while not stop_flag:
    if imu.dataReady():
        imu.getAgmt()
        mx = imu.mxRaw
        my = imu.myRaw
        mz = imu.mzRaw
        if mx > m_biases[0]:
            m_biases[0] = mx
        if mx < m_biases[1]:
            m_biases[1] = mx
        if my > m_biases[2]:
            m_biases[2] = my
        if my < m_biases[3]:
            m_biases[3] = my
        if mz > m_biases[4]:
            m_biases[4] = mz
        if mz < m_biases[5]:
            m_biases[5] = mz

offsets[3] = (m_biases[0] + m_biases[1]) / 2.0
offsets[4] = (m_biases[2] + m_biases[3]) / 2.0
offsets[5] = (m_biases[4] + m_biases[5]) / 2.0
rx = (m_biases[0] - m_biases[1]) / 2.0
ry = (m_biases[2] - m_biases[3]) / 2.0
rz = (m_biases[4] - m_biases[5]) / 2.0
r = (rx + ry + rz) / 3.0
scale[3] = r / rx
scale[4] = r / ry
scale[5] = r / rz
print("Magnetometer offsets and scale factors:")
print(f"  mx offset: {offsets[3]:.2f}, scale: {scale[3]:.6f}")
print(f"  my offset: {offsets[4]:.2f}, scale: {scale[4]:.6f}")
print(f"  mz offset: {offsets[5]:.2f}, scale: {scale[5]:.6f}")

print("Writing calibration to imu.yaml.")

config["imu_wrapper"]["ros__parameters"]["gyro_x_offset"] = int(offsets[6])
config["imu_wrapper"]["ros__parameters"]["gyro_y_offset"] = int(offsets[7])
config["imu_wrapper"]["ros__parameters"]["gyro_z_offset"] = int(offsets[8])
config["imu_wrapper"]["ros__parameters"]["accel_x_offset"] = int(offsets[0])
config["imu_wrapper"]["ros__parameters"]["accel_y_offset"] = int(offsets[1])
config["imu_wrapper"]["ros__parameters"]["accel_z_offset"] = int(offsets[2])
config["imu_wrapper"]["ros__parameters"]["mag_x_offset"] = int(offsets[3])
config["imu_wrapper"]["ros__parameters"]["mag_y_offset"] = int(offsets[4])
config["imu_wrapper"]["ros__parameters"]["mag_z_offset"] = int(offsets[5])
config["imu_wrapper"]["ros__parameters"]["accel_x_scale"] = scale[0]
config["imu_wrapper"]["ros__parameters"]["accel_y_scale"] = scale[1]
config["imu_wrapper"]["ros__parameters"]["accel_z_scale"] = scale[2]
config["imu_wrapper"]["ros__parameters"]["mag_x_scale"] = scale[3]
config["imu_wrapper"]["ros__parameters"]["mag_y_scale"] = scale[4]
config["imu_wrapper"]["ros__parameters"]["mag_z_scale"] = scale[5]

try:
    with open("../config/imu.yaml", 'w') as stream:
        yaml.dump(config, stream)
except Exception as exc:
    print(exc)
    print("Could not write imu.yaml. Aborting.")
    exit()

print("Calibration complete.")