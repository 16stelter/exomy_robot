#!/usr/bin/env python
import math
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import qwiic_icm20948
from ahrs.filters import Madgwick


def quat_mult(q1, q2):
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    return np.array([
        w1*w2 - x1*x2 - y1*y2 - z1*z2,
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2
    ])

def quat_inv(q):
    w, x, y, z = q
    norm = w*w + x*x + y*y + z*z
    return np.array([w, -x, -y, -z]) / norm


class IMUWrapper(Node):
    def __init__(self):
        super().__init__('imu_wrapper')
        self.ns = self.get_namespace()
        if self.ns == '/':
            self.ns = ''
        self.imu_raw_pub = self.create_publisher(Imu, 'imu_raw', 10)
        self.imu_pub = self.create_publisher(Imu, 'imu', 10)
        self.imu = qwiic_icm20948.QwiicIcm20948()
        if not self.imu.connected:
            self.get_logger().error("IMU not connected. Please check the connection.")
            return
        
        self.parameters = {}
        self.init_params()

        self.imu.begin()
        self.imu.setFullScaleRangeAccel(self.parameters['accel_range'])
        self.imu.setFullScaleRangeGyro(self.parameters['gyro_range'])

        self.last_time = None
        self.q = np.array([1.0, 0.0, 0.0, 0.0]) 
        self.madgwick = Madgwick(beta=0.9)

        self.alpha_acc = 0.005
        self.alpha_gyr = 0.005
        self.alpha_mag = 0.005
        self.filtered_acc = None
        self.filtered_gyr = None
        self.filtered_mag = None

        self.q_offset = None
        self.startup_counter = 0

        self.timer = self.create_timer(0.02, self.read_imu)

    def init_params(self):
        self.declare_parameter('accel_range', 0)
        self.parameters['accel_range'] = self.get_parameter('accel_range').value
        self.declare_parameter('gyro_range', 0)
        self.parameters['gyro_range'] = self.get_parameter('gyro_range').value
        self.declare_parameter('accel_scale', 0.0)
        self.parameters['accel_scale'] = self.get_parameter('accel_scale').value
        self.declare_parameter('gyro_scale', 0.0)
        self.parameters['gyro_scale'] = self.get_parameter('gyro_scale').value
        self.declare_parameter('mag_scale', 0.0)
        self.parameters['mag_scale'] = self.get_parameter('mag_scale').value
        self.declare_parameter('gravity', 9.81)
        self.parameters['gravity'] = self.get_parameter('gravity').value
        self.declare_parameter('accel_x_offset', 0)
        self.parameters['accel_x_offset'] = self.get_parameter('accel_x_offset').value
        self.declare_parameter('accel_y_offset', 0)
        self.parameters['accel_y_offset'] = self.get_parameter('accel_y_offset').value
        self.declare_parameter('accel_z_offset', 0)
        self.parameters['accel_z_offset'] = self.get_parameter('accel_z_offset').value
        self.declare_parameter('gyro_x_offset', 0)
        self.parameters['gyro_x_offset'] = self.get_parameter('gyro_x_offset').value
        self.declare_parameter('gyro_y_offset', 0)
        self.parameters['gyro_y_offset'] = self.get_parameter('gyro_y_offset').value
        self.declare_parameter('gyro_z_offset', 0) 
        self.parameters['gyro_z_offset'] = self.get_parameter('gyro_z_offset').value
        self.declare_parameter('mag_x_offset', 0)
        self.parameters['mag_x_offset'] = self.get_parameter('mag_x_offset').value
        self.declare_parameter('mag_y_offset', 0)
        self.parameters['mag_y_offset'] = self.get_parameter('mag_y_offset').value
        self.declare_parameter('mag_z_offset', 0)
        self.parameters['mag_z_offset'] = self.get_parameter('mag_z_offset').value
        self.declare_parameter('accel_x_scale', 1.0)
        self.parameters['accel_x_scale'] = self.get_parameter('accel_x_scale').value
        self.declare_parameter('accel_y_scale', 1.0)
        self.parameters['accel_y_scale'] = self.get_parameter('accel_y_scale').value
        self.declare_parameter('accel_z_scale', 1.0)
        self.parameters['accel_z_scale'] = self.get_parameter('accel_z_scale').value
        self.declare_parameter('mag_x_scale', 0.0)
        self.parameters['mag_x_scale'] = self.get_parameter('mag_x_scale').value
        self.declare_parameter('mag_y_scale', 0.0)
        self.parameters['mag_y_scale'] = self.get_parameter('mag_y_scale').value
        self.declare_parameter('mag_z_scale', 0.0)
        self.parameters['mag_z_scale'] = self.get_parameter('mag_z_scale').value

    def read_imu(self):
        if not self.imu.dataReady():
            return

        self.imu.getAgmt()
        imu_raw = Imu()
        imu_raw.header.stamp = self.get_clock().now().to_msg()
        imu_raw.header.frame_id = f'{self.ns}/imu' if self.ns else 'imu'
        imu_raw.linear_acceleration.x = float(self.imu.axRaw)
        imu_raw.linear_acceleration.y = float(self.imu.ayRaw)
        imu_raw.linear_acceleration.z = float(self.imu.azRaw)
        imu_raw.angular_velocity.x = float(self.imu.gxRaw)
        imu_raw.angular_velocity.y = float(self.imu.gyRaw)
        imu_raw.angular_velocity.z = float(self.imu.gzRaw)
        imu_raw.orientation.x = float(self.imu.mxRaw)
        imu_raw.orientation.y = float(self.imu.myRaw)
        imu_raw.orientation.z = float(self.imu.mzRaw)
        imu_raw.orientation.w = 0.0

        ax = (imu_raw.linear_acceleration.x - self.parameters["accel_x_offset"])* self.parameters["accel_x_scale"] / self.parameters["accel_scale"] * self.parameters["gravity"]
        ay = (imu_raw.linear_acceleration.y - self.parameters["accel_y_offset"])* self.parameters["accel_y_scale"] / self.parameters["accel_scale"] * self.parameters["gravity"]
        az = (imu_raw.linear_acceleration.z - self.parameters["accel_z_offset"])* self.parameters["accel_z_scale"] / self.parameters["accel_scale"] * self.parameters["gravity"]
        gx = (imu_raw.angular_velocity.x - self.parameters["gyro_x_offset"]) / self.parameters["gyro_scale"] * (math.pi / 180.0)
        gy = (imu_raw.angular_velocity.y - self.parameters["gyro_y_offset"]) / self.parameters["gyro_scale"] * (math.pi / 180.0)
        gz = (imu_raw.angular_velocity.z - self.parameters["gyro_z_offset"]) / self.parameters["gyro_scale"] * (math.pi / 180.0)
        mx = (float(self.imu.mxRaw) - self.parameters["mag_x_offset"]) * self.parameters["mag_scale"] * self.parameters["mag_x_scale"]
        my = (float(self.imu.myRaw) - self.parameters["mag_y_offset"]) * self.parameters["mag_scale"] * self.parameters["mag_y_scale"]
        mz = (float(self.imu.mzRaw) - self.parameters["mag_z_offset"]) * self.parameters["mag_scale"] * self.parameters["mag_z_scale"]

        acc = np.array([ax, ay, az])
        if self.filtered_acc is None:
            self.filtered_acc = acc.copy()
        else:
            self.filtered_acc = self.alpha_acc * acc + (1 - self.alpha_acc) * self.filtered_acc
        norm_acc = self.filtered_acc / np.linalg.norm(self.filtered_acc)
        #acc /= np.linalg.norm(acc)
        gyr = np.array([gx, gy, gz])
        if self.filtered_gyr is None:
            self.filtered_gyr = gyr.copy()
        else:
            self.filtered_gyr = self.alpha_gyr * gyr + (1 - self.alpha_gyr) * self.filtered_gyr
        norm_gyr = self.filtered_gyr
        mag = np.array([mx, -my, -mz]) # y and z axis of the magnetometer are inverted
        if self.filtered_mag is None:
            self.filtered_mag = mag.copy()
        else:
            self.filtered_mag = self.alpha_mag * mag + (1 - self.alpha_mag) * self.filtered_mag
        norm_mag = self.filtered_mag / np.linalg.norm(self.filtered_mag)
        #mag /= np.linalg.norm(mag)

        now = self.get_clock().now()
        if self.last_time is None:
            self.madgwick.Dt = 0.01
        else:
            self.madgwick.Dt = (now - self.last_time).nanoseconds * 1e-9
        q = self.madgwick.updateMARG(self.q, gyr=norm_gyr, acc=norm_acc, mag=norm_mag, dt=self.madgwick.Dt)

        if self.startup_counter < 200:
            self.startup_counter += 1
            return

        self.last_time = now

        if self.q_offset is None:
            self.q_offset = q.copy()

        q_rel = quat_mult(q, quat_inv(self.q_offset))
        qw, qx, qy, qz = q_rel

        self.imu_raw_pub.publish(imu_raw)
        imu = Imu()
        imu.header.stamp = self.get_clock().now().to_msg()
        imu.header.frame_id = f'{self.ns}/imu' if self.ns else 'imu'
        imu.linear_acceleration.x = filtered_acc[0]
        imu.linear_acceleration.y = filtered_acc[1]
        imu.linear_acceleration.z = filtered_acc[2]
        imu.angular_velocity.x = filtered_gyr[0]
        imu.angular_velocity.y = filtered_gyr[1]
        imu.angular_velocity.z = filtered_gyr[2]
        imu.orientation.x = qx
        imu.orientation.y = qy
        imu.orientation.z = qz
        imu.orientation.w = qw
        self.imu_pub.publish(imu)

def main(args=None):
    rclpy.init()
    imu_wrapper = IMUWrapper()
    rclpy.spin(imu_wrapper)
    imu_wrapper.destroy_node()
    rclpy.shutdown()
