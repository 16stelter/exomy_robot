#!/usr/bin/env python
import math
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import qwiic_icm20948
from tf_transformations import quaternion_from_euler
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from ahrs.filters import Madgwick

class IMUWrapper(Node):
    def __init__(self):
        super().__init__('imu_wrapper')
        self.imu_raw_pub = self.create_publisher(Imu, 'imu_raw', 10)
        self.imu_pub = self.create_publisher(Imu, 'imu', 10)
        self.imu = qwiic_icm20948.QwiicIcm20948()
        if not self.imu.connected:
            self.get_logger().error("IMU not connected. Please check the connection.")
            return
        
        self.parameters = {}
        self.init_params()

        self.tf_broadcaster = TransformBroadcaster(self)

        self.imu.setFullScaleRangeAccel(self.parameters['accel_range'])
        self.imu.setFullScaleRangeGyro(self.parameters['gyro_range'])
        self.imu.begin()
        self.readImu()

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

    def readImu(self):
        last_time = None
        q = np.array([1.0, 0.0, 0.0, 0.0]) 
        madgwick = Madgwick()

        alpha_acc = 0.2
        alpha_gyr = 0.2
        alpha_mag = 0.2
        filtered_acc = np.array([0.0, 0.0, 0.0])
        filtered_gyr = np.array([0.0, 0.0, 0.0])
        filtered_mag = np.array([0.0, 0.0, 0.0])

        while rclpy.ok():
            if self.imu.dataReady():
                self.imu.getAgmt()
                imu_raw = Imu()
                imu_raw.header.stamp = self.get_clock().now().to_msg()
                imu_raw.header.frame_id = 'imu'
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

                ax = (imu_raw.linear_acceleration.x - self.parameters["accel_x_offset"]* self.parameters["accel_x_scale"]) / self.parameters["accel_scale"] * self.parameters["gravity"]
                ay = (imu_raw.linear_acceleration.y - self.parameters["accel_y_offset"]* self.parameters["accel_y_scale"]) / self.parameters["accel_scale"] * self.parameters["gravity"]
                az = (imu_raw.linear_acceleration.z - self.parameters["accel_z_offset"]* self.parameters["accel_z_scale"]) / self.parameters["accel_scale"] * self.parameters["gravity"]
                gx = (imu_raw.angular_velocity.x - self.parameters["gyro_x_offset"]) / self.parameters["gyro_scale"] * (math.pi / 180.0)
                gy = (imu_raw.angular_velocity.y - self.parameters["gyro_y_offset"]) / self.parameters["gyro_scale"] * (math.pi / 180.0)
                gz = (imu_raw.angular_velocity.z - self.parameters["gyro_z_offset"]) / self.parameters["gyro_scale"] * (math.pi / 180.0)
                mx = (float(self.imu.mxRaw) - self.parameters["mag_x_offset"]) * self.parameters["mag_scale"] * self.parameters["mag_x_scale"]
                my = (float(self.imu.myRaw) - self.parameters["mag_y_offset"]) * self.parameters["mag_scale"] * self.parameters["mag_y_scale"]
                mz = (float(self.imu.mzRaw) - self.parameters["mag_z_offset"]) * self.parameters["mag_scale"] * self.parameters["mag_z_scale"]

                acc = np.array([ax, ay, az])
                filtered_acc = alpha_acc * acc + (1 - alpha_acc) * filtered_acc
                norm_acc = filtered_acc / np.linalg.norm(filtered_acc)
                #acc /= np.linalg.norm(acc)
                gyr = np.array([gx/3, gy/3, gz/3]) # idk why but this seems to help
                filtered_gyr = alpha_gyr * gyr + (1 - alpha_gyr) * filtered_gyr
                norm_gyr = filtered_gyr
                mag = np.array([mx, -my, -mz]) # y and z axis of the magnetometer are inverted
                filtered_mag = alpha_mag * mag + (1 - alpha_mag) * filtered_mag
                norm_mag = filtered_mag / np.linalg.norm(filtered_mag)
                #mag /= np.linalg.norm(mag)

                now = self.get_clock().now()
                if last_time is None:
                    madgwick.dt = 0.01
                else:
                    madgwick.dt = (now - last_time).nanoseconds * 1e-9
                q = madgwick.updateMARG(q, gyr=norm_gyr, acc=norm_acc, mag=norm_mag)
                qw, qx, qy, qz = q
                last_time = now

                self.imu_raw_pub.publish(imu_raw)
                imu = Imu()
                imu.header.stamp = self.get_clock().now().to_msg()
                imu.header.frame_id = 'imu'
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

                t = TransformStamped()
                t.header.stamp = self.get_clock().now().to_msg()
                t.header.frame_id = 'base_link'
                t.child_frame_id = 'imu'
                t.transform.translation.x = 0.0
                t.transform.translation.y = 0.0
                t.transform.translation.z = 0.0
                t.transform.rotation.x = qx
                t.transform.rotation.y = qy
                t.transform.rotation.z = qz
                t.transform.rotation.w = qw
                self.tf_broadcaster.sendTransform(t)




def main(args=None):
    rclpy.init()
    imu_wrapper = IMUWrapper()
    rclpy.spin(imu_wrapper)
    imu_wrapper.destroy_node()
    rclpy.shutdown()