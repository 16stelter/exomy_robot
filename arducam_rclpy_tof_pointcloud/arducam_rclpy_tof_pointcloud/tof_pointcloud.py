from argparse import ArgumentParser
from typing import Optional
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Float32MultiArray, Header
import numpy as np
from threading import Thread, Lock


from ArducamDepthCamera import (
    ArducamCamera,
    Connection,
    DeviceType,
    FrameType,
    Control,
    DepthData,
)

class Option:
    cfg: Optional[str]
    


class TOFPublisher(Node):
    def __init__(self, options: Option):
        super().__init__("arducam")
        
        namespace = self.declare_parameter('namespace', '').get_parameter_value().string_value
        tof = self.__init_camera(options)
        if tof is None:
            raise Exception("Failed to initialize camera")
        self.lock = Lock()
        self.declare_parameter('margin_x', 0)
        self.declare_parameter('margin_y', 0)
        self.margin_x = self.get_parameter('margin_x').value
        self.margin_y = self.get_parameter('margin_y').value

        self.tof_ = tof
        self.pointsize_ = self.width_ * self.height_
        self.frame_id = "sensor_frame"
        self.depth_msg_ = Float32MultiArray()
        self.publisher_ = self.create_publisher(PointCloud2, "pointcloud", 10)
        self.publisher_depth_ = self.create_publisher(
            Float32MultiArray, "depth_frame", 10
        )
        self.confidence_buf = None
        self.fx = tof.getControl(Control.INTRINSIC_FX) / 100
        self.fy = tof.getControl(Control.INTRINSIC_FY) / 100
        self.header = Header()
        self.header.frame_id = f"{namespace}/camera" if namespace else "camera"
        self.points = None
        self.running_ = True
        self.timer_ = self.create_timer(1 / 30, self.update)
        self.process_point_cloud_thr = Thread(
            target=self.__generateSensorPointCloud, daemon=True
        )
        self.process_point_cloud_thr.start()

    def __init_camera(self, options: Option):
        print("pointcloud publisher init")
        tof = ArducamCamera()
        ret = 0
        
        if options.cfg is not None:
            ret = tof.openWithFile(options.cfg, 0)
        else:
            ret = tof.open(Connection.CSI, 0)
        if ret != 0:
            print("Failed to open camera. Error code:", ret)
            return

        ret = tof.start(FrameType.DEPTH)
        if ret != 0:
            print("Failed to start camera. Error code:", ret)
            tof.close()
            return

        info = tof.getCameraInfo()
        if info.device_type == DeviceType.HQVGA:
            self.width_ = info.width
            self.height_ = info.height
            tof.setControl(Control.RANGE, 4)
        elif info.device_type == DeviceType.VGA:
            self.width_ = info.width
            self.height_ = info.height // 10 - 1
        print(f"Open camera success, width: {self.width_}, height: {self.height_}")

        print("Pointcloud publisher start")
        return tof

    def __generateSensorPointCloud(self):
        while self.running_:
            frame = self.tof_.requestFrame(200)
            if frame is not None and isinstance(frame, DepthData):
                self.fx = self.tof_.getControl(Control.INTRINSIC_FX) / 100
                self.fy = self.tof_.getControl(Control.INTRINSIC_FY) / 100
                depth_buf = frame.depth_data
                confidence_buf = frame.confidence_data

                depth_buf[confidence_buf < 30] = 0

                if self.margin_y > 0 or self.margin_x > 0:
                    depth_buf = depth_buf[self.margin_y:-self.margin_y or None,
                                        self.margin_x:-self.margin_x or None]
                    confidence_buf = confidence_buf[self.margin_y:-self.margin_y or None,
                                                    self.margin_x:-self.margin_x or None]
                h, w = depth_buf.shape

                self.depth_msg_.data = depth_buf.flatten() / 1000

                # Convert depth values ​​from millimeters to meters
                z = depth_buf / 1000.0
                z[z <= 0] = np.nan  # Handling invalid depth values

                # Calculate x and y coordinates
                u = np.arange(self.margin_x, self.margin_x + w)
                v = np.arange(self.margin_y, self.margin_y + h)
                u, v = np.meshgrid(u, v)

                # Calculate point cloud coordinates
                x = (u - self.width_ / 2) * z / self.fx
                y = (v - self.height_ / 2) * z / self.fy

                # Combined point cloud
                points = np.stack((x, y, z), axis=-1)
                mask = ~np.isnan(points).any(axis=-1)
                with self.lock:
                    self.points = points[mask]
                    self.confidence_buf = confidence_buf[mask]
                self.tof_.releaseFrame(frame)

    def update(self):
        # self.__generateSensorPointCloud()
        with self.lock:
            if self.points is None or self.confidence_buf is None:
                return
            points = self.points.copy()
            confidence = self.confidence_buf.copy()
        self.header.stamp = self.get_clock().now().to_msg()

        fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name="intensity", offset=12, datatype=PointField.FLOAT32, count=1),
        ]

        points_with_intensity = np.hstack((points, confidence.reshape(-1, 1))).astype(np.float32)
        pc2_msg_ = point_cloud2.create_cloud(
            self.header, fields, points_with_intensity
        )

        self.publisher_.publish(pc2_msg_)
        self.publisher_depth_.publish(self.depth_msg_)

    def stop(self):
        self.running_ = False
        self.process_point_cloud_thr.join()
        self.tof_.stop()
        self.tof_.close()


def main(args=None):
    rclpy.init(args=args)
    parser = ArgumentParser()
    parser.add_argument("--cfg", type=str, help="Path to camera configuration file")
    
    ns, _ = parser.parse_known_args()
    
    options = Option()
    options.cfg = ns.cfg
    
    tof_publisher = TOFPublisher(options)

    rclpy.spin(tof_publisher)
    rclpy.shutdown()
    tof_publisher.stop()


if __name__ == "__main__":
    main()
