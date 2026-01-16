import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
import numpy as np
from scipy.spatial.transform import Rotation as R
import os

class KittiConverter(Node):
    def __init__(self):
        super().__init__('kitti_converter')
        
        # 1. Topic names must match the YAML
        self.pose_pub = self.create_publisher(PoseStamped, '/kitti/noisy_pose', 10)
        self.path_pub = self.create_publisher(Path, '/kitti/ground_truth_path', 10)
        
        # 2. File Path Logic
        script_dir = os.path.dirname(os.path.realpath(__file__))
        self.file_path = os.path.join(script_dir, '..', 'data', '00.txt')
        
        self.poses = [line.strip().split() for line in open(self.file_path, 'r')]
        self.full_path = Path()
        self.full_path.header.frame_id = 'odom'
        
        self.idx = 0
        self.timer = self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        if self.idx >= len(self.poses): return

        data = np.array(self.poses[self.idx], dtype=float).reshape(3, 4)
        now = self.get_clock().now().to_msg()

        # COORDINATE TRANSFORMATION (KITTI Camera -> ROS REP-105)
        # KITTI: Z Forward, X Right, Y Down
        # ROS:   X Forward, Y Left,  Z Up
        tx, ty, tz = data[2, 3], -data[0, 3], -data[1, 3]

        # Orientation Transformation
        r = R.from_matrix(data[:, :3])
        correction = R.from_euler('xyz', [-90, 0, -90], degrees=True)
        quat = (correction * r).as_quat()

        # Create Ground Truth Pose for the Path
        gt_pose = PoseStamped()
        gt_pose.header.stamp, gt_pose.header.frame_id = now, 'odom'
        gt_pose.pose.position.x, gt_pose.pose.position.y, gt_pose.pose.position.z = tx, ty, tz
        gt_pose.pose.orientation.x, gt_pose.pose.orientation.y, gt_pose.pose.orientation.z, gt_pose.pose.orientation.w = quat

        # Add Noise to create "Sensor Data" for EKF
        noisy_pose = PoseStamped()
        noisy_pose.header = gt_pose.header
        noisy_pose.pose.position.x = tx + np.random.normal(0, 0.15) # 15cm noise
        noisy_pose.pose.position.y = ty + np.random.normal(0, 0.15)
        noisy_pose.pose.position.z = tz
        noisy_pose.pose.orientation = gt_pose.pose.orientation

        # Publish
        self.pose_pub.publish(noisy_pose)
        self.full_path.poses.append(gt_pose)
        self.path_pub.publish(self.full_path)
        self.idx += 1

def main():
    rclpy.init()
    rclpy.spin(KittiConverter())
    rclpy.shutdown()