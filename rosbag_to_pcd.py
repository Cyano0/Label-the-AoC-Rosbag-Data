import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
import open3d as o3d
from rclpy.qos import QoSProfile, ReliabilityPolicy
import numpy as np
import os
import sys
from scipy.io import savemat

class PointCloudExtractor(Node):
    def __init__(self, base_output_dir):
        super().__init__('pointcloud_extractor')

        # Define output directory
        self.output_dir = os.path.join(base_output_dir, "output_pointclouds")
        os.makedirs(self.output_dir, exist_ok=True)

        # Setup QoS profile
        qos_profile = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, depth=10)

        # Create subscription to LiDAR topic
        self.subscription = self.create_subscription(
            PointCloud2,
            '/front_lidar/points',  # Replace with your LiDAR topic
            self.pointcloud_callback,
            qos_profile
        )

        # Store timestamps for MATLAB in duration format (seconds)
        self.timestamps = []

    def pointcloud_callback(self, msg):
        if msg.width == 0:
            self.get_logger().info("Empty PointCloud2 message received.")
            return

        try:
            # Extract points from PointCloud2 message
            points = np.array([[p[0], p[1], p[2]] for p in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)])
            pcd = o3d.geometry.PointCloud()
            pcd.points = o3d.utility.Vector3dVector(points)

            # Extract timestamp (convert to seconds)
            timestamp_sec = msg.header.stamp.sec
            timestamp_nanosec = msg.header.stamp.nanosec
            timestamp = timestamp_sec + timestamp_nanosec * 1e-9  # Convert nanoseconds to fractional seconds

            # Generate filename
            timestamp_str = f"{timestamp_sec}_{timestamp_nanosec:09d}"
            file_name = os.path.join(self.output_dir, f"cloud_{timestamp_str}.pcd")

            # Save point cloud
            o3d.io.write_point_cloud(file_name, pcd)
            self.get_logger().info(f"Saved point cloud to {file_name}")

            # Store timestamp for MATLAB duration format
            self.timestamps.append(timestamp)

        except Exception as e:
            self.get_logger().error(f"Error processing point cloud: {e}")

    def save_timestamps(self):
        """ Saves timestamps as a MATLAB-compatible .mat file in `duration` format """
        try:
            timestamp_file = os.path.join(self.output_dir, "timestamps.mat")
            timestamps_np = np.array(self.timestamps).reshape(-1, 1)
            savemat(timestamp_file, {"timestamps": timestamps_np})

            # Use print() instead of self.get_logger().info()
            print(f"✅ Saved timestamps to {timestamp_file}")

        except Exception as e:
            print(f"❌ Error saving timestamps: {e}")

def main(args=None):
    rclpy.init(args=args)

    if len(sys.argv) > 1:
        base_output_dir = sys.argv[1]
    else:
        base_output_dir = input("Enter the output folder name: ").strip()
    
    base_output_dir = os.path.abspath(base_output_dir)
    os.makedirs(base_output_dir, exist_ok=True)

    node = PointCloudExtractor(base_output_dir)

    try:
        rclpy.spin(node)
    except (rclpy.executors.ExternalShutdownException, KeyboardInterrupt):
        if rclpy.ok():  
            node.get_logger().info("Shutdown detected, exiting gracefully.")
    finally:
        # Save timestamps before shutting down
        node.save_timestamps()

        # Shutdown ROS before destroying the node
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()

        # Use print() for the final confirmation message (not ROS logging)
        print(f"✅ Saved timestamps to {os.path.join(base_output_dir, 'output_pointclouds/timestamps.mat')}")
        print("✅ PointCloudExtractor has exited cleanly.")

if __name__ == '__main__':
    main()


