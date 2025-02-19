import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from rclpy.qos import QoSProfile, ReliabilityPolicy
import cv2
import os
import sys
import numpy as np
from scipy.io import savemat

class ImageExtractor(Node):
    def __init__(self, base_output_dir):
        super().__init__('image_extractor')
        self.bridge = CvBridge()
        
        # Define topics and output directories under base_output_dir
        self.image_topics = {
            '/front_camera/image_raw': os.path.join(base_output_dir, 'output_images'),
            '/fisheye_image_SN00013': os.path.join(base_output_dir, 'fisheye_images_13'),
            '/fisheye_image_SN00012': os.path.join(base_output_dir, 'fisheye_images_12'),
            '/fisheye_image_SN00014': os.path.join(base_output_dir, 'fisheye_images_14')
        }

        # Create directories and initialize timestamp dictionaries
        self.timestamps = {topic: [] for topic in self.image_topics}
        for folder in self.image_topics.values():
            os.makedirs(folder, exist_ok=True)

        # Set QoS profile for best-effort reliability
        qos_profile = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, depth=10)

        # Create subscribers for each image topic
        for topic, folder in self.image_topics.items():
            self.create_subscription(Image, topic, lambda msg, t=topic: self.image_callback(msg, t), qos_profile)

    def image_callback(self, msg, topic):
        try:
            output_dir = self.image_topics[topic]  # Get corresponding folder
            
            # Convert ROS image message to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # Extract timestamps
            timestamp_sec = msg.header.stamp.sec
            timestamp_nanosec = msg.header.stamp.nanosec
            timestamp = timestamp_sec + timestamp_nanosec * 1e-9  # Convert to seconds

            # Generate timestamp-based filename
            timestamp_str = f"{timestamp_sec}_{timestamp_nanosec:09d}"
            file_name = os.path.join(output_dir, f"{timestamp_str}.png")

            # Save image
            cv2.imwrite(file_name, cv_image)
            self.get_logger().info(f"Saved image {file_name}")

            # Store timestamp in duration format (MATLAB expects seconds as duration)
            self.timestamps[topic].append(timestamp)

        except Exception as e:
            self.get_logger().error(f"Error processing image from {topic}: {e}")

    def save_timestamps(self):
        """ Saves timestamps as a MATLAB-compatible .mat file in `duration` format """
        try:
            for topic, timestamps in self.timestamps.items():
                output_dir = self.image_topics[topic]
                timestamp_file = os.path.join(output_dir, "timestamps.mat")
                timestamps_np = np.array(timestamps).reshape(-1, 1)
                savemat(timestamp_file, {"timestamps": timestamps_np})

                # ✅ Use print() instead of self.get_logger().info()
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

    node = ImageExtractor(base_output_dir)

    try:
        rclpy.spin(node)
    except (rclpy.executors.ExternalShutdownException, KeyboardInterrupt):
        if rclpy.ok():  
            node.get_logger().info("Shutdown detected, exiting gracefully.")
    finally:
        # ✅ Save timestamps before shutting down
        node.save_timestamps()

        # ✅ Shutdown ROS before destroying the node
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()

        # ✅ Use print() for the final confirmation message (not ROS logging)
        print(f"✅ Saved timestamps to {os.path.join(base_output_dir, 'output_images/timestamps.mat')}")
        print(f"✅ Saved timestamps to {os.path.join(base_output_dir, 'fisheye_images_13/timestamps.mat')}")
        print(f"✅ Saved timestamps to {os.path.join(base_output_dir, 'fisheye_images_12/timestamps.mat')}")
        print(f"✅ Saved timestamps to {os.path.join(base_output_dir, 'fisheye_images_14/timestamps.mat')}")
        print("✅ ImageExtractor has exited cleanly.")

if __name__ == '__main__':
    main()


