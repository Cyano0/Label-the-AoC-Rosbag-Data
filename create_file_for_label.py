import subprocess
import time
import sys
import os
import signal

def run_command(command):
    """Runs a command as a subprocess and returns the process."""
    return subprocess.Popen(command, shell=True, preexec_fn=os.setsid)

def main():
    # Step 1: Get the bagfile name from the user
    if len(sys.argv) < 2:
        print("Usage: python run_rosbag_pipeline.py <bagfile_name>")
        sys.exit(1)

    bagfile_name = sys.argv[1]  # Example: 1_2_3
    label_name = f"{bagfile_name}_label"  # Example: 1_2_3_label

    print(f"Running pipeline for: {bagfile_name}")
    print(f"Using label directory: {label_name}")

    # Step 2: Run the first script (rosbag_to_all_images.py)
    print("Starting rosbag_to_all_images...")
    image_process = run_command(f"python rosbag_to_all_images.py {label_name}")

    # Step 3: Run the second script (rosbag_to_pcd.py)
    print("Starting rosbag_to_pcd...")
    pcd_process = run_command(f"python rosbag_to_pcd.py {label_name}")

    time.sleep(3)

    # Step 4: Run ros2 bag play
    print(f"Playing ROS2 bag: {bagfile_name}")
    bag_process = run_command(f"ros2 bag play {bagfile_name} --rate 0.2")

    # Step 5: Wait for `ros2 bag play` to finish
    bag_process.wait()
    
    # Step 6: Wait 1 second before stopping the extraction scripts
    print("Bag finished. Waiting 1 second before stopping image and PCD extraction...")
    time.sleep(3)

    # Step 7: Kill both Python processes (image and PCD extraction)
    print("Stopping rosbag_to_all_images...")
    os.killpg(os.getpgid(image_process.pid), signal.SIGTERM)  # Terminate group

    print("Stopping rosbag_to_pcd...")
    os.killpg(os.getpgid(pcd_process.pid), signal.SIGTERM)  # Terminate group

    print("All processes stopped. Pipeline complete.")

if __name__ == "__main__":
    main()

