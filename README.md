# Label the AoC Rosbag Data

# Data Labeling Pipeline

## Overview

This repository contains scripts for extracting images and point clouds from ROS2 bag files for data labeling. The pipeline involves converting ROS2 bag files into images and point clouds, which are then stored in labeled directories for further processing.

## Prerequisites

Ensure you have the following dependencies installed:

### Software Requirements:

- [ROS2 (Foxy or later, Humble preferred)](https://docs.ros.org/en/humble/Installation.html)
- [Python 3.8+](https://www.python.org/downloads/)
- [MATLAB](https://www.mathworks.com/products/matlab.html)

### Python Packages:

- OpenCV: `pip install opencv-python`
- NumPy: `pip install numpy`
- SciPy: `pip install scipy`
- Open3D: `pip install open3d`
- SciPy IO: `pip install scipy`

## Usage

### Step 1: Run the ROS2 Bag Processing Pipeline

This script automates the extraction of images and point clouds from a given ROS2 bag file.

#### Command:

```bash
python create_file_for_label.py <bagfile_name>
```

#### Example:

```bash
python rcreate_file_for_label.py 1_2_3
```

This will:

1. Extract images from the bag file into a labeled directory.
2. Extract point clouds into another labeled directory.
3. Play the ROS2 bag file at a reduced speed (`0.2x`).
4. Save timestamps in MATLAB-compatible `.mat` format.

### Step 2: Extract Images from ROS Topics

The `image_extractor.py` script subscribes to multiple camera topics, extracts images, and stores them with corresponding timestamps.

#### Command:

```bash
python image_extractor.py <output_folder>
```

#### Example:

```bash
python image_extractor.py dataset_output
```

#### Image Topics:

- `/front_camera/image_raw` → `output_images/`
- `/fisheye_image_SN00013` → `fisheye_images_13/`
- `/fisheye_image_SN00012` → `fisheye_images_12/`
- `/fisheye_image_SN00014` → `fisheye_images_14/`

Each extracted image is saved with a timestamped filename, and timestamps are stored in `timestamps.mat`.

### Step 3: Extract Point Clouds from ROS Topics

The `pointcloud_extractor.py` script subscribes to a LiDAR topic and extracts point clouds.

#### Command:

```bash
python pointcloud_extractor.py <output_folder>
```

#### Example:

```bash
python pointcloud_extractor.py dataset_output
```

#### Point Cloud Topic:

- `/front_lidar/points` → `output_pointclouds/`

Each point cloud is saved in `.pcd` format, and timestamps are stored in `timestamps.mat`.

### Step 4: Import Data into MATLAB and Label Using Ground Truth Labeller

After extracting the images and point clouds, the next step is to import them into MATLAB and use the Ground Truth Labeller tool for annotation.

1. Open MATLAB and navigate to the dataset directory.
2. Load the extracted timestamps for synchronization.
3. Use the Ground Truth Labeller tool to annotate objects of interest in the images.
4. Ensure that the labels are consistent across different viewpoints and sensors.

For a step-by-step guide, refer to [this tutorial](video_link).

### Step 5: Export Labels to JSON

Once labeling is complete, the annotations need to be exported in JSON format for further processing using gTruth\_to\_json.m. It willl do:

1. Extract available label definitions from the MATLAB workspace.
2. Load corresponding timestamps for each labeled frame.
3. Format the labeled data into a structured JSON format.
4. Save the JSON file in the dataset directory.

This JSON file will contain timestamps, filenames, and bounding box annotations for each labeled image and pcd file.

## Shutdown and Cleanup

If you need to stop the scripts:

1. Use `Ctrl+C` to terminate running scripts.
2. Ensure timestamps are saved before shutdown.

## Notes

- Ensure the ROS2 environment is correctly sourced before running scripts.
- MATLAB-compatible timestamps (`timestamps.mat`) help synchronize images and point clouds.

## License

This project is licensed under the MIT License.

