# Label the AoC Rosbag Data

## Overview

This repository contains scripts for extracting images and point clouds from ROS2 bag files for data labeling. The pipeline involves converting ROS2 bag files into images and point clouds, which are then stored in labeled directories for further processing using MATLAB.

## Prerequisites

Ensure you have the following dependencies installed:

### Software Requirements:

- [MATLAB](https://www.mathworks.com/products/matlab.html)


## Usage

### Step 1: Check Teams for the list of tasks.

Go to Team on Teams, and find your own channel. There is an Excel file for you to check.

### Step 2: Download the zip file to your own computer, and extract it.


### Step 3: Import Data into MATLAB and Label Using Ground Truth Labeller

After extracting the images and point clouds, the next step is to import them into MATLAB and use the Ground Truth Labeller tool for annotation.

1. Open MATLAB and navigate to the dataset directory.
2. Load the extracted timestamps for synchronization.
3. Use the Ground Truth Labeller tool to annotate objects of interest in the images.
4. Ensure that the labels are consistent across different viewpoints and sensors.

For a step-by-step guide, refer to [this tutorial](https://universityoflincoln-my.sharepoint.com/:v:/g/personal/zhuang_lincoln_ac_uk/Ed-UfqKa-zRCorqXSayXZ-gBDXAZqGJH1J2AqO5NizRhAA?e=58oTOc&nav=eyJyZWZlcnJhbEluZm8iOnsicmVmZXJyYWxBcHAiOiJTdHJlYW1XZWJBcHAiLCJyZWZlcnJhbFZpZXciOiJTaGFyZURpYWxvZy1MaW5rIiwicmVmZXJyYWxBcHBQbGF0Zm9ybSI6IldlYiIsInJlZmVycmFsTW9kZSI6InZpZXcifX0%3D).

### Step 4: Export Labels to JSON

Once labeling is complete, the annotations need to be exported in JSON format for further processing using gTruth\_to\_json.m. It willl do:

1. Extract available label definitions from the MATLAB workspace.
2. Load corresponding timestamps for each labeled frame.
3. Format the labeled data into a structured JSON format.
4. Save the JSON file in a new directory.

This JSON file will contain timestamps, filenames, and bounding box annotations for each labeled image and pcd file.



## Notes

- Ensure Matlab is installed correctly.
- MATLAB-compatible timestamps (`timestamps.mat`) help synchronize images and point clouds.

## License

This project is licensed under the MIT License.

