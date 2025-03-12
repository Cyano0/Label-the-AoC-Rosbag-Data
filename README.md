# Label the AoC Rosbag Data

## Overview

This repository contains scripts for extracting images and point clouds from ROS2 bag files for data labeling. The pipeline involves converting ROS2 bag files into images and point clouds, which are then stored in labeled directories for further processing using MATLAB.

## Prerequisites

Ensure you have the following dependencies installed:

### Software Requirements:

- [MATLAB](https://www.mathworks.com/products/matlab.html)


## Usage

### Step 1: Check Teams for the list of tasks.

Go to Team on Teams, and find your own channel. There is an Excel file (list\_of\_bags) lists all the bagfile names for you to check.
![image](https://github.com/user-attachments/assets/a8d014a7-957e-4fa1-b061-bdc2783dae61)


### Step 2: Download the zip file to your own computer, and extract it.

The bag files are under **Files**.
![image](https://github.com/user-attachments/assets/f91a8576-cc92-4918-bc05-c1f230ac8912)

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
4. Save the JSON files for all resources (cameras and LiDar) together in a new directory called **json\_files**. You can see something like:
![image](https://github.com/user-attachments/assets/116bf385-812e-4234-94b0-86290d65adf8)

This JSON file will contain timestamps, filenames, and bounding box annotations for each labeled image and pcd file.


### Step 5: Rename and upload JSON files to Teams 

1. **Rename** the json_files folder to {rosbagname}\_json\_files to help distinguish with different json files. E.g. in\_straw\_2pick\_diff\_st\_10\_31\_2024\_1\_label\_json\_files.
2. Upload it to a folder called JsonFiles in your channel.
3. Update information in the Excel file lists all the bagfile names in the channel:
  (1) Status: choose the status of the bag here (completed? In progress?)
  (2) Time when completed (date): put a date here
  (3) How long it took: put the time length you used for labelling.
  (4) Other notes: optional, anything you want to note down.

![image](https://github.com/user-attachments/assets/6d75ec0e-70e3-49f5-811b-e466995691fc)


## Notes

- Ensure Matlab is installed correctly.
- MATLAB-compatible timestamps (`timestamps.mat`) help synchronize images and point clouds.

## License

This project is licensed under the MIT License.

