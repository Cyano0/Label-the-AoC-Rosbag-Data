cd '/home/zhuoling/Documents/MATLAB_label/in_straw_2pick_diff_st_10_31_2024_1_label'

fish12data = load('fisheye_images_12/timestamps.mat'); 
fish12timestamps = fish12data.timestamps; % Extract Unix timestamps
% Convert Unix time (seconds) to MATLAB duration format
fish12timestamps = seconds(fish12timestamps);

fish13data = load('fisheye_images_13/timestamps.mat'); 
fish13timestamps = fish13data.timestamps; % Extract Unix timestamps
% Convert Unix time (seconds) to MATLAB duration format
fish13timestamps = seconds(fish13timestamps);

fish14data = load('fisheye_images_14/timestamps.mat'); 
fish14timestamps = fish14data.timestamps; % Extract Unix timestamps
% Convert Unix time (seconds) to MATLAB duration format
fish14timestamps = seconds(fish14timestamps);

rgbdata = load('output_images/timestamps.mat'); 
rgbtimestamps = rgbdata.timestamps; % Extract Unix timestamps
% Convert Unix time (seconds) to MATLAB duration format
rgbtimestamps = seconds(rgbtimestamps);

pcddata = load('output_pointclouds/timestamps.mat'); 
pcdtimestamps = pcddata.timestamps; % Extract Unix timestamps
% Convert Unix time (seconds) to MATLAB duration format
pcdtimestamps = seconds(pcdtimestamps);