%% TO3D Application Developed by Aaron Chen and Chris Lin
% Developed for ENGO 559 Digital Imaging Winter 2020
% 1. Use the camera matrix and the distortion parameters to undistort the images 
% 2. Run feature matching with estimation of the fundamental matrix and the rectification parameters
% 3. Find the fundamental matrix and the rectification parameters
% 4. Compute the disparity map of a rectified image pair

% Version 1.Something.2 Changelog
% Implement above for SINGLE pair of images
% Completed up to step 2

%% Main 
clc
% ***** IMPORTANT: UNCOMMENT SECTION BELOW FOR PRODUCTION VERSION *****
% User must enter in the directory containing images and file extension
% fprintf('Welcome to TO3D!\n')
% dir_in = input('Please enter name of directory with images: ', 's');
% type = input('Please enter the file format of your images (ex. png, jpg, tiff): ', 's');
% dir_in = strcat(dir_in, '/*.', type);
% % read directory
% images = dir(dir_in);
% % if nothing is read in, error message
% if isempty(images)
%     fprintf('Error reading files in directory: %s\n', dir_in);
%     fprintf('Press any key to exit.\n');
%     pause;
%     return
% end
%
% for i = 1: length(images)
%     impath = strcat(images(i).folder,'/', images(i).name); 
%     a = imread(impath); 
%     feat = detectHarrisFeatures(rgb2gray(a));
%     plot_feat(a, feat);
% end

%% Undistort Images
clc

% Load in image pairs
left = imread('TestStereo/DSC05899.JPG');
right = imread('TestStereo/DSC05900.JPG');

% Define camera Intrinsic Matrix and Radial Distortion Parameters
inMatrix=[4066.861 0 0;0 4066.861 0; 2978.91370054375 2030.83585703852 1];
radDist=[-0.062 0.083 0.014];

cameraParams = cameraParameters('IntrinsicMatrix', inMatrix, ...
    'RadialDistortion', radDist, ...
    'NumRadialDistortionCoefficients', 3);

% Undistort Image
fprintf('Undistorting Images...')
[left_undistorted, left_origin] = undistortImage(left, cameraParams);
[right_undistorted, right_origin] = undistortImage(right, cameraParams);
fprintf('DONE\n')

% Display undistorted image for testing
% imshowpair(left, left_undistorted, 'montage')
% figure 
% imshowpair(right, right_undistorted, 'montage')

% Feature Detection
fprintf('Detecting Features...')
left_points = detectSURFFeatures(rgb2gray(left_undistorted));
right_points = detectSURFFeatures(rgb2gray(right_undistorted));
fprintf('DONE\n')

% Extract Features (currently only the strongest 1000)
fprintf('Extracting Features...')
[left_features, left_coords] = extractFeatures(rgb2gray(left_undistorted),left_points.selectStrongest(1000));
[right_features, right_coords] = extractFeatures(rgb2gray(right_undistorted),right_points.selectStrongest(1000));
fprintf('DONE\n')

% Matching Features
fprintf('Matching Features...')
indexPairs = matchFeatures(left_features, right_features);
left_match = left_coords.Location(indexPairs(:,1),1:2);
right_match = right_coords.Location(indexPairs(:,2),1:2);
% Use RANSAC to find true matches
[H, inliers] = ransacfithomography(left_match', right_match', 0.01);
fprintf('DONE\n')

% End of Program 
fprintf('\nProgram finished, press any key to exit.\n');
pause;
return 


%% Functions


