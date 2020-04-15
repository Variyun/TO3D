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
clear all
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

% for i = 1:length(images)-1
%     fprintf('Working on image pair %s & $s .....', images(i).name, images(i+1).name);
%     impath1 = strcat(images(i).folder,'\', images(i).name); 
%     impath2 = strcat(images(i+1).folder,'\', images(i+1).name); 
%     disparity(impath1, impath2, images(i).name, images(i+1).name);
%     fprintf('DONE\n');
% end

% TESTING TESTING TESTING

path1 = 'StereoImages\DSC05899.JPG';
path2 = 'StereoImages\DSC05900.JPG';
fID = fopen('PointCloud.txt', 'a');
disparity(path1, path2, 'DSC05899.JPG', 'DSC05900.JPG', fID);
% path1 = 'DSC05644.JPG';
% path2 = 'DSC05645.JPG';
% disparity(path1, path2, 'DSC05644.JPG', 'DSC05645.JPG');
fprintf('DONE\n');
fclose('all');
% End of Program 
% fprintf('\nProgram finished, press any key to exit.\n');
% pause;
return 

%% Undistort Images
function disparity(path1, path2, name1, name2, fID)
clc
close all

% Load in image pairs
try
    left = imread(path1);
catch
    fprintf("Couldn't open image %s.\nPress any key to close.\n", name1);
    pause;
    return
end
try
    right = imread(path2);
catch
    fprintf("Couldn't open image %s.\nPress any key to close.\n", name2);
    pause;
    return
end

% Define camera Intrinsic Matrix and Radial Distortion Parameters
inMatrix=[4066.861 0 0;0 4066.861 0; 2978.91370054375 2030.83585703852 1];
radDist=[-0.062 0.083 0.014];

cameraParams = cameraParameters('IntrinsicMatrix', inMatrix, ...
    'RadialDistortion', radDist, ...
    'NumRadialDistortionCoefficients', 3);

% Undistort Image
% NOTE - Images being resized to reduce computation time and machine stress
left = imresize(left, 0.25);
right = imresize(right, 0.25); 
[left_undistorted, ~] = undistortImage(left, cameraParams);
[right_undistorted, ~] = undistortImage(right, cameraParams);

% Feature Detection
left_points = detectSURFFeatures(rgb2gray(left_undistorted));
right_points = detectSURFFeatures(rgb2gray(right_undistorted));

% Extract Features (currently only the strongest 1000)
[left_features, left_coords] = extractFeatures(rgb2gray(left_undistorted),left_points.selectStrongest(500));
[right_features, right_coords] = extractFeatures(rgb2gray(right_undistorted),right_points.selectStrongest(500));

% Matching Features
indexPairs = matchFeatures(left_features, right_features);

left_match = double(left_coords.Location(indexPairs(:,1),1:2));
right_match = double(right_coords.Location(indexPairs(:,2),1:2));

% Compute fundamental matrix and get inliers for true matches 
% [f, indexPairs] = estimateFundamentalMatrix([left_match(:,2) left_match(:,1)], ...
%     [right_match(:,2) right_match(:,1)], 'Method', 'RANSAC');
[f, indexPairs] = estimateFundamentalMatrix(left_match, right_match, 'Method', 'RANSAC');

left_true = left_match(indexPairs, 1:2); 
right_true = right_match(indexPairs, 1:2); 

% clearvars -except left_true right_true f name1 name2 ...
%     left_undistorted right_undistorted cameraParams

% Calculate stereo parameters
[R, t] = cameraPose(f, cameraParams, left_true, right_true);

clearvars -except R t cameraParams left_undistorted right_undistorted fID
  
% Get stereoParameters
stereoParams = stereoParameters(cameraParams, cameraParams, R, t);
%Rectify images using stereo parameters
[left_rect, right_rect] = rectifyStereoImages(left_undistorted, right_undistorted, stereoParams, 'OutputView','full');

% Show stereo anaglyph 
% stereo = stereoAnaglyph(left_rect, right_rect);
% figure;
% imshow(stereo);

% Compute disparity map
disparityRange = [0 128];
disparityMap = disparitySGM(rgb2gray(left_rect),rgb2gray(right_rect), ...
     'DisparityRange',disparityRange, 'UniquenessThreshold', 0);
 
% -- UNCOMMENT BELOW TO SAVE IMAGES AND DISPLAY DISPARITY MAP --
% figure
% imshow(disparityMap,disparityRange)
% title('Disparity Map')
% colormap jet
% colorbar

% name1 = strcat('Rectified\RECT_', name1);  
% name2 = strcat('Rectified\RECT_', name2);  
% 
% imwrite(left_rect, name1);
% imwrite(right_rect, name2);

% DisparitySGM very hard on computer for large images - BE CAREFUL
% imwrite(disparitySGM(rgb2gray(left_rect),rgb2gray(right_rect), ...
%      'DisparityRange',disparityRange, 'UniquenessThreshold', 0),...
%      'Disparity\DISP_Pair.jpg');

xyzPoints = reconstructScene(disparityMap,stereoParams);

clearvars -except xyzPoints fID


X = xyzPoints(:,:,1);
Y = xyzPoints(:,:,2);
Z = xyzPoints(:,:,3);

X = reshape(X,[],1);
Y = reshape(Y,[],1);
Z = reshape(Z,[],1);

index = X >= -7000 & X <= 7000;

X = X(index);
Y = Y(index);
Z = Z(index);

fprintf(fID, '%f %f %f\n', [X,Y,Z].');
end


