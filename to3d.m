%% TO3D Application Developed by Aaron Chen and Chris Lin
% Developed for ENGO 559 Digital Imaging Winter 2020
clc
% User must enter in the directory containing images and file extension
fprintf('Welcome to TO3D!\n')
dir_in = input('Please enter name of directory with images: ', 's');
type = input('Please enter the file format of your images (ex. png, jpg, tiff): ', 's');
dir_in = strcat(dir_in, '/*.', type);
% read directory
images = dir(dir_in);
% if nothing is read in, error message
if isempty(images)
    fprintf('Error reading files in directory: %s\n', dir_in);
    fprintf('Press any key to exit.\n');
    pause;
    return
end

for i = 1: length(images)
    impath = strcat(images(i).folder,'/', images(i).name); 
    a = imread(impath); 
    feat = detectHarrisFeatures(rgb2gray(a));
    plot_feat(a, feat);
end

fprintf('\nProgram finished, press any key to exit.\n');
pause;
return 

%% Function Definitions

% Function to plot images with strongest 50 features 
function plot_feat(I, feature)
    figure
    imshow(I)
    hold on
    plot(feature.selectStrongest(50));
    hold off
end