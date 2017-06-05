%% Training Soccer Ball detection
% Load the positive samples data from a MAT file. The file contains
% a table specifying bounding boxes for several object categories.
% The table was exported from the Training Image Labeler app.
%%
% Load positive samples.
load('../training_images/ball/positive/ball_positive_table.mat');
%%
% Select the bounding boxes for stop signs from the table.
positiveInstances = ball_table(:,1:2);
%%
% Add the image directory to the MATLAB path.
imDir = '../training_images/ball/positive/';
addpath(imDir);
%%
% Specify the folder for negative images.
negativeFolder = '../training_images/ball/negative/';
%%
% Create an |imageDatastore| object containing negative images.
negativeImages = imageDatastore(negativeFolder);
%%
% Train a cascade object detector called 'stopSignDetector.xml'
% using HOG features.
% NOTE: The command can take several minutes to run.
trainCascadeObjectDetector('ball.xml',positiveInstances, ...
    negativeFolder,'FalseAlarmRate',0.05,'NumCascadeStages',4);
movefile('ball.xml', '../cascades/ball.xml')
%%
% Use the newly trained classifier to detect a stop sign in an image.
detector = vision.CascadeObjectDetector('../cascades/ball.xml');
%%
% Read the test image.
img = imread('ball4.jpg');
%%
% Detect a stop sign.
bbox = step(detector,img); 
%%
% Insert bounding box rectangles and return the marked image.
 detectedImg = insertObjectAnnotation(img,'rectangle',bbox,'ball');
%%
% Display the detected stop sign.
figure; imshow(detectedImg);
%%
% Remove the image directory from the path.
rmpath(imDir); 