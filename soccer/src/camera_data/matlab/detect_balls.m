load('../training_images/ball/ball.mat');
positiveInstances = ball( :, 1 : 2);

imDir = '../training_images/ball/';
negativeFolder = '../training_images/negative/';

addpath(imDir);
negativeImages = imageDatastore(negativeFolder);

trainCascadeObjectDetector('ball.xml', positiveInstances, negativeFolder, 'FalseAlarmRate', 0.01, 'NumCascadeStages', 4, 'FeatureType', 'Haar');

movefile('ball.xml', '../cascades/ball.xml')

detector = vision.CascadeObjectDetector('../cascades/ball.xml');
img = imread('test/ball5.jpg');
bbox = step(detector, img);
detectedImg = insertObjectAnnotation(img, 'rectangle', bbox, 'ball');
figure;
imshow(detectedImg);
rmpath(imDir); 