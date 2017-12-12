%%
clear all
close all
clc
%% 
% Insert the object1 image 
eraserImage = imread('eraser.jpg');
eraserImage = rgb2gray(eraserImage);
% Show th gray object1 image
figure;
imshow(eraserImage);
title('Image of a eraser');
%%
% Insert the scene image.
sceneImage = imread('scene3.jpg');
sceneImage = rgb2gray(sceneImage);
% Show the gray scene.
figure;
imshow(sceneImage);
title('Image of a Scene');
%% 
% Detect the features in the object and scene image.
eraserPoints = detectSURFFeatures(eraserImage);
scenePoints = detectSURFFeatures(sceneImage);
%% 
% Visualize the strongest feature points found in the reference image.
figure; 
imshow(eraserImage);
title('100 Strongest Feature Points from eraser Image');
hold on;
plot(selectStrongest(eraserPoints, 100));
%%
% Visualize the strongest feature points found in the scene image.
figure;
imshow(sceneImage);
title('300 Strongest Feature Points from Scene Image');
hold on;
plot(selectStrongest(scenePoints, 300));
%% 
% Extract feature descriptors at the interest points in both images.
[eraserFeatures, eraserPoints] = extractFeatures(eraserImage, eraserPoints);
[sceneFeatures, scenePoints] = extractFeatures(sceneImage, scenePoints);
%% 
% Match the features using their descriptors. 
eraserPairs = matchFeatures(eraserFeatures, sceneFeatures);
%% 
% Display putatively matched features. 
matchederaserPoints = eraserPoints(eraserPairs(:, 1), :);
matchedScenePoints = scenePoints(eraserPairs(:, 2), :);
figure;
showMatchedFeatures(eraserImage, sceneImage, matchederaserPoints, ...
    matchedScenePoints, 'montage');
title('Putatively Matched Points (Including Outliers)');
%% 
% Calculates the transformation relating the matched points, while 
% eliminating outliers. This transformation allows us to localize the 
% object in the scene.
[tform, inliereraserPoints, inlierScenePoints] = ...
    estimateGeometricTransform(matchederaserPoints, matchedScenePoints, 'affine');
%%
% Display the matching point pairs with the outliers removed
figure;
showMatchedFeatures(eraserImage, sceneImage, inliereraserPoints, ...
    inlierScenePoints, 'montage');
title('Matched Points (Inliers Only)');
%% 
% Get the bounding polygon of the reference image.
eraserPolygon = [1, 1;...                           % top-left
        size(eraserImage, 2), 1;...                 % top-right
        size(eraserImage, 2), size(eraserImage, 1);... % bottom-right
        1, size(eraserImage, 1);...                 % bottom-left
        1, 1];                   % top-left again to close the polygon
% Transform the polygon into the coordinate system of the target image.
% The transformed polygon indicates the location of the object in the
% scene.
neweraserPolygon = transformPointsForward(tform, eraserPolygon);   
% Display the detected object.
% figure;
% imshow(sceneImage);
% hold on;
% line(neweraserPolygon(:, 1), neweraserPolygon(:, 2), 'Color', 'y');
% title('Detected eraser');
%%
% Insert the object2 image.
ADImage = imread('AD.jpg');
ADImage = rgb2gray(ADImage);
% Show th gray object2 image.
figure;
imshow(ADImage);
title('Image of an AD');
%% 
% Detect and visualize feature points.
ADPoints = detectSURFFeatures(ADImage);
figure;
imshow(ADImage);
hold on;
plot(selectStrongest(ADPoints, 100));
title('100 Strongest Feature Points from AD Image');
%%
% Extract feature descriptors.
[AD, ADPoints] = extractFeatures(ADImage, ADPoints);
%% 
% Match Features
ADPairs = matchFeatures(AD, sceneFeatures, 'MaxRatio', 0.9);
%%
% Display putatively matched features.
matchedADPoints = ADPoints(ADPairs(:, 1), :);
matchedScenePoints = scenePoints(ADPairs(:, 2), :);
figure;
showMatchedFeatures(ADImage, sceneImage, matchedADPoints, ...
    matchedScenePoints, 'montage');
title('Putatively Matched Points (Including Outliers)');
%%
%Estimate Geometric Transformation and Eliminate Outliers
[tform, inlierADPoints, inlierScenePoints] = ...
    estimateGeometricTransform(matchedADPoints, matchedScenePoints, 'affine');
figure;
% Display putatively matched features. 
showMatchedFeatures(ADImage, sceneImage, inlierADPoints, ...
    inlierScenePoints, 'montage');
title('Matched Points (Inliers Only)');

%% 
% Get the bounding polygon of the reference image.
ADPolygon = [1, 1;...                                 % top-left
        size(ADImage, 2), 1;...                       % top-right
        size(ADImage, 2), size(ADImage, 1);...  % bottom-right
        1, size(ADImage, 1);...                       % bottom-left
        1,1];                         % top-left again to close the polygon
 
newADPolygon = transformPointsForward(tform, ADPolygon);    
%%
% Display Both Objects
figure;
imshow(sceneImage);
hold on;
line(neweraserPolygon(:, 1), neweraserPolygon(:, 2), 'Color', 'y');
line(newADPolygon(:, 1), newADPolygon(:, 2), 'Color', 'g');
title('Detected AD and eraser');
