%%
clear all
close all
clc
%%
% Insert the object image.
objectImage = imread('idcard.jpg');
objectImage = rgb2gray(objectImage);
% Show th gray object image.
figure;
imshow(objectImage);
title('Image of a eraser');
%%
% Insert the scene image.
sceneImage = imread('scene4.jpg');
sceneImage = rgb2gray(sceneImage);
% Show the gray scene.
figure;
imshow(sceneImage);
title('Image of a Scene');
%% 
% Detect the features in the object and scene image.
objectPoints = detectSURFFeatures(objectImage);
scenePoints = detectSURFFeatures(sceneImage);
%% 
% Visualize the strongest feature points found in the object image.
figure; 
imshow(objectImage);
title('100 Strongest Feature Points from object Image');
hold on;
plot(selectStrongest(objectPoints, 100));
hold off
%%
% Visualize the strongest feature points found in the scene image.
figure;
imshow(sceneImage);
title('300 Strongest Feature Points from Scene Image');
hold on;
plot(selectStrongest(scenePoints, 300));
hold off
%% 
% Extract feature descriptors at the interest points in both images.
[objectFeatures, objectPoints] = extractFeatures(objectImage, objectPoints);
[sceneFeatures, scenePoints] = extractFeatures(sceneImage, scenePoints);
%% 
% Match the features using their descriptors. 
objectPairs = matchFeatures(objectFeatures, sceneFeatures);
%% 
% Display putatively matched features. 
matchedobjectPoints = objectPoints(objectPairs(:, 1), :);
matchedScenePoints = scenePoints(objectPairs(:, 2), :);
figure;
showMatchedFeatures(objectImage, sceneImage, matchedobjectPoints, ...
    matchedScenePoints, 'montage');
title('Putatively Matched Points (Including Outliers)');
%% Step 5: Locate the Object in the Scene Using Putative Matches
% Calculates the transformation relating the matched points, while 
% eliminating outliers. This transformation allows us to localize the 
% object in the scene.
[tform, inlierobjectPoints, inlierScenePoints] = ...
    estimateGeometricTransform(matchedobjectPoints, matchedScenePoints, 'affine');
%%
% Display the matching point pairs with the outliers removed
figure;
showMatchedFeatures(objectImage, sceneImage, inlierobjectPoints, ...
    inlierScenePoints, 'montage');
title('Matched Points (Inliers Only)');
%% 
% Get the bounding polygon of the reference image.
objectPolygon = [1, 1;...                           % top-left
        size(objectImage, 2), 1;...                 % top-right
        size(objectImage, 2), size(objectImage, 1);... % bottom-right
        1, size(objectImage, 1);...                 % bottom-left
        1, 1];                   % top-left again to close the polygon
% Transform the polygon into the coordinate system of the target image.
% The transformed polygon indicates the location of the object in the
% scene.
newobjectPolygon = transformPointsForward(tform, objectPolygon);   
%%
% Display the result of the object identification.
figure;
imshow(sceneImage);
hold on;
line(newobjectPolygon(:, 1), newobjectPolygon(:, 2), 'Color', 'y');
title('Detected AD and eraser');
hold off
%%
% We can see that the identification is wrong since it captures the wrong 
% object. We think the differences between the two cards are not enough
% for this method to distinguish these two.
