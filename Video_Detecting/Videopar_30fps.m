%% 
clear  
clc
close all
%%
% Insert the object image.
objectImage = imread('eraser.jpg');
% Detect the object features 
objectImageg = rgb2gray(objectImage);
objectPoints = detectSURFFeatures(objectImageg);
%% 
% Displaying video.
videoFileReader = vision.VideoFileReader('test30.mp4');
videoPlayer = vision.VideoPlayer('Position',[100,100,680,520]);
%% 
% To see how many frames are in this video. 
VideoObject = VideoReader('test30.mp4');
frameNum = VideoObject.NumberofFrames;
% VideoObject.FrameRate = 15;
% Try to set the fps on iphone from 30 to 60.
% See whether we can got a better result.
%% 
% Read the first video frame to see whether it contains the object.
% (This part is not necessary.)
objectFrame = step(videoFileReader);
%% 
% If we do not insert the object image, instead wanting to choose the
% object in the video, we can use the following commands to select the 
% object region using a mouse. 

% figure; imshow(objectFrame);
% objectRegion=round(getPosition(imrect))

%%
% Getting the frame of the very beginning in the video.
sceneImage = objectFrame;
% Extracting the features in this scene.
sceneImage = rgb2gray(sceneImage);
scenePoints = detectSURFFeatures(sceneImage);
%%
% Matching the features form the object and the scene.
[points,tform,inlierobjecteraserPoints,status,status2] = matchingfeatures(objectImageg,objectPoints,sceneImage,scenePoints);
%% 
% Find the object polygon that block the object.
[Polygon] = objectPolygon(objectImage);
neweraserPolygon = transformPointsForward(tform, Polygon);   
%% 
% Show the initial frame of the video with a polygon block the object.
figure;
imshow(objectFrame);
hold on
% Draw out the polygon
line(neweraserPolygon(:, 1), neweraserPolygon(:, 2), 'Color', 'y');
hold off
%% 
% Create a tracker object.
tracker = vision.PointTracker('MaxBidirectionalError',1);
%% 
% Initialize the tracker.
initialize(tracker,points.Location,objectFrame);
%% 
% Read, track, display points, and results in each video frame.
while ~isDone(videoFileReader)
        frame = step(videoFileReader);

%       Getting frames of each step in the video and extract the features.
        sceneImage = frame;
        sceneImage = rgb2gray(sceneImage);
        scenePoints = detectSURFFeatures(sceneImage);
%       Find the matching features between objects and frames
        [points,tform,inlierobjecteraserPoints,status,status2] = matchingfeatures(objectImageg,objectPoints,sceneImage,scenePoints);
%       If the amount of the matching features is large enough(here we pick > 5),
%       find the polygon to block the object.        
        if status2 == true
            [Polygon] = objectPolygon(objectImageg);
            neweraserPolygon = transformPointsForward(tform, Polygon);   
            neweraserPolygon = [neweraserPolygon(1:4,:),[neweraserPolygon(2:4,:);neweraserPolygon(1,:)]];
%           Show the polygon in the video.
            out = insertShape(frame, 'Line', neweraserPolygon, 'LineWidth', 5);
            step(videoPlayer,out);
        else
%           If not, do nothing about the polygon.
            a = [0 0 0 0;0 0 0 0;0 0 0 0;0 0 0 0];
            out = [insertShape(frame, 'Line', a, 'LineWidth', 5);];
            step(videoPlayer,out)
        end       
end
%% 
% Release the video reader and player.
release(videoPlayer);
release(videoFileReader);