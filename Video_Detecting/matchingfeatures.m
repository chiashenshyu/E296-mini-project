% Matching the features form the object and the scene.
function [points,tform,inlierobjecteraserPoints,status,status2] = matchingfeatures(objectImage,objectPoints,sceneImage,scenePoints)
%   Extract the features from the scene and the object 
    [objectFeatures, objectPoints] = extractFeatures(objectImage, objectPoints);
    [sceneFeatures, scenePoints] = extractFeatures(sceneImage, scenePoints);
%   Match the features
    objectPairs = matchFeatures(objectFeatures, sceneFeatures);
    matchedobjectPoints = objectPoints(objectPairs(:, 1), :);
    matchedScenePoints = scenePoints(objectPairs(:, 2), :);
    if matchedobjectPoints.Count > 7 && matchedScenePoints.Count > 7
%   Get the inlierpoints in the scene and the tform of the object
        [tform, inlierobjecteraserPoints, inlierScenePoints,status] = ...
            estimateGeometricTransform(matchedobjectPoints, matchedScenePoints, 'affine');
%   Note that the status is required to be given; otherwise errors will occur
%   and the code will stop.
        points = inlierScenePoints;
        status2 = true;
    else 
        status2 = false;
        points = NaN;
        tform = NaN;
        inlierobjecteraserPoints = NaN;
        status = NaN;
    end
end