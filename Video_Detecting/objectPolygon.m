% Build up the polygonif the object.
function Polygon = objectPolygon(objectImage)
Polygon = [1, 1;...                           % top-left
        size(objectImage, 2), 1;...                 % top-right
        size(objectImage, 2), size(objectImage, 1);... % bottom-right
        1, size(objectImage, 1);...                 % bottom-left
        1, 1];
