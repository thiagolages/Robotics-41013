function [intersectionPoint,check] = checkIntersection(pointOnPlane)
% A plane with normal [0,0,1] that goes through the origin is defined by

pointOnPlane  = [0,0,0];
planeNormal = [0,0,1]; 

% and a line that goes from [0,0,1] to [0,0,-1] would have

point1OnLine = [0,0,1];
point2OnLine = [0,0,-1];

% then call

[intersectionPoint,check] = LinePlaneIntersection(planeNormal,pointOnPlane,point1OnLine,point2OnLine);

% which will return the intersectionPoint and if there is an intersection with that plane when check==true. in this case the returned values should be
% intersectionPoint = [0,0,0] and check = true;