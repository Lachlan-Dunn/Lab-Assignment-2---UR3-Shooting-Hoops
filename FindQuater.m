%% Solve Quater
%
% Solve quater defines the quadrant a point is inside the x/y plane
%
% quater = FindQuater(X,Y);

% OUTPUTS
% quater = Offset to add to the calculated angle after defining quadrent

% INPUTS
% X = X co-ordinate of the point
% Y = Y co-ordinate of the point


function quater = FindQuater(X,Y);

quater = 0;
if X<0
    quater = 270;
    if Y<0
        quater = quater - 90;
    end
else
    if Y<0
        quater = quater + 90;
    end
end
end

