function [bool] = StopCondition(side, cx, cy, side_length)
%UNTITLED Based on the side we are drawing it asseses stop condition
%   For going up is side 1, going right is side 2, side 3 is down and side
%   4 is left

    switch side
        case 1
            bool = cy < side_length;
        case 2
            bool = cx < side_length;
        case 3
            bool = cy > 0;
        case 4
            bool = cx > 0;
        otherwise
            bool = false;
    end
end