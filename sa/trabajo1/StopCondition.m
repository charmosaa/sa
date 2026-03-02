function [bool] = StopCondition(side, cx, cy)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

    switch side
        case 1
            bool = cy < 4;
        case 2
            bool = cx < 4;
        case 3
            bool = cy > 0;
        case 4
            bool = cx > 0;
        otherwise
            bool = false;
    end
end