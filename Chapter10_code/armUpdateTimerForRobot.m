function armUpdateTimerForRobot(~,~,handles)
%   Copyright 2014-2016 The MathWorks, Inc
    disableCollisionAvoidance(handles.bc,3);
    updateArms(handles.bc,3);
end