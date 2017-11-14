% Script to test Baxter Communicator class
%
%   This code was tested with Baxter Simulator (and ROS Kinetic Kame).
%   Description of the installation and use are provided in the book, 
%   ROS Robotics By Example 2nd edition by Carol Fairchild and Dr. Thomas Harman.
%
%   The original example script was created by Carlos Santacruz-Rosero of Mathworks
%   to utilize the classdef of BaxterCommWithSim.
%
%% Notes: 
%   1) If you are running on a laptop, you might need to disable your 
%      antivirus software. 
%   2) If you are running on a real Baxter, make sure to turn on Baxter with 
%      the E-Stop released.
%   3) This script will disable Baxter's Collision Avoidance so be careful when
%      using the robot!
%   4) If you are running on a real Baxer, Baxter's IP should be declared in the 
%      hosts file in C:\Windows\System32\drivers\etc check that your antivirus 
%      is off and E-Stop is disabled
%   Copyright 2014-2016 The MathWorks, Inc

%% If you are running on a real Baxter, connect to ROS MASTER on Baxter:
%   Change '011310P0010.local' to your Baxter's hostname
%   NodeHost should be the IP address of your laptop or host machine
% rosinit('011310P0010.local','NodeHost','192.168.11.139')

%% If you are running Baxter Simulation, use the folowing commands:
rosshutdown;
rosinit;

%% Create object of Baxter Communicator class
% bc = BaxterCommWithSim();       % for real Baxter
bc = BaxterCommWithSim(true);     % for Baxter Simulation

%% Turn on Motors
enable(bc);

%% Start timer to periodically update the joint commands
handles.bc = bc;

%%%% be aware that the timer disables Collision Avoidance for the real Baxter  %%%%
% uncomment next line if using real Baxter
% timer1 = timer('TimerFcn',{@armUpdateTimerForRobot,handles},'Period',0.1,'ExecutionMode','fixedSpacing');

% uncomment next line if using Baxter Simulation
timer1 = timer('TimerFcn',{@armUpdateTimerForSim,handles},'Period',0.1,'ExecutionMode','fixedSpacing');

start(timer1);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% The arm and gripper control commands below are passed the object handle (bc) and the VALUE.
%    The VALUE argument denotes which gripper or arm will be activated:
%    1=left,2=right,3=both
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Untuck both arms
untuck(bc, 3);

%% Calibrate both grippers
calibrateGrip(bc,3);

%% The grip and release commands will only work if the gripper has been calibrated.
%% Close right arm gripper ('go' to 'position: 0.0')
grip(bc, 2);

%% Release right arm gripper ('go' to 'position: 100.0')
release(bc, 2);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% The head pan commands below are passed the object handle (bc), the TARGET and the SPEED_RATIO.
%      The TARGET argument is a 32bit float for the angle in radians.
%      The SPEED_RATIO argument is a 32bit float for the percentage of the maximum speed (0.0-1.0).
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Rotate Baxter's head to the side and then back to the center slowly
panHead(bc, 1.0);
panHead(bc, 0.0, 0.2);

%% Move Baxter's arms to a Y pose.
pose_Y(bc);


%% Activate endpoint feedback
enableEndpointDisplay(bc,3);

%% Get right arm current pose
currPose = bc.RightArmEndpoint;
disp(currPose.Pos);
disp(currPose.Orientation);


%% Send both arms to the untuck configuration again
untuck(bc, 3);

%% Send both arms to the tuck configuration 
tuck(bc, 3);

%% Turn off servos
disable(bc);

%% Stop and delete timer started in baxterStartTime
stop(timer1);
delete(timer1);

%% Shutdown ROS
rosshutdown;







 
 


