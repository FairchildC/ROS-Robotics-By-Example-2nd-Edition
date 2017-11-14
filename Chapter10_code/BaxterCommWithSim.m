classdef BaxterCommWithSim < handle
    %BaxterComm - Class for communicating with the Baxter robot
	%   The original code for BaxterCommWithSim.m was written by Carlos 
        %   Santacruz-Rosero of MathWorks. Carlos developed the MATLAB 
        %   classdef of BaxterCommWithSim to have Baxter to play checkers
        %   with a human opponent (2014).  It has been modified to work 
        %   with Baxter's latest version of software v1.2.0 by Carol Fairchild 
        %   and Dr. Thomas Harman.  This code is featured in their book, 
        %   ROS Robotics By Example 2nd edition (2017).
    %
    %   This code was tested with Baxter Simulator (and ROS Kinetic Kame).
    %   Description of the installation and use are provided in the book.
    %	
    %   BC = BaxterComm() creates an object that allows for easy
    %   control and interaction with the Baxter manipulator platform. The
    %   constructor takes no arguments and subscribes to additional topics
    %   when the 'enable' functions are used.
    %
    %   Communicator methods:
    %       BaxterComm              - Constructor
    %       enable                  - Enables robot movement
    %       disable                 - Disables robot movement
    %       enableEndpointDisplay   - Enables subscriber for arm endpoints
    %       enableJointAngleDisplay - Enables subscriber for joint angles
    %       panHead                 - Pans head to specified angle
    %       updateArms              - Updates arm position
    %       disableCollisionAvoidance   - Disables collision avoidance
    %       untuck                  - Set target arm point as untucked
    %       tuck                    - Set target arm point as tucked
    %       armToPointLeft          - Specify target left arm end pose
    %       armToPointRight         - Specify target right arm end pose
    %       armToAnglesLeft         - Specify target left arm joint angles
    %       armToAngleRight         - Specify target right arm joint angles
    %       calibrateGrip           - Calibrates gripper
    %       grip                    - Grips with specified gripper
    %       release                 - Releases with specified gripper
    %       showTabletImage         - Displays specified image on tablet
    %
    %   Communicator properties:
    %       Enabled             - Boolean for motion activation state
    %       LeftArmAngles       - Angles of left arm joints
    %       RightArmAngles      - Angles of right arm joints
    %       LeftArmEndpoint     - Endpoint of left arm
    %       RightArmEndpoint    - Endpoint of right arm
    %       LeftArmPoseCommand  - Commanded endpoint pose for left arm
    %       RightArmPoseCommand - Commanded endpoint pose for right arm
    %       LeftArmAnglesCommand    - Commanded joint angles for left arm
    %       RightArmAnglesCommand   - Commanded joint angles for right arm
    %       DisplayImage        - Image shown on Baxter display
    %       GripCalibration     - Indicates which grippers are calibrated
    %       EndpointDisplay     - Indicates if endpoints are shown
    %       JointAngleDisplay   - Indicates if joint angles are shown
    %       SonarEnabled        - Indicates if sonar is enabled
    %       LaserEnabled        - Indicates if lasers are enabled
    %   
    %   Notes:
    %       For joint angle lists, convention is:
    %       s0, s1, e0, e1, w0, w1, w2 
    %       Quaternion convention is: [qw, qx, qy, qz]
    %       Euler angle convention is: [rpy]
    %       See SDK for more information: http://sdk.rethinkrobotics.com/wiki/API_Reference
    
    %   Copyright 2014 The MathWorks, Inc.
    
    properties (SetAccess = protected)
        Enabled = false;
        LeftArmAngles = [];
        RightArmAngles = [];
        LeftArmEndpoint = [];
        RightArmEndpoint = [];
        LeftArmPoseCommand = [];
        RightArmPoseCommand = [];
        LeftArmAnglesCommand = [];
        RightArmAnglesCommand = [];
        DisplayImage = [];
        GripCalibration = [0 0];    % left, right
        EndpointDisplay = 0;        % none, left, right, both
        JointAngleDisplay = 0;
    end
    
    properties (Access = protected)
        SuperEnablePub = [];
        SuperEnableMsg = [];
        StateSub = [];

        % ROS Objects for Arms and Head
        HeadPanPub = [];
        HeadPanMsg = [];

        ArmCmdRightPub = [];
        ArmCmdRightMsg = [];
        ArmCmdLeftPub = [];
        ArmCmdLeftMsg = [];
        CollisionSuppressLeftPub = [];
        CollisionSuppressLeftMsg = [];
        CollisionSuppressRightPub = [];
        CollisionSuppressRightMsg = [];
        LeftEndpointSub = [];
        RightEndpointSub = [];
        JointDisplaySub = [];

        % ROS Objects for Display
        DisplayPub = [];
        DisplayMsg = [];

        % ROS Objects for Grippers
        GripRightSub = [];
        GripRightPub = [];
        GripRightMsg = [];
        GripLeftSub = [];
        GripLeftPub = [];
        GripLeftMsg = [];
    end
    
    methods
        function h = BaxterCommWithSim(isSimulation)
            %Communicator Constructor
            
            % Define Enable Objects
            h.SuperEnablePub = rospublisher('/robot/set_super_enable');
            h.SuperEnableMsg = rosmessage(h.SuperEnablePub.MessageType);
            
            % Define Head Objects
            h.HeadPanPub = rospublisher('/robot/head/command_head_pan');
            h.HeadPanMsg = rosmessage(h.HeadPanPub.MessageType);
            h.HeadNodPub = rospublisher('/robot/head/command_head_nod');
            h.HeadNodMsg = rosmessage(h.HeadNodPub.MessageType);
            
            % Define Collision Suppression Objects
            if ((nargin == 1) && ~isSimulation) || nargin == 0
                h.CollisionSuppressLeftPub = rospublisher('/robot/limb/left/suppress_collision_avoidance');
                h.CollisionSuppressLeftMsg = rosmessage(h.CollisionSuppressLeftPub.MessageType);
                h.CollisionSuppressRightPub = rospublisher('/robot/limb/right/suppress_collision_avoidance');
                h.CollisionSuppressRightMsg = rosmessage(h.CollisionSuppressRightPub.MessageType);
            end
            
            % Define Arm Objects
            h.ArmCmdRightPub = rospublisher('/robot/limb/right/joint_command');
            h.ArmCmdRightMsg = rosmessage(h.ArmCmdRightPub.MessageType);
            h.ArmCmdLeftPub = rospublisher('/robot/limb/left/joint_command');
            h.ArmCmdLeftMsg = rosmessage(h.ArmCmdLeftPub.MessageType);
            
      
            % Define Display Objects
            % Note: Baxter expects a bgr formatted image
            h.DisplayPub = rospublisher('/robot/xdisplay');
            h.DisplayMsg = rosmessage(h.DisplayPub.MessageType);
            
            % Define Gripper Objects
            h.GripLeftSub = rossubscriber('/robot/end_effector/left_gripper/state');
            h.GripLeftPub = rospublisher('/robot/end_effector/left_gripper/command');
            h.GripLeftMsg = rosmessage(h.GripLeftPub.MessageType);

            leftGripMsg = receive(h.GripLeftSub,5);
            h.GripLeftMsg.Id = leftGripMsg.Id;

            h.GripRightSub = rossubscriber('/robot/end_effector/right_gripper/state');
            h.GripRightPub = rospublisher('/robot/end_effector/right_gripper/command');
            h.GripRightMsg = rosmessage(h.GripRightPub.MessageType);

            rightGripMsg = receive(h.GripRightSub,5);
            h.GripRightMsg.Id = rightGripMsg.Id;
            
            % Determine Robot State
            h.StateSub = rossubscriber('/robot/state','baxter_core_msgs/AssemblyState');
            stateData = receive(h.StateSub,5);
            if stateData.Enabled
                h.Enabled = true;
            else
                h.Enabled = false;
            end
        end
        
        function enable(h)
            %ENABLE - Enables robot movement
            if ~h.Enabled
                h.SuperEnableMsg.Data = 1;
                send(h.SuperEnablePub,h.SuperEnableMsg);
                disp('Enabled Baxter Robot')
                h.Enabled = true;
            else
                disp('Baxter Already Enabled: No Action Taken');
            end
        end
        
        function disable(h)
            %DISABLE - Disables robot movement
            if h.Enabled
                h.SuperEnableMsg.Data = 0;
                send(h.SuperEnablePub,h.SuperEnableMsg);
                disp('Disabled Baxter Robot')
                h.Enabled = false;
            else
                disp('Baxter Already Disabled: No Action Taken');
            end
        end
        
        function enableEndpointDisplay(h,value)
            %ENABLEENDPOINTDISPLAY - Enables display of endpoints
            %   VALUE specifies which limbs are affected:
            %   value: 0=none, 1=left, 2=right, 3=both
            if value == 1
                h.RightEndpointSub = [];
                h.RightArmEndpoint = [];
                h.LeftEndpointSub = rossubscriber('/robot/limb/left/endpoint_state');
                h.LeftEndpointSub.NewMessageFcn = {@h.leftEndpointCallback};
            elseif value == 2
                h.LeftEndpointSub = [];
                h.LeftArmEndpoint =[];
                h.RightEndpointSub = rossubscriber('/robot/limb/right/endpoint_state');
                h.RightEndpointSub.NewMessageFcn = {@h.rightEndpointCallback};
            elseif value == 3
                h.LeftEndpointSub = rossubscriber('/robot/limb/left/endpoint_state');
                h.LeftEndpointSub.NewMessageFcn = {@h.leftEndpointCallback};
                h.RightEndpointSub = rossubscriber('/robot/limb/right/endpoint_state');
                h.RightEndpointSub.NewMessageFcn = {@h.rightEndpointCallback};
            else
                h.LeftEndpointSub = [];
                h.RightEndpointSub = [];
                h.LeftArmEndpoint = [];
                h.RightArmEndpoint = [];
            end
            h.EndpointDisplay = value;
        end
        
        function enableJointAngleDisplay(h,value)
            %ENABLEJOINTANGLEDISPLAY - Enables display of joint angles
            %   VALUE is an on/off boolean
            %   value: 0=off, 1=on
            if value==1
                h.JointDisplaySub = rossubscriber('/robot/joint_states');
                h.JointDisplaySub.NewMessageFcn = {@h.jointAnglesCallback};
                h.JointAngleDisplay = 1;
            else
                h.JointDisplaySub = [];
                h.LeftArmAngles = [];
                h.RightArmAngles = [];
                h.JointAngleDisplay = 0;
            end
        end
        
        
        function panHead(h,angle,varargin)
            %PANHEAD - Turns head to a specified angle orientation
            %   Takes ANGLE as an argument, specified in radians. The
            %   optional argument specifies the speed by an integer value
            %   of 1-100.
            if ~h.Enabled
                disp('Robot Not Enabled. Enable Robot To Execute Command');
                return
            end
            
            h.HeadPanMsg.Target = angle;    % Radians
            if ~isempty(varargin)
                h.HeadPanMsg.Speed = varargin{1}; % int from 1-100
            else
                h.HeadPanMsg.Speed = 10;
            end
            send(h.HeadPanPub,h.HeadPanMsg)
        end

        function updateArms(h,value)
            % UPDATEARMS - Sends an arm update command to the robot. 
            %   The VALUE argument specifies which arm to update:
            %   1=left,2=right,3=both
            %   This command must be sent repeatedly for the arms to reach
            %   the desired points, since the robot will default back to a
            %   limp state otherwise. The arm position will be updated to
            %   the latest arm commands, which are stored in the
            %   LeftArmAnglesCommand and RightArmAnglesCommand properties.
            if ~h.Enabled
                disp('Robot Not Enabled. Enable Robot To Execute Command');
                return
            end
            
            if value==1
                send(h.ArmCmdLeftPub,h.ArmCmdLeftMsg);
            elseif value==2
                send(h.ArmCmdRightPub,h.ArmCmdRightMsg);
            elseif value==3
                send(h.ArmCmdLeftPub,h.ArmCmdLeftMsg);
                send(h.ArmCmdRightPub,h.ArmCmdRightMsg);
            end
        end
        
        function disableCollisionAvoidance(h,value)
            %DISABLECOLLISIONAVOIDANCE - Disables collision avoidance in a
            %specified arm
            %   The VALUE argument specifies the arm:
            %   1=left,2=right,3=both
            %   Many arm motions (such as tuck and untuck) are only
            %   possible if collision avoidance is disabled before each
            %   update command is sent.
            if value==1
                send(h.CollisionSuppressLeftPub,h.CollisionSuppressLeftMsg);
            elseif value==2
                send(h.CollisionSuppressRightPub,h.CollisionSuppressRightMsg);
            elseif value==3
                send(h.CollisionSuppressLeftPub,h.CollisionSuppressLeftMsg);
                send(h.CollisionSuppressRightPub,h.CollisionSuppressRightMsg);
            end
        end
        
        function untuck(h,value)
            %UNTUCK - Specifies arm command to untucked position
            %   The VALUE argument specifies the arm:
            %   1=left,2=right,3=both
            %   This command does not update the arms- it strictly updates
            %   the desired position.
            if ~h.Enabled
                disp('Robot Not Enabled. Enable Robot To Execute Command');
                return
            end
            
            if value==1
                h.ArmCmdLeftMsg.Mode = 1;
                h.ArmCmdLeftMsg.Command = [-0.08, -1.0, -1.19, 1.94,  0.67, 1.03, -0.50];
                h.ArmCmdLeftMsg.Names = {'left_s0', 'left_s1', 'left_e0', 'left_e1', 'left_w0', 'left_w1', 'left_w2'};
            elseif value==2
                h.ArmCmdRightMsg.Mode = 1;
                h.ArmCmdRightMsg.Command = [0.08, -1.0,  1.19, 1.94, -0.67, 1.03,  0.50];
                h.ArmCmdRightMsg.Names = {'right_s0', 'right_s1', 'right_e0', 'right_e1', 'right_w0', 'right_w1', 'right_w2'};
            elseif value==3
                h.ArmCmdLeftMsg.Mode = 1;
                h.ArmCmdLeftMsg.Command = [-0.08, -1.0, -1.19, 1.94,  0.67, 1.03, -0.50];
                h.ArmCmdLeftMsg.Names = {'left_s0', 'left_s1', 'left_e0', 'left_e1', 'left_w0', 'left_w1', 'left_w2'};
                h.ArmCmdRightMsg.Mode = 1;
                h.ArmCmdRightMsg.Command = [0.08, -1.0,  1.19, 1.94, -0.67, 1.03,  0.50];
                h.ArmCmdRightMsg.Names = {'right_s0', 'right_s1', 'right_e0', 'right_e1', 'right_w0', 'right_w1', 'right_w2'};
            end
            h.LeftArmAnglesCommand = h.ArmCmdLeftMsg.Command;
            h.RightArmAnglesCommand = h.ArmCmdLeftMsg.Command;
        end
        
        function tuck(h,value)
            %TUCK - Specifies arm command to tucked position
            %   The VALUE argument specifies the arm:
            %   1=left,2=right,3=both
            %   This command does not update the arms- it strictly updates
            %   the desired position.
            if ~h.Enabled
                disp('Robot Not Enabled. Enable Robot To Execute Command');
                return
            end
            
            if value==1
                h.ArmCmdLeftMsg.Mode = 1;
                h.ArmCmdLeftMsg.Command = [-1.0, -2.07,  3.0, 2.55,  0.0, 0.01,  0.0];
                h.ArmCmdLeftMsg.Names = {'left_s0', 'left_s1', 'left_e0', 'left_e1', 'left_w0', 'left_w1', 'left_w2'};
            elseif value==2
                h.ArmCmdRightMsg.Mode = 1;
                h.ArmCmdRightMsg.Command = [1.0, -2.07, -3.0, 2.55, -0.0, 0.01,  0.0];
                h.ArmCmdRightMsg.Names = {'right_s0', 'right_s1', 'right_e0', 'right_e1', 'right_w0', 'right_w1', 'right_w2'};
            elseif value==3
                h.ArmCmdLeftMsg.Mode = 1;
                h.ArmCmdLeftMsg.Command = [-1.0, -2.07,  3.0, 2.55,  0.0, 0.01,  0.0];
                h.ArmCmdLeftMsg.Names = {'left_s0', 'left_s1', 'left_e0', 'left_e1', 'left_w0', 'left_w1', 'left_w2'};
                h.ArmCmdRightMsg.Mode = 1;
                h.ArmCmdRightMsg.Command = [1.0, -2.07, -3.0, 2.55, -0.0, 0.01,  0.0];
                h.ArmCmdRightMsg.Names = {'right_s0', 'right_s1', 'right_e0', 'right_e1', 'right_w0', 'right_w1', 'right_w2'};
            end
            h.LeftArmAnglesCommand = h.ArmCmdLeftMsg.Command;
            h.RightArmAnglesCommand = h.ArmCmdLeftMsg.Command;
        end



        % This function is an example of commanding Baxter's arms to a specific pose.  The pose specified is Y with Baxter's
		% arms raised up and to either side of his head.
		
        function pose_Y(h)
              if ~h.Enabled
                disp('Robot Not Enabled. Enable Robot To Execute Command');
                return
              end
              disp('y')
                h.ArmCmdLeftMsg.Mode = 1;
                h.ArmCmdLeftMsg.Command = [0, -1.0, 0, 0, 0, 0, 0];
                h.ArmCmdLeftMsg.Names = {'left_s0', 'left_s1', 'left_e0', 'left_e1', 'left_w0', 'left_w1', 'left_w2'};
                h.ArmCmdRightMsg.Mode = 1;
                h.ArmCmdRightMsg.Command = [0, -1.0, 0, 0, 0, 0, 0];
                h.ArmCmdRightMsg.Names = {'right_s0', 'right_s1', 'right_e0', 'right_e1', 'right_w0', 'right_w1', 'right_w2'};
                h.LeftArmAnglesCommand = h.ArmCmdLeftMsg.Command;
                h.RightArmAnglesCommand = h.ArmCmdLeftMsg.Command;
        end
       
        function armToAnglesLeft(h,angles)
            %ARMTOANGLESLEFT - Specifies left arm angle positions
            %   ANGLES is received as a 1x7 vector with the elements given
            %   in the following order: 'left_s0', 'left_s1', 'left_e0',
            %   'left_e1', 'left_w0', 'left_w1', 'left_w2'.
            %   All values are given in radians. The 'LeftArmAngles'
            %   property is stored in the same format, so it can be used to
            %   discover appropriate joint angles.
            
            h.ArmCmdLeftMsg.Mode = 1;
            h.ArmCmdLeftMsg.Command = angles';
            h.ArmCmdLeftMsg.Names = {'left_s0', 'left_s1', 'left_e0', 'left_e1', 'left_w0', 'left_w1', 'left_w2'};
            h.LeftArmAnglesCommand = h.ArmCmdLeftMsg.Command;
            h.RightArmAnglesCommand = h.ArmCmdRightMsg.Command;
        end
        
        function armToAnglesRight(h,angles)
            %ARMTOANGLESRIGHT - Specifies right arm angle positions
            %   ANGLES is received as a 1x7 vector with the elements given
            %   in the following order: 'right_s0', 'right_s1', 'right_e0',
            %   'right_e1', 'right_w0', 'right_w1', 'right_w2'.
            %   All values are given in radians. The 'RightArmAngles'
            %   property is stored in the same format, so it can be used to
            %   discover appropriate joint angles.
            
            h.ArmCmdRightMsg.Mode = 1;
            h.ArmCmdRightMsg.Command = angles';
            h.ArmCmdRightMsg.Names = {'right_s0', 'right_s1', 'right_e0', 'right_e1', 'right_w0', 'right_w1', 'right_w2'};
            h.LeftArmAnglesCommand = h.ArmCmdLeftMsg.Command;
            h.RightArmAnglesCommand = h.ArmCmdRightMsg.Command;
        end
 
        function calibrateGrip(h,value)
            %CALIBRATEGRIP - Calibrates the specified gripper
            %   The VALUE parameter denotes which grippers to calibrate:
            %   1=left, 2=right, 3=both
            %   The grippers must be calibrated before they are used.
            %   The GripCalibration property keeps track of this.
            if ~h.Enabled
                disp('Robot Not Enabled. Enable Robot To Execute Command');
                return
            end
            
            if value == 1
                if ~h.GripLeftSub.LatestMessage.Calibrated
                    h.GripLeftMsg.Command = 'calibrate';
                    send(h.GripLeftPub,h.GripLeftMsg);
                    h.GripCalibration(1) = h.GripLeftSub.LatestMessage.Calibrated;
                else
                    h.GripCalibration(1) = 1;
                end
            elseif value == 2
                if ~h.GripRightSub.LatestMessage.Calibrated
                    h.GripRightMsg.Command = 'calibrate';
                    send(h.GripRightPub,h.GripRightMsg);
                    h.GripCalibration(2) = h.GripRightSub.LatestMessage.Calibrated;
                else
                    h.GripCalibration(2) = 1;
                end
            elseif value == 3
                if ~h.GripLeftSub.LatestMessage.Calibrated
                    h.GripLeftMsg.Command = 'calibrate';
                    send(h.GripLeftPub,h.GripLeftMsg);
                    h.GripCalibration(1) = h.GripLeftSub.LatestMessage.Calibrated;
                else
                    h.GripCalibration(1) = 1;
                end
                if ~h.GripRightSub.LatestMessage.Calibrated
                    h.GripRightMsg.Command = 'calibrate';
                    send(h.GripRightPub,h.GripRightMsg);
                    pause(1.5);
                    h.GripCalibration(2) = h.GripRightSub.LatestMessage.Calibrated;
                else
                    h.GripCalibration(2) = 1;
                end
            end
        end
        
         function grip(h,value)
            %GRIP - Sends grip command to the specified gripper
            %   The VALUE argument denotes which gripper will be activated:
            %   1=left,2=right,3=both 
            %   Position 0.0 indicates a closed gripper
            %   This will only work if the gripper has been calibrated
            if ~h.Enabled
                disp('Robot Not Enabled. Enable Robot To Execute Command');
                return
            end
            
            if value == 1
              h.GripLeftMsg.Command = 'go'; 
              h.GripLeftMsg.Args = '{position: 0.0}';
              send(h.GripLeftPub,h.GripLeftMsg);
            elseif value == 2
              h.GripRightMsg.Command = 'go'; 
              h.GripRightMsg.Args = '{position: 0.0}';   
              send(h.GripRightPub,h.GripRightMsg);
            elseif value == 3
                h.GripLeftMsg.Command = 'go'; 
                h.GripLeftMsg.Args = '{position: 0.0}';
                send(h.GripLeftPub,h.GripLeftMsg);
               % 
                h.GripRightMsg.Command = 'go'; 
                h.GripRightMsg.Args = '{position: 0.0}';
                send(h.GripRightPub,h.GripRightMsg);
            end
        end
       
        function release(h,value)
            %RELEASE - Sends release command to the specified gripper
            %   The VALUE argument denotes which gripper will be activated:
            %   1=left,2=right,3=both
            %   Position 100.0 indicates an open gripper
            %   This will only work if the gripper has been calibrated
            if ~h.Enabled
                disp('Robot Not Enabled. Enable Robot To Execute Command');
                return
            end
            
            if value == 1
              h.GripLeftMsg.Command = 'go'; 
              h.GripLeftMsg.Args = '{position: 100.0}'; 
              send(h.GripLeftPub,h.GripLeftMsg);
            elseif value == 2
              h.GripRightMsg.Command = 'go'; 
              h.GripRightMsg.Args = '{position: 100.0}'; 
              send(h.GripRightPub,h.GripRightMsg);
            elseif value == 3
              h.GripLeftMsg.Command = 'go'; 
              h.GripLeftMsg.Args = '{position: 100.0}'; 
              send(h.GripLeftPub,h.GripLeftMsg);
              %  
              h.GripRightMsg.Command = 'go'; 
              h.GripRightMsg.Args = '{position: 100.0}'; 
              send(h.GripRightPub,h.GripRightMsg);
            end
        end
          
        function showTabletImage(h,img)
            %SHOWTABLETIMAGE - Publishes an image to the Baxter display
            %   The IMG argument takes an rgb8 image in standard MATLAB
            %   format and publishes it to the display. The image is
            %   converted from rgb8 to bgr8 to support the Baxter display.
            
            % Convert from normal matlab rgb8 to bgr8 (Baxter only accepts
            % bgr8).
            h.DisplayImage = img;
            imgcopy = img;
            img(:,:,1) = imgcopy(:,:,3);
            img(:,:,3) = imgcopy(:,:,1);
            
            h.DisplayMsg.Encoding = 'bgr8';
            writeImage(h.DisplayMsg,img);
            send(h.DisplayPub,h.DisplayMsg);
        end
    end
    
    methods (Access = protected)
        function headImageCallback(h, ~, message)
            h.HeadImage = readImage(message);
        end
        
        function leftImageCallback(h, ~, message)
            h.LeftArmImage = readImage(message);
        end
        
        function rightImageCallback(h, ~, message)
            h.RightArmImage = readImage(message);
        end
        
        function leftEndpointCallback(h, ~, message)
            h.LeftArmEndpoint.Pos = message.Pose.Position;
            h.LeftArmEndpoint.Orientation = message.Pose.Orientation;
        end
        
        function rightEndpointCallback(h, ~, message)
            h.RightArmEndpoint.Pos = message.Pose.Position;
            h.RightArmEndpoint.Orientation = message.Pose.Orientation;
        end
        
        function jointAnglesCallback(h, ~, message)
            h.LeftArmAngles = message.Position([5,6,3,4,7:9]);
            h.RightArmAngles = message.Position([12,13,10,11,14:16]);
        end
       
    end
end
