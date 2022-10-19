classdef SprayPaintingBots
    %SPRAYPAINTINGBOTS Summary of this class goes here
    %   A dual cobot system designed to spray paint pieces of paper
    
    properties
%         CameraRgbSub;
%         CameraDepthSub;
%         PoseSub;
        jointStateSubscriber;


        MarkerImg;
        Intrinsics;
        MarkerSize = 0.09;
        paperSize = [0.2 0.3];


        bufferSeconds = 1; % This allows for the time taken to send the message. If the network is fast, this could be reduced.
        durationSeconds = 5; % This is how many seconds the movement will take
    end
    
    methods
        function obj = SprayPaintingBots(NodeHost)
            %SPRAYPAINTINGBOTS Construct an instance of this class
            %   Detailed explanation goes here

%             rosinit(NodeHost); % input NodeHost as '192.168.0.253'
            
%             focalLength    = [554 554]; 
%             principalPoint = [320 240];
%             imageSize      = [480 640]; %change these to the camera required

%             obj.Intrinsics = cameraIntrinsics(focalLength,principalPoint,imageSize);
%             obj.MarkerImg = rgb2gray (imread('../meshes/0.png'));
%             obj.CameraRgbSub = rossubscriber();
%             obj.CameraDepthSub = rossubscriber();
%             obj.PoseSub = rossubscriber();
            jointStateSubscriber = rossubscriber('joint_states','sensor_msgs/JointState');

        end

        function StartWorkFlow
            
%             depthMsg = CameraDepthCallback(obj);
%             rbgImgMsg = receive(obj.CameraRgbSub);
%             [markerPresent,paperPose] = AnalyseImage(obj, rgbImgMsg, depthMsg, ur3Pose);

            paperPose = eye(4,4);
            paperPose = paperPose*transl(0,-0.6,0.5);
            paperCornersAll = GetPaperCorners(obj, paperPose);
            
            jointNames = {'shoulder_pan_joint','shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'};
            
            [client, goal] = rosactionclient('/scaled_pos_joint_traj_controller/follow_joint_trajectory');
            goal.Trajectory.JointNames = jointNames;
            goal.Trajectory.Header.Seq = 1;
            goal.Trajectory.Header.Stamp = rostime('Now','system');
            goal.GoalTimeTolerance = rosduration(0.05);

            ur3Robot = getUR3(obj);
            [nextJointState_123456] = SprayPaintUR3Sim(obj, ur3Robot, paperCornersAll, paperMoving);

            [startJointSend, currentJointState_123456] = UR3PoseCallback(obj);
            [goal] = UR3GetTrajectory(obj, startJointSend, nextJointState_123456, goal);
            SprayPaintUR3Real(obj, nextJointState_123456, client, goal)



        end
        
        function outputArg = CameraRGBCallback(obj)
            rbgImgMsg = receive(obj.CameraRgbSub);
        end

        function depthMsg = CameraDepthCallback(obj)
            depthMsg = receive(obj.CameraDepthSub);
        end

        function [startJointSend, currentJointState_123456] = UR3PoseCallback(obj)
            currentJointState_321456 = (obj.jointStateSubscriber.LatestMessage.Position)'; % Note the default order of the joints is 3,2,1,4,5,6
            currentJointState_123456 = [currentJointState_321456(3:-1:1),currentJointState_321456(4:6)];
            
            startJointSend = rosmessage('trajectory_msgs/JointTrajectoryPoint');
            startJointSend.Positions = currentJointState_123456;
            startJointSend.TimeFromStart = rosduration(0);
        end

        function [goal] = UR3GetTrajectory(obj, startJointSend, nextJointState_123456, goal)
            endJointSend = rosmessage('trajectory_msgs/JointTrajectoryPoint');
%             nextJointState_123456 = currentJointState_123456 + [pi/8,0,0,0,0,pi/8];
            endJointSend.Positions = nextJointState_123456;
            endJointSend.TimeFromStart = rosduration(obj.durationSeconds);

            goal.Trajectory.Points = [startJointSend; endJointSend];
        end

        function [markerPresent,paperPose] = AnalyseImage(obj, rgbImgMsg, depthMsg, ur3Pose)
            rgbImg = rosReadImage(rgbImgMsg);
            grayImage = rgb2gray(rgbImg);

            % adjust the image to allow for easier detection
            refinedImage = imadjust(grayImage);
            refinedImage = imlocalbrighten(refinedImage);
            refinedImage(refinedImage >= 10) = 255; % make grey pixels white to increase contrast
            %imshow(refinedImage);

            % april tag     
            I = undistortImage(rgbImg,obj.Intrinsics,OutputView="same");
            [id,loc,pose] = readAprilTag(refinedImage,"tag36h11", obj.Intrinsics,obj.MarkerSize);
            
            if isempty(id)
                markerPresent = false;
                worldPose = pose;
                disp("Marker was not detected");
            else 
                % rotate axis to coinside with robot frame
                rotation = eul2rotm([0 0 -pi/2]);
                tform = rigid3d(rotation,[0 0 0]);
                updatedR = pose.Rotation * tform.Rotation;
                pose = rigid3d(updatedR, pose.Translation);

                % display tag axis
                worldPoints = [0 0 0; obj.MarkerSize/2 0 0; 0 obj.MarkerSize/2 0; 0 0 obj.MarkerSize/2];

                % Get image coordinates for axes.
                imagePoints = worldToImage(obj.Intrinsics,pose(1),worldPoints);
            
                % Draw colored axes.
                I = insertShape(I,Line=[imagePoints(1,:) imagePoints(2,:); ...
                    imagePoints(1,:) imagePoints(3,:); imagePoints(1,:) imagePoints(4,:)], ...
                    Color=["red","green","blue"],LineWidth=7);
            
                I = insertText(I,loc(1,:,1),id(1),BoxOpacity=1,FontSize=12);
    
                % find central tag image point
                uMin = min(loc(:,1));
                uMax = max(loc(:,1));
                vMin = min(loc(:,2));
                vMax = max(loc(:,2));
    
                centerPoint = [round(mean([uMax uMin])) round(mean([vMax vMin]))];                 
                I = insertMarker(I,centerPoint,"circle","Size",10,"Color","yellow");
                %figure(1);
                %imshow(I);
       
                % convert image point to 3d points
                depthImg = rosReadImage(depthMsg);
                depth = depthImg(centerPoint(2),centerPoint(1)); % get from sensor

                % Draw colored axes.
                depthImg = insertShape(depthImg,Line=[imagePoints(1,:) imagePoints(2,:); ...
                    imagePoints(1,:) imagePoints(3,:); imagePoints(1,:) imagePoints(4,:)], ...
                    Color=["red","green","blue"],LineWidth=7);
            
                depthImg = insertText(depthImg,loc(1,:,1),id(1),BoxOpacity=1,FontSize=12);
                depthImg = insertMarker(depthImg,centerPoint,"circle","Size",10,"Color","yellow");

                %figure(2);
                %imshow(depthImg);

                translation = [depth ...
                    depth * (centerPoint(1)-obj.Intrinsics.PrincipalPoint(1))/obj.Intrinsics.FocalLength(1) ...
                    depth * (centerPoint(2)-obj.Intrinsics.PrincipalPoint(2))/obj.Intrinsics.FocalLength(2)];

                poseM = eul2rotm([0 0 0]); %eul2rotm([-1.57 0 -1.57]); % from camera - see urdf file
                poseM(1:3,4) = translation';
                poseM(4,4) = 1;
    
                quat = ur3Pose.Orientation;
                worldPoseTr = quat2rotm([quat.W quat.X quat.Y quat.Z]);
                worldPoseTr(1:3,4) = [ur3Pose.Position.X;ur3Pose.Position.Y;ur3Pose.Position.Z];
                worldPoseTr(4,4) = 1;
    
                paperPose = worldPoseTr * poseM;

                markerPresent = true;
                disp("Marker detected at");
                disp(paperPose);
            end
        end

        function paperCornersAll = GetPaperCorners(obj, paperPose)
            %METHOD1 Summary of this method goes here
            %   Detailed explanation goes here

            topLeft = paperPose*transl(obj.paperSize(1)/2,0,obj.paperSize(2)/2);
            topRight = paperPose*transl(-obj.paperSize(1)/2,0,obj.paperSize(2)/2);
            bottomLeft = paperPose*transl(obj.paperSize(1)/2,0,-obj.paperSize(2)/2);
            bottomRight = paperPose*transl(-obj.paperSize(1)/2,0,-obj.paperSize(2)/2);

            paperCorners = [topLeft; topRight; bottomLeft; bottomRight];
            numPaperCorners = 4;

            for i = 1:numPaperCorners
                paperCornersAll(:,:,i) = paperCorners((i-1)*4+(1:4),1:4);
            end
        end

        function ur3Robot = getUR3(obj)
            %   create the UR3 at a point in the workspace
             ur3Robot = UR3;
            % Move from zero position to position ready to spray
            
            % readyEEPosition = [-0.2133; -0.1942; 0.4809];
            % readyEEAngles = [pi 0 -pi/2];
            % readyEEOrientation = eul2rotm(readyEEAngles);
            % T2 = cat(1,cat(2,readyEEOrientation,readyEEPosition),[0 0 0 1]);
            steps = 20;
            q1 = ur3Robot.model.getpos();
            q2 = [0 -pi/2 pi/2 pi 0 0];
            
            qMatrix = jtraj(q1,q2,steps);
            
            for i = 1:steps
                ur3Robot.model.animate(qMatrix(i,:));
                pause(0.01);
            end
        end

        function [nextJointState_123456] = SprayPaintUR3Sim(obj, ur3Robot, paperCornersAll, paperMoving)            
            % Begin when paper is detected and not moving
            steps = 20;
            if paperMoving == 0
                % Translate paper corners away from paper by x distance
                distanceFromPaper = 0.2;
                goalTopLeft = paperCornersAll(:,:,1)*transl(0,distanceFromPaper,0);
                goalTopRight = paperCornersAll(:,:,2)*transl(0,distanceFromPaper,0);
                goalBottomLeft = paperCornersAll(:,:,3)*transl(0,distanceFromPaper,0);
                goalBottomRight = paperCornersAll(:,:,4)*transl(0,distanceFromPaper,0);
                
                % Make two more waypoints in the centre of the paper
                distx = goalTopRight(1,4)-goalTopLeft(1,4);
                disty = goalBottomRight(1,4)-goalBottomLeft(1,4);
                goalTopCentre = goalTopLeft*transl(distx/2,disty/2,0);
                goalBottomCentre = goalBottomLeft*transl(distx/2,disty/2,0);
                
                paperPoints = [goalTopLeft; goalBottomLeft; goalTopCentre; goalBottomCentre; goalTopRight; goalBottomRight];
                numPaperPoints = 6;
                
                for i = 1:numPaperPoints
                    paperPointsAll(:,:,i) = paperPoints((i-1)*4+(1:4),1:4);
                    paperPointsAll(1:3,1:3,i) = [-1 0 0;0 0 -1; 0 -1 0];
                end
                % Spray the paper at all points
                for j = 1:numPaperPoints
                    q1 = ur3Robot.model.getpos();
                    TR1 = ur3Robot.model.fkine(q1);
                    TR2 = paperPointsAll(:,:,j);
                    q2 = ur3Robot.model.ikcon(TR2, q1); % , q1, [1 1 1 0 1 1]
                    nextJointState_123456(:,:,j) = q2;
                    
%                     qMatrix = jtraj(q1,q2,steps);
                    s = lspb(0,1,steps);
                    for i = 1:steps
                      qMatrix(i,:) = (1-s(i))*q1 + s(i)*q2;
                    end
                    
                    for i = 1:steps
                        ur3Robot.model.animate(qMatrix(i,:));
                        pause(0.1);
                    end
                    EEPose = ur3Robot.model.fkine(ur3Robot.model.getpos());
                    errorMarginPos = TR2(1:3,4)-EEPose(1:3,4)
                    errorMarginRot = rotm2eul(TR2(1:3,1:3))-rotm2eul(EEPose(1:3,1:3))
                 end
                
                % Return to ready and wait for next paper
                q1 = ur3Robot.model.getpos();
                q2 = [0 -pi/2 pi/2 pi 0 0];
                
                qMatrix = jtraj(q1,q2,steps);
                
                for i = 1:steps
                    ur3Robot.model.animate(qMatrix(i,:));
                    pause(0.01);
                end
            end
        end

        %% Do Collision Detection!!!

        function SprayPaintUR3Real(obj, nextJointState_123456, client, goal) 
            numPaperPoints = 6;
            % Spray the paper at all points
            for j = 1:numPaperPoints
%                 q1 = ur3Robot.model.getpos();
%                 TR1 = ur3Robot.model.fkine(q1);
%                 TR2 = paperPointsAll(:,:,j);
%                 q2 = ur3Robot.model.ikcon(TR2, q1); % , q1, [1 1 1 0 1 1]

                [startJointSend, currentJointState_123456] = UR3PoseCallback(obj);
                disp(currentJointState_123456)
                goal = UR3GetTrajectory(obj, startJointSend, nextJointState_123456(:,:,j), goal);
                disp(nextJointState_123456(:,:,j))
                
                
                goal.Trajectory.Header.Stamp = obj.jointStateSubscriber.LatestMessage.Header.Stamp + rosduration(obj.bufferSeconds);
                sendGoal(client,goal);
%                 EEPose = ur3Robot.model.fkine(ur3Robot.model.getpos());
%                 errorMarginPos = TR2(1:3,4)-EEPose(1:3,4)
%                 errorMarginRot = rotm2eul(TR2(1:3,1:3))-rotm2eul(EEPose(1:3,1:3))
             end
            
            % Return to ready and wait for next paper
%             q1 = ur3Robot.model.getpos();
            q2 = [0 -pi/2 pi/2 pi 0 0];
            [startJointSend, currentJointState_123456] = UR3PoseCallback(obj);
            disp(currentJointState_123456)
            [goal] = UR3GetTrajectory(obj, startJointSend, q2, goal);
            disp(q2)
            
            goal.Trajectory.Header.Stamp = obj.jointStateSubscriber.LatestMessage.Header.Stamp + rosduration(obj.bufferSeconds);
            sendGoal(client,goal);
        end


    end
end

