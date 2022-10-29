classdef SprayPaintingBots
    %SPRAYPAINTINGBOTS Summary of this class goes here
    %   A dual cobot system designed to spray paint pieces of paper
    
    properties
%         CameraRgbSub;
%         CameraDepthSub;


        MarkerImg;
        Intrinsics;
        MarkerSize = 0.09;
        paperSize = [0.26 0.19]; %width by height

        paperPickupCoords = [-0.605 0.3 0.575];% Paper Coordinates
        paperPutdownCoords = [-0.605 -0.3 0.575];

        bufferSeconds = 1; % This allows for the time taken to send the message. If the network is fast, this could be reduced.
        durationSeconds = 5; % This is how many seconds the movement will take
    end
    
    methods
        function obj = SprayPaintingBots
            %SPRAYPAINTINGBOTS Construct an instance of this class
            %   Detailed explanation goes here

%             rosinit('192.168.0.253'); % input NodeHost as '192.168.0.253'
            
%             focalLength    = [554 554]; 
%             principalPoint = [320 240];
%             imageSize      = [480 640]; %change these to the camera required

%             obj.Intrinsics = cameraIntrinsics(focalLength,principalPoint,imageSize);
%             obj.MarkerImg = rgb2gray (imread('../meshes/0.png'));
%             obj.CameraRgbSub = rossubscriber();
%             obj.CameraDepthSub = rossubscriber();
%             jointStateSubscriber = rossubscriber('joint_states','sensor_msgs/JointState');

        end

        function StartRealUR3(obj)

            rosinit('192.168.0.253');
            jointStateSubscriber = rossubscriber('joint_states','sensor_msgs/JointState');
            pause(2);

            jointNames = {'shoulder_pan_joint','shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'};
            pause(2);

            [client, goal] = rosactionclient('/scaled_pos_joint_traj_controller/follow_joint_trajectory');
            pause(2);
            goal.Trajectory.JointNames = jointNames;
            goal.Trajectory.Header.Seq = 1;
            goal.Trajectory.Header.Stamp = rostime('Now','system');
            goal.GoalTimeTolerance = rosduration(0.05);

%             depthMsg = CameraDepthCallback(obj);
%             rbgImgMsg = receive(obj.CameraRgbSub);
%             [markerPresent,paperPose] = AnalyseImage(obj, rgbImgMsg, depthMsg, ur3Pose);
            if markerPresent && ~isnan(paperPose.Position.X)
                paperCornersAll = GetPaperCorners(obj, paperPose);

                clf
                [auboi3Robot, ur3Robot, paperModel] = PlotSprayPaintEnvironment(obj)
                [nextJointState_123456, movementDuration] = SprayPaintUR3Sim(obj, ur3Robot, paperCornersAll, 0)

                [startJointSend, currentJointState_123456] = UR3PoseCallback(obj, jointStateSubscriber);
                [goal] = UR3GetTrajectory(obj, startJointSend, nextJointState_123456, goal);
                readyRealUR3(obj, client, goal, jointStateSubscriber);
                SprayPaintUR3Real(obj, nextJointState_123456, client, goal, jointStateSubscriber, movementDuration)
            end



        end
        
        function outputArg = CameraRGBCallback(obj)
            rbgImgMsg = receive(obj.CameraRgbSub);
        end

        function depthMsg = CameraDepthCallback(obj)        
            depthMsg = receive(obj.CameraDepthSub);
        end

%         function [markerPresent,paperPose] = AnalyseImage(obj, rgbImgMsg, depthMsg, ur3Pose)
%             rgbImg = rosReadImage(rgbImgMsg);
%             grayImage = rgb2gray(rgbImg);
% 
%             % adjust the image to allow for easier detection
%             refinedImage = imadjust(grayImage);
%             refinedImage = imlocalbrighten(refinedImage);
%             refinedImage(refinedImage >= 10) = 255; % make grey pixels white to increase contrast
%             %imshow(refinedImage);
% 
%             % april tag     
%             I = undistortImage(rgbImg,obj.Intrinsics,OutputView="same");
%             [id,loc,pose] = readAprilTag(refinedImage,"tag36h11", obj.Intrinsics,obj.MarkerSize);
%             
%             if isempty(id)
%                 markerPresent = false;
%                 worldPose = pose;
%                 disp("Marker was not detected");
%             else 
%                 % rotate axis to coinside with robot frame
%                 rotation = eul2rotm([0 0 -pi/2]);
%                 tform = rigid3d(rotation,[0 0 0]);
%                 updatedR = pose.Rotation * tform.Rotation;
%                 pose = rigid3d(updatedR, pose.Translation);
% 
%                 % display tag axis
%                 worldPoints = [0 0 0; obj.MarkerSize/2 0 0; 0 obj.MarkerSize/2 0; 0 0 obj.MarkerSize/2];
% 
%                 % Get image coordinates for axes.
%                 imagePoints = worldToImage(obj.Intrinsics,pose(1),worldPoints);
%             
%                 % Draw colored axes.
%                 I = insertShape(I,Line=[imagePoints(1,:) imagePoints(2,:); ...
%                     imagePoints(1,:) imagePoints(3,:); imagePoints(1,:) imagePoints(4,:)], ...
%                     Color=["red","green","blue"],LineWidth=7);
%             
%                 I = insertText(I,loc(1,:,1),id(1),BoxOpacity=1,FontSize=12);
%     
%                 % find central tag image point
%                 uMin = min(loc(:,1));
%                 uMax = max(loc(:,1));
%                 vMin = min(loc(:,2));
%                 vMax = max(loc(:,2));
%     
%                 centerPoint = [round(mean([uMax uMin])) round(mean([vMax vMin]))];                 
%                 I = insertMarker(I,centerPoint,"circle","Size",10,"Color","yellow");
%                 %figure(1);
%                 %imshow(I);
%        
%                 % convert image point to 3d points
%                 depthImg = rosReadImage(depthMsg);
%                 depth = depthImg(centerPoint(2),centerPoint(1)); % get from sensor
% 
%                 % Draw colored axes.
%                 depthImg = insertShape(depthImg,Line=[imagePoints(1,:) imagePoints(2,:); ...
%                     imagePoints(1,:) imagePoints(3,:); imagePoints(1,:) imagePoints(4,:)], ...
%                     Color=["red","green","blue"],LineWidth=7);
%             
%                 depthImg = insertText(depthImg,loc(1,:,1),id(1),BoxOpacity=1,FontSize=12);
%                 depthImg = insertMarker(depthImg,centerPoint,"circle","Size",10,"Color","yellow");
% 
%                 %figure(2);
%                 %imshow(depthImg);
% 
%                 translation = [depth ...
%                     depth * (centerPoint(1)-obj.Intrinsics.PrincipalPoint(1))/obj.Intrinsics.FocalLength(1) ...
%                     depth * (centerPoint(2)-obj.Intrinsics.PrincipalPoint(2))/obj.Intrinsics.FocalLength(2)];
% 
%                 poseM = eul2rotm([0 0 0]); %eul2rotm([-1.57 0 -1.57]); % from camera - see urdf file
%                 poseM(1:3,4) = translation';
%                 poseM(4,4) = 1;
%     
%                 quat = ur3Pose.Orientation;
%                 worldPoseTr = quat2rotm([quat.W quat.X quat.Y quat.Z]);
%                 worldPoseTr(1:3,4) = [ur3Pose.Position.X;ur3Pose.Position.Y;ur3Pose.Position.Z];
%                 worldPoseTr(4,4) = 1;
%     
%                 paperPose = worldPoseTr * poseM;
% 
%                 markerPresent = true;
%                 disp("Marker detected at");
%                 disp(paperPose);
%             end
%         end

        function [startJointSend, currentJointState_123456] = UR3PoseCallback(obj, jointStateSubscriber)
            
            currentJointState_321456 = (jointStateSubscriber.LatestMessage.Position)'; % Note the default order of the joints is 3,2,1,4,5,6
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

        function [auboi3Robot, ur3Robot, paperModel] = PlotSprayPaintEnvironment(obj)
            workspace = [4 -4 4 -4 3 0];

            auboi3Robot = GetAuboi3();
            PlotAndColourRobot(auboi3Robot, workspace);

            hold on
            paperModel = GetPaperModel(obj);
            ur3Robot = GetUR3(obj);
            mesh_environment = PlaceObject('decimated_enviroment.PLY');
            vertices = get(mesh_environment,'Vertices');
            transformedVertices = [vertices,ones(size(vertices,1),1)] * transl(-2,0,0)';
            transformedVertices = transformedVertices * trotx(pi/2)';
            set(mesh_environment,'Vertices',transformedVertices(:,1:3));
            
            
            
%             [f, v, data] = plyread('whiteEnvelope1.ply','tri');
%             data.vertex.red = data.vertex.x;
%             data.vertex.green = data.vertex.y;
%             data.vertex.blue = data.vertex.z;
%             vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;
%             paper_h = trisurf(f,v(:,1)+ obj.paperPickupCoords(1) ,v(:,3)+obj.paperPickupCoords(2), v(:,2)+obj.paperPickupCoords(3), 'FaceVertexCData', vertexColours, 'EdgeColor', 'interp');

            drawnow();
            axis equal
            view(3)
        end

        function paperCornersAll = GetPaperCorners(obj, paperPose)
            %METHOD1 Summary of this method goes here
            %   Detailed explanation goes here

            topLeft = paperPose*transl(0,-obj.paperSize(1)/2,obj.paperSize(2)/2);
            topRight = paperPose*transl(0,obj.paperSize(1)/2,obj.paperSize(2)/2);
            bottomLeft = paperPose*transl(0,-obj.paperSize(1)/2,-obj.paperSize(2)/2);
            bottomRight = paperPose*transl(0,obj.paperSize(1)/2,-obj.paperSize(2)/2);

            paperCorners = [topLeft; topRight; bottomLeft; bottomRight];
            numPaperCorners = 4;

            for i = 1:numPaperCorners
                paperCornersAll(:,:,i) = paperCorners((i-1)*4+(1:4),1:4);
            end
        end

        function ur3Robot = GetUR3(obj)
            %   create the UR3 at a point in the workspace
            L1 = Link('d',0.1519,'a',0,'alpha',pi/2,'qlim',deg2rad([-360 360]), 'offset',0);
            L2 = Link('d',0,'a',-0.24365,'alpha',0,'qlim', deg2rad([-360 360]), 'offset',0);
            L3 = Link('d',0,'a',-0.21325,'alpha',0,'qlim', deg2rad([-360 360]), 'offset', 0);
            L4 = Link('d',0.11235,'a',0,'alpha',pi/2,'qlim',deg2rad([-360 360]),'offset', 0);
            L5 = Link('d',0.08535,'a',0,'alpha',-pi/2,'qlim',deg2rad([-360,360]), 'offset',0);
            L6 = Link('d',0.0819,'a',0,'alpha',0,'qlim',deg2rad([-360,360]), 'offset', 0);

            ur3Robot = SerialLink([L1 L2 L3 L4 L5 L6],'name','ur3Robot');

            %   plot and colour
            for linkIndex = 0:(ur3Robot.n-1)
                [ faceData, vertexData, plyData{linkIndex + 1} ] = plyread(['ur3link_',num2str(linkIndex),'.ply'],'tri'); %#ok<AGROW>
                ur3Robot.faces{linkIndex + 1} = faceData;
                ur3Robot.points{linkIndex + 1} = vertexData;
            end
            %   use end effector with gripper and spray can
            for linkIndex = ur3Robot.n
                [ faceData, vertexData, plyData{linkIndex + 1} ] = plyread('gripper_with_spray.ply','tri'); %#ok<AGROW>
                ur3Robot.faces{linkIndex + 1} = faceData;
                ur3Robot.points{linkIndex + 1} = vertexData;
            end

            % Display robot
            ur3Robot.plot3d(zeros(1,ur3Robot.n),'noarrow','workspace',[-0.6 0.6 -0.6 0.6 -0.2 1.1]);
            if isempty(findobj(get(gca,'Children'),'Type','Light'))
                camlight
            end
            ur3Robot.delay = 0;

            % Try to correctly colour the arm (if colours are in ply file data)
            for linkIndex = 0:ur3Robot.n
                handles = findobj('Tag', ur3Robot.name);
                h = get(handles,'UserData');
                try
                    h.link(linkIndex+1).Children.FaceVertexCData = [plyData{linkIndex+1}.vertex.red ...
                        , plyData{linkIndex+1}.vertex.green ...
                        , plyData{linkIndex+1}.vertex.blue]/255;
                    h.link(linkIndex+1).Children.FaceColor = 'interp';
                catch ME_1
                    disp(ME_1);
                    continue;
                end
            end
            
            % Move from zero position to position ready to spray
            ur3Robot.base = ur3Robot.base * trotz(-pi/2);
            ur3Robot.base = ur3Robot.base*transl(0,0.5,0.575);
            steps = 20;
            q1 = ur3Robot.getpos();
            q2 = [0 -pi/2 pi/2 pi 0 0];
            
            qMatrix = jtraj(q1,q2,steps);
            
            for i = 1:steps
                ur3Robot.animate(qMatrix(i,:));
                pause(0.01);
            end
        end        

        function readyRealUR3(obj, client, goal, jointStateSubscriber)
            % Move from zero position to position ready to spray
            
            % readyEEPosition = [-0.2133; -0.1942; 0.4809];
            % readyEEAngles = [pi 0 -pi/2];
            % readyEEOrientation = eul2rotm(readyEEAngles);
            % T2 = cat(1,cat(2,readyEEOrientation,readyEEPosition),[0 0 0 1]);
            % steps = 20;
            % q1 = ur3Robot.getpos();
            q2 = [0 -pi/2 pi/2 pi 0 0];
            
            [startJointSend, currentJointState_123456] = UR3PoseCallback(obj, jointStateSubscriber);
            disp(currentJointState_123456)
            goal = UR3GetTrajectory(obj, startJointSend, q2, goal);
            disp(q2)
            
            
            goal.Trajectory.Header.Stamp = jointStateSubscriber.LatestMessage.Header.Stamp + rosduration(obj.bufferSeconds);
            sendGoal(client,goal);
        end

        function MovetoPaperAuboi3Sim(obj, auboi3Robot, paperPose, paperModel, gui, objectCenter, radius)
            x = obj.paperPickupCoords(1);
            y = obj.paperPickupCoords(2);
            z = obj.paperPickupCoords(3);
            T2 = [1 0 0 x; 0 -1 0 y; 0 0 -1 z+0.1865; 0 0 0 1];

            steps = 20;

            q1_hardcoded = auboi3Robot.getpos();
            q2_hardcoded = auboi3Robot.ikcon(T2,q1_hardcoded);
            qMatrix = jtraj(q1_hardcoded,q2_hardcoded,steps);

            for i = 1:steps
                if gui.aubo_stop 
                    auboi3Robot.animate(auboi3Robot.getpos());
                    gui.aubo_stop = 1;
                    while gui.aubo_stop
                        VisualServo(obj,auboi3Robot,gui.aubo_stop,gui.aubo_x,gui.aubo_y,gui.aubo_z,gui.aubo_q1,gui.aubo_q2,gui.aubo_q3,gui.aubo_q4,gui.aubo_q5,gui.aubo_q6);
                    end
                else
                auboi3Robot.animate(qMatrix(i,:));
                drawnow()
                pause(0.2)
%                 [isCollision] = CollisionCheck(obj,auboi3Robot,objectCenter,radius);
                end
            end

            %Move back to spray
            x = paperPose(1,4);
            y = paperPose(2,4);
            z = paperPose(3,4);
            T2 = [0 0 1 x; -1 0 0 y; 0 -1 0 z; 0 0 0 1];

            q1_hardcoded = auboi3Robot.getpos();
            q2_hardcoded = auboi3Robot.ikcon(T2,q1_hardcoded);
            qMatrix = jtraj(q1_hardcoded,q2_hardcoded,steps);

            for i = 1:steps
                auboi3Robot.animate(qMatrix(i,:));
                EEPose = auboi3Robot.fkine(auboi3Robot.getpos());
                paperModel.base = EEPose*transl(0,0,0.1865);
                paperModel.animate(0);
                drawnow()
                pause(0.2)
            end
            
        end

        function PutDownPaperAuboi3Sim(obj, auboi3Robot, paperModel, gui)
            % First go to zero position

            steps = 20;

            q1_hardcoded = auboi3Robot.getpos();
            q2_hardcoded = [0 0 0 0 0 0];
            s = lspb(0,1,steps);
            for i = 1:steps
                qMatrix(i,:) = (1-s(i))*q1_hardcoded + s(i)*q2_hardcoded;
            end

            for i = 1:steps
                auboi3Robot.animate(qMatrix(i,:));
                EEPose = auboi3Robot.fkine(auboi3Robot.getpos());
                paperModel.base = EEPose*transl(0,0,0.1865);
                paperModel.animate(0);
                drawnow()
                pause(0.2)
            end

            % Now put down
            x = obj.paperPutdownCoords(1);
            y = obj.paperPutdownCoords(2);
            z = obj.paperPutdownCoords(3);
            T2 = [1 0 0 x; 0 -1 0 y; 0 0 -1 z+0.1865; 0 0 0 1];

            steps = 20;

            q1_hardcoded = auboi3Robot.getpos();
            q2_hardcoded = auboi3Robot.ikcon(T2,q1_hardcoded);
            s = lspb(0,1,steps);
            for i = 1:steps
                qMatrix(i,:) = (1-s(i))*q1_hardcoded + s(i)*q2_hardcoded;
            end

            for i = 1:steps
                if gui.aubo_stop 
                    auboi3Robot.animate(auboi3Robot.getpos());
                    gui.aubo_stop = 1;
                    while gui.aubo_stop
                        VisualServo(obj,auboi3Robot,gui.aubo_stop,gui.aubo_x,gui.aubo_y,gui.aubo_z,gui.aubo_q1,gui.aubo_q2,gui.aubo_q3,gui.aubo_q4,gui.aubo_q5,gui.aubo_q6);
                    end
                else
                auboi3Robot.animate(qMatrix(i,:));
                EEPose = auboi3Robot.fkine(auboi3Robot.getpos());
                paperModel.base = EEPose*transl(0,0,0.1865);
                paperModel.animate(0);
                drawnow()
                pause(0.2)
%                 [isCollision] = CollisionCheck(obj,auboi3Robot,objectCenter,radius);
                end
            end
        end

        function paperModel = GetPaperModel(obj)
            %creating a simple piece of paper in workspace
            workspace = [-2 2 -2 2 -1 2];
            x = obj.paperPickupCoords(1);
            y = obj.paperPickupCoords(2);
            z = obj.paperPickupCoords(3);

            name = ['objectPaper1_',datestr(now,'yyyymmddTHHMMSSFFF')];

            L1 = Link([pi 0 0 pi/2 0]);
            L1.qlim = [-pi pi]; %L1.qlim = [-0.8 0];
            L1.offset = 0;
            paperModel = SerialLink(L1,'name',name);

            % model.base =  model.base * trotx(pi/2) * troty(pi/2) * transl([0,0,1]);   %transl matrix [y,z,x]
            paperModel.base =paperModel.base * transl([x,y,z]);

            % PlotAndColour
            % Given a robot index, add the glyphs (vertices and faces) and
            % colour them in if data is available
            for linkIndex = 0: paperModel.n
                if  linkIndex ==  paperModel.n
                    [ faceData, vertexData, plyData{linkIndex + 1} ] = plyread('whiteEnvelope.ply','tri'); % #ok<AGROW>
                    paperModel.faces{linkIndex + 1} = faceData;
                    paperModel.points{linkIndex + 1} = vertexData;
                else
                end

            end

            % Display
            paperModel.plot3d(zeros(1, paperModel.n),'noarrow','workspace', workspace);
            if isempty(findobj(get(gca,'Children'),'Type','Light'))
                camlight
            end
            paperModel.delay = 0;
            
            % Try to correctly colour the arm (if colours are in ply file data)
            for linkIndex = paperModel.n
                handles = findobj('Tag',  paperModel.name);
                h = get(handles,'UserData');
                try
                    h.link(linkIndex+1).Children.FaceVertexCData = [plyData{linkIndex+1}.vertex.red ...
                        , plyData{linkIndex+1}.vertex.green ...
                        , plyData{linkIndex+1}.vertex.blue]/255;
                    h.link(linkIndex+1).Children.FaceColor = 'interp';
                catch ME_1
                    disp(ME_1);
                    continue;
                end
            end
        end

        function [nextJointState_123456, movementDuration] = SprayPaintUR3Sim(obj, ur3Robot, paperCornersAll, paperMoving, gui)            
            % Begin when paper is detected and not moving
            steps = 20;
            movementDuration = ones(1,7)*obj.durationSeconds;
            if paperMoving == 0
                % Translate paper corners away from paper by x distance
                distanceFromPaper = 0.2+0.1903; % spray distance + reach of end effector with spray can attached
                goalTopLeft = paperCornersAll(:,:,1)*transl(distanceFromPaper,0,0);
                goalTopRight = paperCornersAll(:,:,2)*transl(distanceFromPaper,0,0);
                goalBottomLeft = paperCornersAll(:,:,3)*transl(distanceFromPaper,0,0);
                goalBottomRight = paperCornersAll(:,:,4)*transl(distanceFromPaper,0,0);
                
                % Make two more waypoints in the centre of the paper
                distx = goalTopRight(1,4)-goalTopLeft(1,4);
                disty = goalBottomRight(2,4)-goalBottomLeft(2,4);
                goalTopCentre = goalTopLeft*transl(distx/2,disty/2,0);
                goalBottomCentre = goalBottomLeft*transl(distx/2,disty/2,0);
                
                paperPoints = [goalTopLeft; goalBottomLeft; goalTopCentre; goalBottomCentre; goalTopRight; goalBottomRight];
                numPaperPoints = 6;
                
                for i = 1:numPaperPoints
                    paperPointsAll(:,:,i) = paperPoints((i-1)*4+(1:4),1:4);
                    paperPointsAll(1:3,1:3,i) = [0 0 -1;1 0 0; 0 -1 0];
                end
                % Spray the paper at all points
                for j = 1:numPaperPoints
                    q1 = ur3Robot.getpos();
                    TR1 = ur3Robot.fkine(q1);
                    TR2 = paperPointsAll(:,:,j);
                    q2 = ur3Robot.ikcon(TR2, q1); % , q1, [1 1 1 0 1 1]
                    nextJointState_123456(:,:,j) = q2;
                    
%                     qMatrix = jtraj(q1,q2,steps);
                    s = lspb(0,1,steps);
                    for i = 1:steps
                      qMatrix(i,:) = (1-s(i))*q1 + s(i)*q2;
                    end
                    movementStart = tic;
                    for i = 1:steps
                        if gui.ur3_stop 
                            auboi3Robot.animate(auboi3Robot.getpos());
                            gui.ur3_stop = 1;
                            while gui.aubo_stop
                                VisualServo(obj,ur3Robot,gui.ur3_stop,gui.ur_x,gui.ur_y,gui.ur_z,gui.ur_q1,gui.ur_q2,gui.ur_q3,gui.ur_q4,gui.ur_q5,gui.ur_q6);
                            end
                        else
                            ur3Robot.animate(qMatrix(i,:));
                            pause(0.1);
%                             [isCollision] = CollisionCheck(obj,auboi3Robot,objectCenter,radius);
                        end
                    end
                    movementDuration(:,j) = toc(movementStart);
                    EEPose = ur3Robot.fkine(ur3Robot.getpos());
                    errorMarginPos = TR2(1:3,4)-EEPose(1:3,4)
                    errorMarginRot = rotm2eul(TR2(1:3,1:3))-rotm2eul(EEPose(1:3,1:3))
                 end
                
                % Return to ready and wait for next paper
                q1 = ur3Robot.getpos();
                q2 = [0 -pi/2 pi/2 pi 0 0];
                
                qMatrix = jtraj(q1,q2,steps);
                
                movementStart = tic;
                for i = 1:steps
                    if gui.ur3_stop 
                        auboi3Robot.animate(auboi3Robot.getpos());
                        gui.ur3_stop = 1;
                        while gui.aubo_stop
                            VisualServo(obj,ur3Robot,gui.ur3_stop,gui.ur_x,gui.ur_y,gui.ur_z,gui.ur_q1,gui.ur_q2,gui.ur_q3,gui.ur_q4,gui.ur_q5,gui.ur_q6);
                        end
                    else
                        ur3Robot.animate(qMatrix(i,:));
                        pause(0.1);
%                         [gui.aubo_stop, gui.ur3_stop] = CollisionCheck(obj,auboi3Robot,objectCenter,radius);
                    end
                end
                movementDuration(:,j+1) = toc(movementStart);
            end
        end

        function [objectCenter,radius] = SpawnSafetySymbol(obj, spawnOn)
            if spawnOn
                objectCenter = [-0.5 0.4 1];
                radius = 0.2;
                mesh_spray = PlaceObject('spray.ply');
                vertices = get(mesh_spray,'Vertices');
                transformedVertices = [vertices,ones(size(vertices,1),1)] * transl(objectCenter)';
%                 transformedVertices = transformedVertices * trotx(pi/2)';
                set(mesh_spray,'Vertices',transformedVertices(:,1:3));
            else
                objectCenter = [10 10 10];
                radius = 0.01;
            end
        end

        function [isCollision] = CollisionCheck(obj,r,objectCenter,radius) %radius usu. 0.2 %add steps = 20
            pause(0.1)
            tr = r.fkine(r.getpos);
            endEffectorToCenterDist = sqrt(sum((objectCenter-tr(1:3,4)').^2));
            if endEffectorToCenterDist <= radius
                disp(['collision detected in ', num2str(endEffectorToCenterDist),'m!!']);
                isCollision = 1;
                %     isCollision = 1;

            else
                disp(['SAFE: End effector to paper object distance (', num2str(endEffectorToCenterDist), 'm)']);
                %         disp(['SAFE: End effector to centre distance (', num2str(endEffectorToCenterDist), 'm) is more than the sphere radius, ' num2str(radius), 'm']);
                %     isCollision = 0;
                isCollision = 0;
            end
        end

        function VisualServo(obj, robot, eStop, x, y, z, q1, q2, q3, q4, q5, q6)
            % 1.1) Set parameters for the simulation
            t = 10;             % Total time (s)
            deltaT = 0.2;      % Control frequency
            steps = t/deltaT;   % No. of steps for simulation
            delta = 2*pi/steps; % Small angle change
            epsilon = 0.1;      % Threshold value for manipulability/Damped Least Squares
            W = diag([1 1 1 0.1 0.1 0.1]);    % Weighting matrix for the velocity vector

            % 1.2) Allocate array data
            m = zeros(steps,1);             % Array for Measure of Manipulability
            qMatrix = zeros(steps,6);       % Array for joint anglesR
            qdot = zeros(steps,6);          % Array for joint velocities
            theta = zeros(3,steps);         % Array for roll-pitch-yaw angles
            position = zeros(3,steps);             % Array for x-y-z trajectory
            positionError = zeros(3,steps); % For plotting trajectory error
            angleError = zeros(3,steps);    % For plotting trajectory error

            % Set up a trajectory using the cartesian points or the joint values
            q1 = robot.getpos();
            TR1 = robot.fkine(q1);
            TR2 = TR1;
            TR2(1:3,4) = [x y z];
            angles = rotm2eul(TR1(1:3,1:3));

            % if robot is already at position within a margin of error,
            % don't move again
            errorMargin = 0.05;
            if abs(TR1(1,4)-x)>errorMargin && abs(TR1(2,4)-y)>errorMargin && abs(TR1(3,4)-z)>errorMargin

                % straight line motion
                s = lspb(0,1,steps);        % Trapezoidal trajectory scalar
                for i = 1:steps
                    position(1,i) = TR1(1,4) + s(i)*((TR2(1,4)-TR1(1,4)));   % Points in x
                    position(2,i) = TR1(2,4) + s(i)*((TR2(2,4)-TR1(2,4)));   % Points in y
                    position(3,i) = TR1(3,4) + s(i)*((TR2(3,4)-TR1(3,4)));   % Points in z
                    theta(1,i) = angles(3);                                 % Roll angle
                    theta(2,i) = angles(2);                                 % Pitch angle
                    theta(3,i) = angles(1);                                 % Yaw angle
                end


                % % 1.3) Set up trajectory, initial pose
                % s = lspb(0,1,steps);                % Trapezoidal trajectory scalar
                % for i=1:steps
                %     x(1,i) = (1-s(i))*0.35 + s(i)*0.35; % Points in x
                %     x(2,i) = (1-s(i))*-0.55 + s(i)*0.55; % Points in y
                %     x(3,i) = 0.5 + 0.2*sin(i*delta); % Points in z
                %     theta(1,i) = 0;                 % Roll angle
                %     theta(2,i) = 5*pi/9;            % Pitch angle
                %     theta(3,i) = 0;                 % Yaw angle
                % end

                T = [rpy2r(theta(1,1),theta(2,1),theta(3,1)) position(:,1);zeros(1,3) 1];          % Create transformation of first point and angle
                q0 = zeros(1,6);                                                            % Initial guess for joint angles
                qMatrix(1,:) = robot.ikcon(T,q1);                                            % Solve joint angles to achieve first waypoint

                % 1.4) Track the trajectory with RMRC
                for i = 1:steps-1
                    T = robot.fkine(qMatrix(i,:));                                           % Get forward transformation at current joint state
                    deltaX = position(:,i+1) - T(1:3,4);                                         	% Get position error from next waypoint
                    Rd = rpy2r(theta(1,i+1),theta(2,i+1),theta(3,i+1));                     % Get next RPY angles, convert to rotation matrix
                    Ra = T(1:3,1:3);                                                        % Current end-effector rotation matrix
                    Rdot = (1/deltaT)*(Rd - Ra);                                                % Calculate rotation matrix error
                    S = Rdot*Ra';                                                           % Skew symmetric!
                    linear_velocity = (1/deltaT)*deltaX;
                    angular_velocity = [S(3,2);S(1,3);S(2,1)];                              % Check the structure of Skew Symmetric matrix!!
                    deltaTheta = tr2rpy(Rd*Ra');                                            % Convert rotation matrix to RPY angles
                    xdot = W*[linear_velocity;angular_velocity];                          	% Calculate end-effector velocity to reach next waypoint.
                    J = robot.jacob0(qMatrix(i,:));                 % Get Jacobian at current joint state
                    m(i) = sqrt(det(J*J'));
                    if m(i) < epsilon  % If manipulability is less than given threshold
                        lambda = (1 - m(i)/epsilon)*5E-2;
                    else
                        lambda = 0;
                    end
                    invJ = inv(J'*J + lambda *eye(6))*J';                                   % DLS Inverse
                    qdot(i,:) = (invJ*xdot)';                                                % Solve the RMRC equation (you may need to transpose the         vector)
                    for j = 1:6                                                             % Loop through joints 1 to 6
                        if qMatrix(i,j) + deltaT*qdot(i,j) < robot.qlim(j,1)                     % If next joint angle is lower than joint limit...
                            qdot(i,j) = 0; % Stop the motor
                        elseif qMatrix(i,j) + deltaT*qdot(i,j) > robot.qlim(j,2)                 % If next joint angle is greater than joint limit ...
                            qdot(i,j) = 0; % Stop the motor
                        end
                    end
                    qMatrix(i+1,:) = qMatrix(i,:) + deltaT*qdot(i,:);                         	% Update next joint state based on joint velocities
                    positionError(:,i) = position(:,i+1) - T(1:3,4);                               % For plotting
                    angleError(:,i) = deltaTheta;                                           % For plotting
                end

                % 1.5) Plot the results
                %             figure(1)
                %             plot3(position(1,:),position(2,:),position(3,:),'k.','LineWidth',1)
                for i = 1:steps
                    robot.animate(qMatrix(i,:))
                    pause(0.01)
                end
                %             for i = 1:6
                %                 figure(2)
                %                 subplot(3,2,i)
                %                 plot(qMatrix(:,i),'k','LineWidth',1)
                %                 title(['Joint ', num2str(i)])
                %                 ylabel('Angle (rad)')
                %                 refline(0,robot.qlim(i,1));
                %                 refline(0,robot.qlim(i,2));
                %
                %                 figure(3)
                %                 subplot(3,2,i)
                %                 plot(qdot(:,i),'k','LineWidth',1)
                %                 title(['Joint ',num2str(i)]);
                %                 ylabel('Velocity (rad/s)')
                %                 refline(0,0)
                %             end
                %
                %             figure(4)
                %             subplot(2,1,1)
                %             plot(positionError'*1000,'LineWidth',1)
                %             refline(0,0)
                %             xlabel('Step')
                %             ylabel('Position Error (mm)')
                %             legend('X-Axis','Y-Axis','Z-Axis')
                %
                %             subplot(2,1,2)
                %             plot(angleError','LineWidth',1)
                %             refline(0,0)
                %             xlabel('Step')
                %             ylabel('Angle Error (rad)')
                %             legend('Roll','Pitch','Yaw')
                %             figure(5)
                %             plot(m,'k','LineWidth',1)
                %             refline(0,epsilon)
                %             title('Manipulability')
            else
                %don't move
            end
        end
        
        function RMRC(obj, robot, x, y, z)
            % 1.1) Set parameters for the simulation
            t = 10;             % Total time (s)
            deltaT = 0.2;      % Control frequency
            steps = t/deltaT;   % No. of steps for simulation
            delta = 2*pi/steps; % Small angle change
            epsilon = 0.1;      % Threshold value for manipulability/Damped Least Squares
            W = diag([1 1 1 0.1 0.1 0.1]);    % Weighting matrix for the velocity vector

            % 1.2) Allocate array data
            m = zeros(steps,1);             % Array for Measure of Manipulability
            qMatrix = zeros(steps,6);       % Array for joint anglesR
            qdot = zeros(steps,6);          % Array for joint velocities
            theta = zeros(3,steps);         % Array for roll-pitch-yaw angles
            position = zeros(3,steps);             % Array for x-y-z trajectory
            positionError = zeros(3,steps); % For plotting trajectory error
            angleError = zeros(3,steps);    % For plotting trajectory error

            % Set up a trajectory using the cartesian points or the joint values
            q1 = robot.getpos();
            TR1 = robot.fkine(q1);
            TR2 = TR1;
            TR2(1:3,4) = [x y z];
            angles = rotm2eul(TR1(1:3,1:3));

            % straight line motion
            s = lspb(0,1,steps);        % Trapezoidal trajectory scalar
            for i = 1:steps
                position(1,i) = TR1(1,4) + s(i)*((TR2(1,4)-TR1(1,4))/steps);   % Points in x
                position(2,i) = TR1(2,4) + s(i)*((TR2(2,4)-TR1(2,4))/steps);   % Points in y
                position(3,i) = TR1(3,4) + s(i)*((TR2(3,4)-TR1(3,4))/steps);   % Points in z
                theta(1,i) = angles(3);                                 % Roll angle
                theta(2,i) = angles(2);                                 % Pitch angle
                theta(3,i) = angles(1);                                 % Yaw angle
            end

            T = [rpy2r(theta(1,1),theta(2,1),theta(3,1)) position(:,1);zeros(1,3) 1];          % Create transformation of first point and angle
            q0 = zeros(1,6);                                                            % Initial guess for joint angles
            qMatrix(1,:) = robot.ikcon(T,q1);                                             % Solve joint angles to achieve first waypoint

            % 1.4) Track the trajectory with RMRC
            for i = 1:steps-1
                T = robot.fkine(qMatrix(i,:));                                           % Get forward transformation at current joint state
                deltaX = position(:,i+1) - T(1:3,4);                                         	% Get position error from next waypoint
                Rd = rpy2r(theta(1,i+1),theta(2,i+1),theta(3,i+1));                     % Get next RPY angles, convert to rotation matrix
                Ra = T(1:3,1:3);                                                        % Current end-effector rotation matrix
                Rdot = (1/deltaT)*(Rd - Ra);                                                % Calculate rotation matrix error
                S = Rdot*Ra';                                                           % Skew symmetric!
                linear_velocity = (1/deltaT)*deltaX;
                angular_velocity = [S(3,2);S(1,3);S(2,1)];                              % Check the structure of Skew Symmetric matrix!!
                deltaTheta = tr2rpy(Rd*Ra');                                            % Convert rotation matrix to RPY angles
                xdot = W*[linear_velocity;angular_velocity];                          	% Calculate end-effector velocity to reach next waypoint.
                J = robot.jacob0(qMatrix(i,:));                 % Get Jacobian at current joint state
                m(i) = sqrt(det(J*J'));
                if m(i) < epsilon  % If manipulability is less than given threshold
                    lambda = (1 - m(i)/epsilon)*5E-2;
                else
                    lambda = 0;
                end
                invJ = inv(J'*J + lambda *eye(6))*J';                                   % DLS Inverse
                qdot(i,:) = (invJ*xdot)';                                                % Solve the RMRC equation (you may need to transpose the         vector)
                for j = 1:6                                                             % Loop through joints 1 to 6
                    if qMatrix(i,j) + deltaT*qdot(i,j) < robot.qlim(j,1)                     % If next joint angle is lower than joint limit...
                        qdot(i,j) = 0; % Stop the motor
                    elseif qMatrix(i,j) + deltaT*qdot(i,j) > robot.qlim(j,2)                 % If next joint angle is greater than joint limit ...
                        qdot(i,j) = 0; % Stop the motor
                    end
                end
                qMatrix(i+1,:) = qMatrix(i,:) + deltaT*qdot(i,:);                         	% Update next joint state based on joint velocities
                positionError(:,i) = position(:,i+1) - T(1:3,4);                               % For plotting
                angleError(:,i) = deltaTheta;                                           % For plotting
            end

            % 1.5) Plot the results
            
            for i = 1:steps
                robot.animate(qMatrix(i,:))
                pause(0.01)
            end
        end
        
        function SprayPaintUR3Real(obj, nextJointState_123456, client, goal, jointStateSubscriber, movementDuration) 
            numPaperPoints = 6;
            % Spray the paper at all points
            for j = 1:numPaperPoints
%                 q1 = ur3Robot.getpos();
%                 TR1 = ur3Robot.fkine(q1);
%                 TR2 = paperPointsAll(:,:,j);
%                 q2 = ur3Robot.ikcon(TR2, q1); % , q1, [1 1 1 0 1 1]

                [startJointSend, currentJointState_123456] = UR3PoseCallback(obj, jointStateSubscriber);
                disp(currentJointState_123456)
                goal = UR3GetTrajectory(obj, startJointSend, nextJointState_123456(:,:,j), goal);
                disp(nextJointState_123456(:,:,j))
                
                
                goal.Trajectory.Header.Stamp = jointStateSubscriber.LatestMessage.Header.Stamp + rosduration(obj.bufferSeconds);
                sendGoal(client,goal);
                pause(movementDuration(:,j));
%                 EEPose = ur3Robot.fkine(ur3Robot.getpos());
%                 errorMarginPos = TR2(1:3,4)-EEPose(1:3,4)
%                 errorMarginRot = rotm2eul(TR2(1:3,1:3))-rotm2eul(EEPose(1:3,1:3))
             end
            
            % Return to ready and wait for next paper
%             q1 = ur3Robot.getpos();
            q2 = [0 -pi/2 pi/2 pi 0 0];
            [startJointSend, currentJointState_123456] = UR3PoseCallback(obj, jointStateSubscriber);
            disp(currentJointState_123456)
            [goal] = UR3GetTrajectory(obj, startJointSend, q2, goal);
            disp(q2)
            
            goal.Trajectory.Header.Stamp = jointStateSubscriber.LatestMessage.Header.Stamp + rosduration(obj.bufferSeconds);
            sendGoal(client,goal);
            pause(movementDuration(:,j+1));
        end


    end
end

