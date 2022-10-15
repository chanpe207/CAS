classdef SprayPaintingBots
    %SPRAYPAINTINGBOTS Summary of this class goes here
    %   A dual cobot system designed to spray paint pieces of paper
    
    properties
        CameraRgbSub;
        CameraDepthSub;
        PoseSub;

        MarkerImg;
        Intrinsics;
        MarkerSize = 0.09;
        paperSize = [0.2 0.3];
    end
    
    methods
        function obj = SprayPaintingBots()
            %SPRAYPAINTINGBOTS Construct an instance of this class
            %   Detailed explanation goes here
            focalLength    = [554 554]; 
            principalPoint = [320 240];
            imageSize      = [480 640]; %change these to the camera required
            obj.Intrinsics = cameraIntrinsics(focalLength,principalPoint,imageSize);
            obj.MarkerImg = rgb2gray (imread('../meshes/0.png'));
            obj.CameraRgbSub = rossubscriber();
            obj.CameraDepthSub = rossubscriber();
            obj.PoseSub = rossubscriber();

        end

        function StartWorkFlow
            
            depthMsg = CameraDepthCallback(obj);
            rbgImgMsg = receive(obj.CameraRgbSub);
            ur3Pose = UR3PoseCallback(obj);
            [markerPresent,paperPose] = AnalyseImage(obj, rgbImgMsg, depthMsg, ur3Pose);
            paperCoords = GetPaperCoords(obj, paperSize, paperPose);

        end
        
        function outputArg = CameraRGBCallback(obj)
            rbgImgMsg = receive(obj.CameraRgbSub);
        end

        function depthMsg = CameraDepthCallback(obj)
            depthMsg = receive(obj.CameraDepthSub);
        end

        function ur3Pose = UR3PoseCallback(obj)
            ur3Pose = receive(obj.PoseSub);
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

        function paperCoords = GetPaperCoords(obj, paperSize, paperPose)
            %METHOD1 Summary of this method goes here
            %   Detailed explanation goes here
            topLeft = paperPose;
            topRight = topLeft*transl(paperSize(1),0,0);
            bottomLeft = topLeft*transl(0,0,-paperSize(2));
            bottomRight = bottomLeft*transl(paperSize(1),0,0);

            paperCoords = [topLeft; topRight; bottomLeft; bottomRight];
        end
    end
end

