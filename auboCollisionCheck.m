function auboCollisionCheck(r,objectCenter,radius)%radius usu. 0.2 %add steps = 20
    pause(0.1)
    tr = r.fkine(r.getpos);
    endEffectorToCenterDist = sqrt(sum((objectCenter-tr(1:3,4)').^2));
    if endEffectorToCenterDist <= radius
        disp(['collision detected in ', num2str(endEffectorToCenterDist),'m!!']);
        %     isCollision = 1;
        
    else
        disp(['SAFE: End effector to paper object distance (', num2str(endEffectorToCenterDist), 'm)']);
%         disp(['SAFE: End effector to centre distance (', num2str(endEffectorToCenterDist), 'm) is more than the sphere radius, ' num2str(radius), 'm']);
        %     isCollision = 0;
        
    end
end
% % function auboCollisionCheck(auboi3Robot)
% % %%
% % %     set(0,'DefaultFigureWindowStyle','docked')
% % %     workspace = [-0.5 1.5 -0.5 1.5 -1 1]
% % %     hold on
% % %     
% % %     %create robot
% % %     r = GetAuboi3;
% % %     PlotAndColourRobot(r,workspace);
% % %     hold on
% % %     axis equal
% %     hold  on
% %     r = auboi3Robot;
% %     
% %     %note: change up order crerating gemoetric object first then robot
% %     paperPickupCoords = [-0.605 0.3 0.575];% Paper Coordinates       
% %     
% %     %const
% %     % circleCenter = [r.base(1,4),r.base(2,4),r.base(3,4)] % robotbasecoord
% %     circleCenter = paperPickupCoords
% %     radius = 0.01
% %     steps = 20
% % 
% % % % % % %     %qMatrix + ctraj
% % % % % % %     q1 = r.getpos()
% % % % % % %     q2 = r.ikcon( transl(-0.605, 0.3, 0.575) , q1 ) %paperPickupCoords
% % % % % % %     s = lspb(0,1,steps);
% % % % % % %     for i = 1:steps
% % % % % % %       qMatrix(i,:) = (1-s(i))*q1 + s(i)*q2;
% % % % % % %     end
% % 
% % % qMatrix + jtraj
% %     q1 = r.getpos()
% %     q2 = r.ikcon( transl(-0.605, 0.3, 0.575) , q1 ) %paperPickupCoords
% %     qMatrix = jtraj(q1,q2,steps);
% %     
% % %     for i = 1:steps
% % %         auboi3Robot.animate(qMatrix(i,:));
% % %         EEPose = auboi3Robot.fkine(auboi3Robot.getpos());
% % %         paperModel.base = EEPose*transl(0,0,0.1865);
% % %         paperModel.animate(0);
% % %         drawnow()
% % %         pause(0.2)
% % %     end
% % 
% %     
% %     for j = 1:steps
% %         r.plot(qMatrix(:,:,j))
% %         drawnow();
% %         CheckCollision(r,circleCenter,radius);
% %     %     if CheckCollision(r,circleCenter,radius) == 1
% %     %         disp('UNSAFE: Robot stoppped')
% %     %         break
% %     %     else
% %     %         disp('safe')
% %     %     end
% %     end
% % 
% %     function CheckCollision(r,circleCenter,radius)
% %         pause(0.1)
% %         tr = r.fkine(r.getpos);
% %         endEffectorToCenterDist = sqrt(sum((circleCenter-tr(1:3,4)').^2));
% %         if endEffectorToCenterDist <= radius
% %             disp('collision detected!!');
% %             %     isCollision = 1;
% %         else
% %             disp(['SAFE: End effector to centre distance (', num2str(endEffectorToCenterDist), 'm) is more than the sphere radius, ' num2str(radius), 'm']);
% % 
% %             %     isCollision = 0;
% %         end
% %     end
% % 
% % 
% % end
% % % function CheckCollision(robot, sphereCenter, radius)%[1 1 0]
% % % % function isCollision = CheckCollision(robot, sphereCenter, radius)
% % % 
% % %     pause(0.1)
% % %     tr = robot.fkine(robot.getpos);
% % %     %endEffectorToCenterDist = sqrt(sum((sphereCenter-tr(1:3,4)').^2));
% % % %     endEffectorToCenterDist = transl(-0.605, 0.3, 0.575) - robot.fkine(robot.getpos) %4x4 current ee pos
% % %     endEffectorToCenterDist = sqrt(sum(([-0.605, 0.3, 0.575]-tr(1:3,4)').^2));
% % %     if endEffectorToCenterDist <= radius
% % %         disp('Oh no a collision!');
% % % %         isCollision = 1;
% % %     else
% % %         disp(['SAFE: End effector to sphere centre distance (', num2str(endEffectorToCenterDist), 'm) is more than the sphere radius, ' num2str(radius), 'm']);
% % % %         isCollision = 0;
% % %     end
% % % 
% % % end