function StackBricks(brick_coords, r, gripper1, gripper2, brick_tr_all, q, brick_h, num_bricks)
%% variables
g_distance = -0.14;
[f, v, data] = plyread('HalfSizedRedGreenBrick.ply','tri');
vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;
brick_stuck = false;
brick_thickness = 0.03354;
brick_goal_coords = [   -0.6200    0.4000    0.3440+brick_thickness; ...
                        -0.4800    0.4000    0.3440+brick_thickness; ...
                        -0.3400    0.4000    0.3440+brick_thickness; ...
                        -0.3400    0.4000    0.3440; ...
                        -0.4800    0.4000    0.3440; ...
                        -0.6200    0.4000    0.3440; ...
                        -0.6200    0.4000    0.3440+(brick_thickness*2); ...
                        -0.4800    0.4000    0.3440+(brick_thickness*2); ...
                        -0.3400    0.4000    0.3440+(brick_thickness*2)];
brick_goaltr_all = zeros(4,4,9);
for i = 1:num_bricks
    brick_goaltr_all(:,:,i) = transl(brick_goal_coords(i,1),brick_goal_coords(i,2),brick_goal_coords(i,3))*troty(pi);
end
%%
%if the bricks are not stacked, stack them
bricks_stacked = any(any(brick_goal_coords ~= brick_coords));
if bricks_stacked == 1
    % find which bricks have not been stacked
    for brick_number_index = 1:size(brick_goal_coords(:,1))
        %if brick brick_number_index is not at the goal stack the brick, otherwise move on
        if any(brick_goal_coords(brick_number_index,:)-brick_coords(brick_number_index,:)) ~= 0
            % Get pose of end effector
            EE_pose = r.fkine(r.getpos());
            %detect when brick brick_number_index has been reached
            current_brick_tr = brick_tr_all(:,:,brick_number_index)*transl(0,0,g_distance);
            error_margin = EE_pose(:,4)- current_brick_tr(:,4);
            % if brick brick_number_index has been reached (error margin very small),
            gripper_is_close = all(abs(error_margin) <= [0.05;0.05;0.05;0]);
            if gripper_is_close == true && brick_stuck==false
                % 'stick' brick to end effector
                current_brick_tr = EE_pose*transl(0,0,-g_distance);
                brick_stuck = true;
               
            % if UR3 is not at a brick
            elseif gripper_is_close == false && brick_stuck==false
                %% Move UR3 to get brick
                steps = 20;
                q1 = r.getpos();
                T2 = current_brick_tr;
                q2 = r.ikcon(T2);
%                 s = lspb(0,1,steps);
%                 for i = 1:steps
%                     qMatrix(i,:) = (1-s(i))*q1 + s(i)*q2;
%                 end
                qMatrix = jtraj(q1,q2,steps);
%                 velocity = zeros(steps,7);
%                 acceleration  = zeros(steps,7);
%                 for i = 2:steps
%                     velocity(i,:) = qMatrix(i,:) - qMatrix(i-1,:);                          % Evaluate relative joint velocity
%                     acceleration(i,:) = velocity(i,:) - velocity(i-1,:);                    % Evaluate relative acceleration
%                 end
% 
%                 figure(2)                %plot all the joint velocities
%                 for i = 1:7
%                     subplot(4,2,i)
%                     plot(velocity(:,i),'k','LineWidth',1)
%                     title(['Joint ', num2str(i)])
%                     xlabel('Step')
%                     ylabel('Joint Velocity')
%                 end
%                 
%                 figure(3)              %plot all the joint accelerations
%                 for i = 1:7
%                     subplot(4,2,i)
%                     plot(acceleration(:,i),'k','LineWidth',1)
%                     title(['Joint ', num2str(i)])
%                     xlabel('Step')
%                     ylabel('Joint Acceleration')
%                 end
                
                figure(1)
                for i = 1:steps
                    r.animate(qMatrix(i,:));
                    gripper1.base = r.fkine(r.getpos())* transl(0,0.013,0.06);
                    gripper1.base = gripper1.base* trotx(pi/2)* troty(pi/2);
                    gripper2.base = r.fkine(r.getpos())* transl(0,-0.013,0.06);
                    gripper2.base = gripper2.base*trotx(-pi/2)*trotz(pi)* troty(pi/2);
                    gripper1.animate(q);
                    gripper2.animate(q);
                end
                current_brick_tr = EE_pose*transl(0,0,-g_distance);
                brick_stuck = true;
                
            end
                %if brick has been stuck,
            if brick_stuck == true
                %% close grippers
                steps = 3;
                g_q1 = q;
                qGrippers = 1.1;
                g_q2=[qGrippers,pi/2-qGrippers];
%                 s = lspb(0,1,steps);
%                 for i = 1:steps
%                     qMatrix_Gripper(i,:) = (1-s(i))*g_q1 + s(i)*g_q2;
%                 end
                qMatrix_Gripper = jtraj(g_q1,g_q2,steps);                
    
                for i = 1:steps
                    gripper1.animate(qMatrix_Gripper(i,:));
                    gripper2.animate(qMatrix_Gripper(i,:));
                end
                %% determine the end goal for each brick
                current_brick_goaltr = brick_goaltr_all(:,:,brick_number_index)*transl(0,0,g_distance);
                %% Move UR3 to place brick at goal
                steps = 20;
                q1 = r.getpos();
                T2 = current_brick_goaltr;
                q2 = r.ikcon(T2);
%                 s = lspb(0,1,steps);
%                 for i = 1:steps
%                     qMatrix(i,:) = (1-s(i))*q1 + s(i)*q2;
%                 end
                qMatrix = jtraj(q1,q2,steps);

                %simulate end effector as it approaches end goal
                for i = 1:steps
                    r.animate(qMatrix(i,:));
                    gripper1.base = r.fkine(r.getpos())* transl(0,0.013,0.06);
                    gripper1.base = gripper1.base* trotx(pi/2)* troty(pi/2);
                    gripper2.base = r.fkine(r.getpos())* transl(0,-0.013,0.06);
                    gripper2.base = gripper2.base*trotx(-pi/2)*trotz(pi)* troty(pi/2);
                    gripper1.animate(g_q2);
                    gripper2.animate(g_q2);
                    % determine the location of the end effector after each step
                    EE_pose = r.fkine(r.getpos());
                    % brick location animated to end effector
                    current_brick_tr = EE_pose*transl(0,0,-g_distance);
                    delete(brick_h(brick_number_index));
                    h.surf = trisurf(f,v(:,1)+ current_brick_tr(1,4),v(:,2)+current_brick_tr(2,4), v(:,3)+current_brick_tr(3,4), ...
                        'FaceVertexCData', vertexColours, 'EdgeColor', 'interp');
                    brick_h(brick_number_index) = h.surf;
                end
                
                %determine if end effector has reached end goal
                EE_pose = r.fkine(r.getpos());
                error_margin = EE_pose(:,4)- current_brick_goaltr(:,4)
                gripper_is_close = all(abs(error_margin) <= [0.05;0.05;0.05;0]);
                brick_stuck = false;
                    %brick no longer moves with end effector
                    %% open grippers
                    steps = 3;
                    g_q1 = g_q2;
                    qGrippers = 0.5336;
                    g_q2=[qGrippers,pi/2-qGrippers];
    %                 s = lspb(0,1,steps);                      %Trapezoidal Velocity Profile
    %                 for i = 1:steps
    %                     qMatrix_Gripper(i,:) = (1-s(i))*g_q1 + s(i)*g_q2;
    %                 end
                    qMatrix_Gripper = jtraj(g_q1,g_q2,steps); %quintic velocity profile
            
                    for i = 1:steps
                        gripper1.animate(qMatrix_Gripper(i,:));
                        gripper2.animate(qMatrix_Gripper(i,:));
                    end
%                 if gripper_is_close == true && brick_stuck==true
%                     %if end goal has been reached, 'unstick' brick from end effector
%                     
%                 end
                    
                %end effector approaches next brick
                %loop back to beginning
            else
                %if brick is not stuck
            end
            
        else
        % the brick is at the goal already
        % move onto next brick
        end
    end
else
    
end

