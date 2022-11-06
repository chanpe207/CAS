function grabPaper(x,y,z,r, brick_goal_coords, brick_tr_all, brick_h)%function grabPaper(r, q, brick_goal_coords, brick_tr_all, brick_h)%function grabPaper(r, brick_tr_all, q, brick_h)%function grabPaper(brick_coords, r, gripper1, gripper2, brick_tr_all, q, brick_h)
        %% variables
        [f, v, data] = plyread('whiteEnvelope1.ply','tri');
        data.vertex.red = data.vertex.x;
        data.vertex.green = data.vertex.y;
        data.vertex.blue = data.vertex.z;
        vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;
        brick_stuck = false;
        %% change this into a parameter [changed! :)]
        % brick_goal_coords = [-0.6200 0.4000 0.3440]; % this can also be predefined in the main script

        %%
        brick_goaltr_all = zeros(4,4,1); % was (4,4,9)
        brick_goaltr_all(:,:,1) = transl(brick_goal_coords(1,1),brick_goal_coords(1,2),brick_goal_coords(1,3))*troty(pi); %(1,3) - 1 was count of obj

        %%
        g_distance = -0.14;

        %% rewrite bottom section
% %         EEmatrix = r.fkine(r.getpos());
% %         currentState = brick_tr_all(:,:,brick_number_index)*transl(0,0,g_distance);
% % notnow


        %% place object paper






        %% here ATM
%             for brick_number_index = 1:size(brick_goal_coords(:,1))
                brick_number_index = 1
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
                        %% Move auboi3 to get paper
                        steps = 100;
                        q1 = r.getpos();
                        T2 = current_brick_tr;
                        q2 = r.ikcon(T2);
                        qMatrix = jtraj(q1,q2,steps);

                        figure(1)
                        for i = 1:steps
                            r.animate(qMatrix(i,:));
                            drawnow();
                        end

                        current_brick_tr = EE_pose*transl(0,0,-g_distance);
                        brick_stuck = true;

                    end
        %%
        %% 

                     if brick_stuck == true
                        %% determine the end goal for each brick
                        current_brick_goaltr = brick_goaltr_all(:,:,brick_number_index)*transl(0,0,g_distance);
                        %% Move UR3 to place brick at goal
                        steps = 100;
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

                            % determine the location of the end effector after each step
                            EE_pose = r.fkine(r.getpos());
                            % brick location animated to end effector
                            current_brick_tr = EE_pose*transl(0,0,-g_distance);
                            delete(brick_h(brick_number_index));
                            h.surf = trisurf(f,v(:,1)+ current_brick_tr(1,4),v(:,2)+current_brick_tr(2,4), v(:,3)+current_brick_tr(3,4), ...
                                'FaceVertexCData', vertexColours, 'EdgeColor', 'interp');
                            brick_h(brick_number_index) = h.surf;
                            drawnow();
                        end

                        %determine if end effector has reached end goal
                        EE_pose = r.fkine(r.getpos());
                        error_margin = EE_pose(:,4)- current_brick_goaltr(:,4)
                        gripper_is_close = all(abs(error_margin) <= [0.05;0.05;0.05;0]);
                        brick_stuck = false;
                    else
                        %if brick is not stuck
                     end
end
