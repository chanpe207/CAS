close all
clf
clear

%% Global variables
workspace = [-5 5 -5 5 0 4];

%% Workspace setup
set(0,'DefaultFigureWindowStyle','docked');
[num_bricks, brick_coords, brick_tr_all, brick_h] = Place_Workspace_Objects();
r = GetLinearUR3();
PlotAndColourRobot(r, workspace);
[gripper1, gripper2, q] = UR3_gripper(r);
axis equal
view (45,45);
%% Start Stacking Bricks
StackBricks(brick_coords, r, gripper1, gripper2, brick_tr_all, q, brick_h, num_bricks);