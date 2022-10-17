% setup robot
close all;clear all;clc
workspace = [2 -2 2 -2 2 -1];
r = GetAuboi3()
PlotAndColourRobot(r, workspace)
endEffectorPos = r.fkine(r.getpos())

% robot arm moves from origin to grab paper
% returns from location of paper to default position
% the below is the location of the paper in space when robot is at default location
x = endEffectorPos(1,4) %- 0.1
y = endEffectorPos(2,4)
z = endEffectorPos(3,4)

% spawning paper at this location: paper origin (pO)
x_pO = x + 0.2
y_pO = y + 0.2
z_pO = z + 0.2
hold on
loadPaperModel = onepaper(x_pO,y_pO,z_pO)

paperPos = loadPaperModel.fkine(loadPaperModel.getpos())

view(3)
axis equal
%% Move Auboi3 to get paper                
steps = 20;
q1 = r.getpos();
T2 = transl([x_pO,y_pO,z_pO-0.05]);
q2 = r.ikcon(T2);
qMatrix = jtraj(q1,q2,steps);

for i = 1:20
    r.animate(qMatrix(i,:));
    drawnow()
end

%% retrieve to position

%for robot
q1_new = r.getpos();
T3 = transl(x,y,z);
q2_new = r.ikcon(T3);
qMatrix2 = jtraj(q1_new,q2_new,steps);

%for paper


for i = 1:20
    r.animate(qMatrix2(i,:));
    drawnow()

    for j = x_pO:x
    loadPaperModel.base =  loadPaperModel.base * transl([x,y,z]);
    loadPaperModel.animate(paperPos);
    r.base = r.base * transl([0,0,0.025]);
    r.animate(r.getpos());
    end
    
    
end
                
            
%% moving model
% step = 0.05;
% x = endEffectorPos(1,4) - 0.1
% y = endEffectorPos(2,4)
% z = endEffectorPos(3,4)
% for i=1:step:2
%     loadPaperModel.base =  loadPaperModel.base * transl([0,0,0.025]);
%     loadPaperModel.animate(paperPos);
% 
%     r.base = r.base * transl([0,0,0.025]);
%     r.animate(r.getpos());
%     
%     pause(0.1)
% end
% 
% % translate AND rotate model
