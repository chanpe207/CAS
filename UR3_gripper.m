function [gripper1, gripper2, q] = UR3_gripper(r)
hold on
    qGrippers = 0.5336;
    q=[qGrippers,pi/2-qGrippers];
    L1 = Link('d',0,'a',0.058,'alpha',0,'qlim',[0.5336 pi/2]);
    L2 = Link('d',0,'a',0.046,'alpha',0,'qlim',[pi/2-qGrippers pi/2]);
%     L3 = Link('d',0,'a',0.05,'alpha',0,'qlim',[0 pi]);
    
    gripper1 = SerialLink([L1 L2],'name','1');
    
    gripper1.base = r.fkine(r.getpos())* transl(0,0.013,0.06);
    gripper1.base = gripper1.base* trotx(pi/2)* troty(pi/2);
    
    gripper1.plot(q, 'noarrow');
    % gripper1.teach();
    
    gripper2 = SerialLink([L1 L2],'name','2');
    
    gripper2.base = r.fkine(r.getpos())* transl(0,-0.013,0.06);
    gripper2.base = gripper2.base*trotx(-pi/2)*trotz(pi)* troty(pi/2);
    gripper2.plot(q, 'noarrow');
end
