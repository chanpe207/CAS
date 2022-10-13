function r = GetAuboi3()
    clf
% Alternative Link system
%     L1 = Link('theta',pi,'a',0,'alpha',pi/2,'qlim',deg2rad([-360 360]), 'offset',0);
%     L2 = Link('d',0.1519,'a',0,'alpha',pi/2,'qlim',deg2rad([-360 360]), 'offset',0);
%     L3 = Link('d',0,'a',-0.24365,'alpha',0,'qlim', deg2rad([-360 360]), 'offset',0);
%     L4 = Link('d',0,'a',-0.21325,'alpha',0,'qlim', deg2rad([-360 360]), 'offset', 0);
%     L5 = Link('d',0.11235,'a',0,'alpha',pi/2,'qlim',deg2rad([-360 360]),'offset', 0);
%     L6 = Link('d',0.08535,'a',0,'alpha',-pi/2,'qlim',deg2rad([-360,360]), 'offset',0);
%     L7 = Link('d',0.0819,'a',0,'alpha',0,'qlim',deg2rad([-360,360]), 'offset', 0);

    % Create the UR3 model mounted on a linear rail
    L(1) = Link([pi     0       pi/2       0    1]); % PRISMATIC Link
    L(2) = Link([0      0.157   0     pi/2    0]);
    L(3) = Link([0      0       -0.266 0       0]);
    L(4) = Link([0      0       -0.2565 0       0]);
    L(5) = Link([0      0.1025  0       pi/2    0]);
    L(6) = Link([0      0.1025 0       -pi/2	0]);
    L(7) = Link([0      0.094  0       0       0]);
    
    % Incorporate joint limits
    L(1).qlim = [-0.8 0];
    L(2).qlim = [-360 360]*pi/180;
    L(3).qlim = [-90 90]*pi/180;
    L(4).qlim = [-170 170]*pi/180;
    L(5).qlim = [-360 360]*pi/180;
    L(6).qlim = [-360 360]*pi/180;
    L(7).qlim = [-360 360]*pi/180;

    L(3).offset = -pi/2;
    L(5).offset = -pi/2;


    r = SerialLink(L,'name','r');

    scale = 0.25;
    
    % Rotate robot to the correct orientation
%     r.base = r.base * trotx(pi/2) * troty(pi/2);
    r.base = r.base * transl(0,0.344,0);

    q = zeros(size(L));

    r.plot(q,'workspace',[-2 2 -2 2 -1 2],'scale',0.5);

end