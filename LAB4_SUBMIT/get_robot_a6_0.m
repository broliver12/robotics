
%declare kuka robot, DH values in mm
function myrobot = get_robot_a6_0()

	%[ theta d a alpha]
    %a6 set to 0
   DH = [0 400 25 pi/2; 0 0 315 0; 0 0 35 pi/2; 0 365 0 -pi/2; 0 0 0 pi/2; 0 161.44 0 0];

    %create robot structure using provided DH table
    L1 = Link([DH(1, :)]);
    L2 = Link([DH(2, :)]);
    L3 = Link([DH(3, :)]);
    L4 = Link([DH(4, :)]);
    L5 = Link([DH(5, :)]);
    L6 = Link([DH(6, :)]);
    
    
    %r = robot({L1 L2 L3 L4 L5 L6});
    myrobot = SerialLink([L1 L2 L3 L4 L5 L6], 'name', 'bot_a6_0')
   


end