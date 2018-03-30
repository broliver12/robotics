function myrobot = mypuma560(DH)


    %create robot structure using provided DH table
    L1 = Link([DH(1, :)]);
    L2 = Link([DH(2, :)]);
    L3 = Link([DH(3, :)]);
    L4 = Link([DH(4, :)]);
    L5 = Link([DH(5, :)]);
    L6 = Link([DH(6, :)]);
    
    
    %r = robot({L1 L2 L3 L4 L5 L6});
    myrobot = SerialLink([L1 L2 L3 L4 L5 L6], 'name', 'bot1')
   


end


% 
%     L1 = Link([pi/2 0 0 76]);
%     L2 = Link([0 43.22769 0 23.65 ]);
%     L3 = Link([pi/2 0 0 0]);
%     L4 = Link([-pi/2 0 0 43.18]);
%     L5 = Link([pi/2 0 0 0]);
%     L6 = Link([0 0 0 20]);