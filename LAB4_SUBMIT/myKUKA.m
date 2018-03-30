
%declare kuka robot, DH values in mm
function myrobot = myKUKA()

	%[ theta d a alpha]
% 	%DH = [0 40 2.5 pi/2; 0 0 31.5 0; 0 0 3.5 -pi/2; 0 36.5 0 pi/2; 0 0 0 pi/2; 0 16.144 -29.623 0];
    DH = [0 400 25 pi/2; 
          0 0 315 0; 
          0 0 35 pi/2; 
          0 365 0 -pi/2; 
          0 0 0 pi/2; 
          0 161.44 -156 0];
      
     
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