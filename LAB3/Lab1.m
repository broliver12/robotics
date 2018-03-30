%DH = [pi/2 0 0 76; 0 43.22769 0 23.65; pi/2 0 0 0; -pi/2 0 0 43.18; pi/2 0 0 0; 0 0 0 20];
DH = [0 76 0 pi/2; 0 -23.65 43.22769 0; 0 0 0 pi/2; 0 43.18 0 -pi/2; 0 0 0 pi/2; 0 20 0 0];



%delcare robot
bot = mypuma560(DH);

%set up forward plot
theta1 = linspace(0, pi, 200);
theta2 = linspace(0, pi/2, 200);
theta3 = linspace(0, pi, 200);
theta4 = linspace(pi/4, 3*pi/4, 200);
theta5 = linspace(-pi/3, pi/3, 200);
theta6 = linspace(0, 2*pi, 200);
temp = [ theta1; theta2; theta3; theta4; theta5; theta6];
q = temp.';

%plot for section 4.2
figure;
title('Plot for section 4.2');

plot(bot,q)
hold on;

%compute forward kin
o = zeros(200,3);

for i=1:200
    joint = [q(i,1) q(i,2) q(i,3) q(i,4) q(i,5) q(i,6)];    

    %calls to forward here
    H = forward(joint, bot);

    o(i,1)=H(1,4);
    o(i,2)=H(2,4);
    o(i,3)=H(3,4);
end

%create plot
figure;
plot3(o(:,1),o(:,2),o(:,3), 'r');
title('Forward Kinematics (4.3)');
plot(bot, q);





%INVERSE KIN
H2 = [cos(pi/4) -sin(pi/4) 0 20; sin(pi/4) cos(pi/4) 0 23; 0 0 1 15; 0 0 0 1];



%set up inverse plot
dx = linspace(10,30,100);
dy = linspace(10,30,100);
dz = linspace(15, 100, 100);

d = [dx;dy;dz].';

%computer inverse kin
RZPI4 = [cos(pi/4) -sin(pi/4) 0; sin(pi/4) cos(pi/4) 0; 0 0 1];
finalq = zeros(100,6);

for i=1:100
   

    H = zeros(4,4);
    H(1:3,1:3) = RZPI4(1:3,1:3);
    H(:,4) = [dx(i); dy(i); dz(i); 1];
    %Calls to inverse here
    finalq(i,:) = inverse(H, bot);

end

%create plot
figure;
plot3(d(:,1),d(:,2),d(:,3), 'r');
title('Inverse Kinematics(4.4)');
plot(bot, finalq);
