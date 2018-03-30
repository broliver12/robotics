%DH = [pi/2 0 0 76; 0 43.22769 0 23.65; pi/2 0 0 0; -pi/2 0 0 43.18; pi/2 0 0 0; 0 0 0 20];
DH = [0 76 0 pi/2; 0 -23.65 43.22769 0; 0 0 0 pi/2; 0 43.18 0 -pi/2; 0 0 0 pi/2; 0 20 0 0];




bot = mypuma560(DH);

theta1 = linspace(0, pi, 200);
theta2 = linspace(0, pi/2, 200);
theta3 = linspace(0, pi, 200);
theta4 = linspace(pi/4, 3*pi/4, 200);
theta5 = linspace(-pi/3, pi/3, 200);
theta6 = linspace(0, 2*pi, 200);

temp = [ theta1; theta2; theta3; theta4; theta5; theta6];
q = temp.';
%plot(bot, q);

o = zeros(200,3);
for i=1:200
joint = [q(i,1) q(i,2) q(i,3) q(i,4) q(i,5) q(i,6)];    
if i==1
    H = forward(joint, bot)
end
H = forward(joint, bot);
o(i,1)=H(1,4);
o(i,2)=H(2,4);
o(i,3)=H(3,4);
end


plot3(o(:,1),o(:,2),o(:,3), 'r')
hold on
%plot(bot, q);

res = inv_script(bot);