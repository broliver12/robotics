
%delcare robot
close all;
myrobot = myKUKA();
%declare modified robot (a6=0)
a6_0_bot = get_robot_a6_0();

%%%%%%%%%%%%%%%%%PREPARATION%%%%%%%%%%%%%%%%%%%%%

%declare obstacles
setupobstacle_lab4rep;

%test repulsive force
tau = rep([pi/10,pi/12,pi/6,pi/2,pi/2,-pi/6],a6_0_bot, prepobs{1})

%test motion planning and plot
p1 = [620 375 50];
p2 = [620 -375 50];
R=[0 0 1;0 -1 0;1 0 0];
H1=[R p1';zeros(1,3) 1];
H2=[R p2';zeros(1,3) 1];
q1 = inverse_kuka(H1, myrobot);
q2 = inverse_kuka(H2, myrobot);
qref = motionplan(q1, q2, 0, 10, a6_0_bot, prepobs);

hold on
axis([-1000 1000 -1000 1000 0 2000])
view(-32,50)
plotobstacle(prepobs);
t=linspace(0,10,300);
qa=ppval(qref,t').';
q6lin = linspace(q1(6),q2(6),300);
qa(:,6) = q6lin.';
plot(myrobot,qa);
hold off

%%%%%%%%%%%%%%%End of Preparation%%%%%%%%%%%%%%%%%%%







%%%%%%%%%%%%%LAB WORK%%%%%%%%%%%%%%%%%%%%%%%
 R=[0 0 1;0 -1 0;1 0 0];
 t=linspace(0,10,300);    
 q_home = [0 pi/2 0 0 pi/2 0];
 setupobstacle;

 %%%%%%%%%%SECTION 4.3%%%%%%%%%%%%%%%%
% 
% p0 = [370 -440 150];
% p1 = [370 -440 20];
% p2 = [750 -220 225];
% p3 = [620 350 225];
% 
% 
% 
% H0 = [R p0';zeros(1,3) 1];
% H1 = [R p1';zeros(1,3) 1];
% H2 = [R p2';zeros(1,3) 1];
% H3 = [R p3';zeros(1,3) 1];
% 
% q0 = inverse_kuka(H0, myrobot);
% q1 = inverse_kuka(H1, myrobot);
% q2 = inverse_kuka(H2, myrobot);
% q3 = inverse_kuka(H3, myrobot);
% 
% %do motion planning
% qref0 = motionplan(q_home, q0, 0, 10, a6_0_bot, obs);
% qref1 = motionplan(q0, q1, 0, 10, a6_0_bot, obs);
% qref2 = motionplan(q1, q2, 0, 10, a6_0_bot, obs);
% 
% qref3 = motionplan(q2, q3, 0, 10, a6_0_bot, obs);
% 
% 
% q0_p = ppval(qref0,t)';
% q1_p = ppval(qref1,t)';
% q2_p = ppval(qref2,t)';
% q3_p = ppval(qref3,t)';
% 
% %linearize q6
% q6lin = linspace(q_home(6),q0(6),300);
% q0_p(:,6) = q6lin.';
% 
% q6lin = linspace(q0(6),q1(6),300);
% q1_p(:,6) = q6lin.';
% 
% q6lin = linspace(q1(6),q2(6),300);
% q2_p(:,6) = q6lin.';
% 
% q6lin = linspace(q2(6),q3(6),300);
% q3_p(:,6) = q6lin.';
% 
% %concatenate
% q_plan = [q0_p; q1_p; q2_p; q3_p;];
% 
% %move robot
% vel = 0.01;
% % for i=1:1200
% % 
% % 	setAngles(q_plan(i,:),vel);
% % end
% 
% % open gripper
% setGripper(0);
% % move to above object from home
% for i = 1:300
%     setAngles(q0_p(i,:), 0.01);
% end
% % move straigt down to target object
% setAngles(q1, 0.01);
% % pick up object
% setGripper(1);
% 
% % move to p2
% for i = 1:300
%     setAngles(q2_p(i,:), 0.01);
% end
% 
% % move to drop off position
% for i = 1:300
%     setAngles(q3_p(i,:), 0.01);
% end
% % drop off object
% setGripper(0);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%




%%%%%%%%%%%%%Creative Motion Plan%%%%%%%%%%%%%%%%%%%%%%%

%%Pick up 2 blocks and place them in the cup at a tricky location in between pillars


%above c1
p0 = [350 -220 150];

%middle
p1 = [620 -220 100];

%above c2
p2 = [350 -440 150];

p4 = [350 -220 20];
p5 = [350 -440 20];

H0 = [R p0';zeros(1,3) 1];
H1 = [R p1';zeros(1,3) 1];
H2 = [R p2';zeros(1,3) 1];
H4 = [R p4';zeros(1,3) 1];
H5 = [R p5';zeros(1,3) 1];

q0 = inverse_kuka(H0, myrobot);
q1 = inverse_kuka(H1, myrobot);
q2 = inverse_kuka(H2, myrobot);
q4 = inverse_kuka(H4, myrobot);
q5 = inverse_kuka(H5, myrobot);

%do motion planning

qref0 = motionplan(q_home, q0, 0, 10, a6_0_bot, obs);
qref1 = motionplan(q0, q1, 0, 10, a6_0_bot, obs);
qref2 = motionplan(q1, q2, 0, 10, a6_0_bot, obs);

qref3 = motionplan(q2, q1, 0, 10, a6_0_bot, obs);


q0_p = ppval(qref0,t)';
q1_p = ppval(qref1,t)';
q2_p = ppval(qref2,t)';
q3_p = ppval(qref3,t)';

q6lin = linspace(q_home(6),q0(6),300);
q0_p(:,6) = q6lin.';

q6lin = linspace(q0(6),q1(6),300);
q1_p(:,6) = q6lin.';

q6lin = linspace(q1(6),q2(6),300);    
q2_p(:,6) = q6lin.';

q6lin = linspace(q2(6),q1(6),300);
q3_p(:,6) = q6lin.';


vel = 0.01;


% open gripper
setGripper(0);
% move to above object from home
for i = 1:300
    setAngles(q0_p(i,:), vel);
end
% % move straigt down to target object
setAngles(q4, vel);
% % pick up object
setGripper(1);
setAngles(q0, vel);

% move to p2
for i = 1:300
    setAngles(q1_p(i,:), vel);
end%q3 = inverse_kuka(H3, myrobot);

setGripper(0);
% move to drop off position
for i = 1:300
    setAngles(q2_p(i,:), vel);
end
setAngles(q5, vel);
% % pick up object
setGripper(1);
setAngles(q2, vel)
% drop off object
for i = 1:300
    setAngles(q3_p(i,:), vel);
end

setGripper(0);




%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
