%DH = [pi/2 0 0 76; 0 43.22769 0 23.65; pi/2 0 0 0; -pi/2 0 0 43.18; pi/2 0 0 0; 0 0 0 20];
DH = [0 0.76 0 pi/2; 0 -0.2365 0.4322769 0; 0 0 0 pi/2; 0 0.4318 0 -pi/2; 0 0 0 pi/2; 0 0.20 0 0];



%delcare robot
myrobot = mypuma560(DH);
close all;



%%%%%%%%%%%%%%%%%SETUP FOR TAU TEST%%%%%%%%%%%%%%%%%%%%%%%%%
H1 = eul2tr([0 pi pi/2]); % eul2tr converts ZYZ Euler angles to a hom. tsf. mtx
H1(1:3,4)=[-1; 3; 3;]/4; % This assigns the desired displacement to the hom.tsf.mtx.
q_start = inverse(H1,myrobot);
% This is the starting joint variable vector.
H2 = eul2tr([0 pi -pi/2]);
H2(1:3,4)=[3; -1; 2;]/4;
q_final = inverse(H2,myrobot);

tau = att(q_start, q_final, myrobot);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%



%%%%%%%%%%%%%%%%%%%%%MOTION PLAN NO OBSTACLES%%%%%%%%%%%%%%%%%%%%%%%%%
q_final(4) = q_final(4) + 2*pi;
qref = motionplan(q_start,q_final,0,10,myrobot,[],0.01,0);
t=linspace(0,10,300);
q = ppval(qref,t)';
plot(myrobot,q);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%




%%%%%%%%%%%%%%%%%%with obstacles TEST%%%%%%%%%%%%%%%%%%%%%%%
setupobstacle
q3 = 0.9*q_start+0.1*q_final;
taur = rep(q3, myrobot, obs{1});



q = [pi/2 pi 1.2*pi 0 0 0];
tau = rep(q,myrobot,obs{6});

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%last part%%%%%%%%%%%%%%

hold on
axis([-1 1 -1 1 0 2])
view(-32,50)
plotobstacle(obs);
qref = motionplan(q_start,q_final,0,10,myrobot,obs,0.01, 1);
t=linspace(0,10,300);
qa=ppval(qref,t').';
plot(myrobot,qa);
hold off


