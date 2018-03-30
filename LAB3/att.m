function tau = att(q,qf,myrobot)

%forward kin for final
[H01f, H02f, H03f, H04f, H05f, H06f] = forward_retAll(qf, myrobot);

Z00f = [0 0 1];

Z01f = H01f(1:3, 3);
Z02f = H02f(1:3, 3);
Z03f = H03f(1:3, 3);
Z04f = H04f(1:3, 3);
Z05f = H05f(1:3, 3);
Z06f = H06f(1:3, 3);

o01f = H01f(1:3, 4);
o02f = H02f(1:3, 4);
o03f = H03f(1:3, 4);
o04f = H04f(1:3, 4);
o05f = H05f(1:3, 4);
o06f = H06f(1:3, 4);

%%%%%Forward Kin for Jacobian Parameters%%%%%%%%%%%%%%%
[H01, H02, H03, H04, H05, H06] = forward_retAll(q, myrobot);

Z00 = [0 0 1];

Z01 = H01(1:3, 3);
Z02 = H02(1:3, 3);
Z03 = H03(1:3, 3);
Z04 = H04(1:3, 3);
Z05 = H05(1:3, 3);
Z06 = H06(1:3, 3);

o01 = H01(1:3, 4);
o02 = H02(1:3, 4);
o03 = H03(1:3, 4);
o04 = H04(1:3, 4);
o05 = H05(1:3, 4);
o06 = H06(1:3, 4);

zeroVec = [0 0 0].';
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%%%%%%%%%%%%%%%%%%%Jacobian Computations%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%cross(Z00, o01)
J01 = [cross(Z00, o01).' zeroVec zeroVec zeroVec zeroVec zeroVec];

J02 = [cross(Z00, o02).' cross(Z01, (o02 -o01)) zeroVec zeroVec zeroVec  zeroVec];

J03 = [cross(Z00, o03).' cross(Z01,(o03 - o01)) cross(Z02, (o03 - o02)) zeroVec zeroVec zeroVec];

J04 = [cross(Z00, o04).' cross(Z01, (o04 - o01)) cross(Z02, (o04 - o02)) cross(Z03, (o04 - o03)) zeroVec zeroVec];

J05 = [cross(Z00, o05).' cross(Z01, (o05 - o01)) cross(Z02, (o05 - o02)) cross(Z03, (o05 - o03)) cross(Z04, (o05 - o04)) zeroVec];

J06 = [cross(Z00, o06).' cross(Z01, (o06 - o01)) cross(Z02, (o06 - o02)) cross(Z03, (o06 - o03)) cross(Z04, (o06 - o04)) cross(Z05, (o06 - o05))];
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%%%%%%%%%%%%%%%%%%%%%%%%FATT computations%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
L = 1;
FATT1 = -L*(o01 - o01f)/100;
FATT2 = -L*(o02 - o02f)/100;
FATT3 = -L*(o03 - o03f)/100;
FATT4 = -L*(o04 - o04f)/100;
FATT5 = -L*(o05 - o05f)/100;
FATT6 = -L*(o06 - o06f)/100;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%Torque Computations%%%%%%%%%%%%%5%%%%%
T1 = J01.'*FATT1;
T2 = J02.'*FATT2;
T3 = J03.'*FATT3;
T4 = J04.'*FATT4;
T5 = J05.'*FATT5;
T6 = J06.'*FATT6;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%




t=T1 + T2 + T3 + T4 + T5 + T6;
n = norm(t);
% if(n > 0.0001)
    tau = t/n;
% else
%     tau = zeros(6,1);
% end
end

