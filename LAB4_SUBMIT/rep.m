function tau = rep(q ,myrobot, obs)

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


%%%%%%%%%%%%%%%%%%%%%%%%FREP computations%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
N = 1;


%IF SPHERE
if(obs.type == 'sph')
        x = obs.c(1);
        y = obs.c(2);

        z = obs.c(3);
    
        o1b = [o01(1) - x; o01(2) - y; o01(3) - z];
        rho1 = norm(o1b) - obs.R;
        if(rho1 <= obs.rho0)
            grad1 = o1b/norm(o1b);
            f1 = N*(1/(rho1) - 1/obs.rho0)*(1/(rho1^2))*grad1;
        else
            f1 = [0;0;0];
        end
        
        o2b = [o02(1) - x; o02(2) - y; o02(3) - z];
        rho2 = norm(o2b) - obs.R;
        if(rho2 <= obs.rho0)
            grad2 = o2b/norm(o2b);
            f2 = N*(1/(rho2) - 1/obs.rho0)*(1/(rho2^2))*grad2;
        else
            f2 = [0;0;0];
        end
        
        o3b = [o03(1) - x; o03(2) - y; o03(3) - z];
        rho3 = norm(o3b) - obs.R;
        if(rho3 <= obs.rho0)
            grad3 = o3b/norm(o3b);
            f3 = N*(1/(rho3) - 1/obs.rho0)*(1/(rho3^2))*grad3;
        else
            f3 = [0;0;0];
        end
        
        o4b = [o04(1) - x; o04(2) - y; o04(3) - z];
        rho4 = norm(o4b) - obs.R;
        if(rho4 <= obs.rho0)
            grad4 = o4b/norm(o4b);
            f4 = N*(1/(rho4) - 1/obs.rho0)*(1/(rho4^2))*grad4;
        else
            f4 = [0;0;0];
        end
        
        
        o5b = [o05(1) - x; o05(2) - y; o05(3) - z];
        rho5 = norm(o5b) - obs.R;
        if(rho5 <= obs.rho0)
            grad5 = o5b/norm(o5b);
            f5 = N*(1/(rho5) - 1/obs.rho0)*(1/(rho5^2))*grad5;
        else
            f5 = [0;0;0];
        end

        o6b = [o06(1) - x; o06(2) - y; o06(3) - z];
        rho6 = norm(o6b) - obs.R;
        if(rho6 <= obs.rho0)
            grad6 = o6b/norm(o6b);
            f6 = N*(1/(rho6) - 1/obs.rho0)*(1/(rho6^2))*grad6;
        else
            f6 = [0;0;0];
        end
        
elseif (obs.type == 'cyl')
    %CLYINDER with finite height
    x = obs.c(1);
    y = obs.c(2);

    f1 = getCylRep(obs, o01);
    f2 = getCylRep(obs, o02);
    f3 = getCylRep(obs, o03);
    f4 = getCylRep(obs, o04);
    f5 = getCylRep(obs, o05);
    f6 = getCylRep(obs, o06);

else
     %PLANE
    f1 = getPlaneRep(obs, o01);
    f2 = getPlaneRep(obs, o02);
    f3 = getPlaneRep(obs, o03);
    f4 = getPlaneRep(obs, o04);
    f5 = getPlaneRep(obs, o05);
    f6 = getPlaneRep(obs, o06);

end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%



%%%%%%%%%%%%%%%%%%%%%%%Torque Computations%%%%%%%%%%%%%%%%%%

T1 = J01.'*f1;
T2 = J02.'*f2;
T3 = J03.'*f3;
T4 = J04.'*f4;
T5 = J05.'*f5;
T6 = J06.'*f6;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

t=T1 + T2 + T3 + T4 + T5 + T6;
n = norm(t);
 if(n > 0.00001)
    tau = t/n;
 else
     tau = zeros(6,1);
 end

 
 
 
end

