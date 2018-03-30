%computes inverse kin for kuka robot
function q = inverse_kuka(H, myrobot)

    %Compute wrist center coordinates
    Od = [ H(1,4);H(2,4);H(3,4)];
    Rd = [ H(1,1) H(1,2) H(1,3); H(2,1) H(2,2) H(2,3); H(3,1) H(3,2) H(3,3)];
    d6_vec = [ myrobot.a(6); 0; myrobot.d(6)];
    
    Oc = Od - Rd*d6_vec;
    
    xc = Oc(1);
    yc = Oc(2);
    zc = Oc(3);
    
    %r square to be used in equations
    r_sq = xc^2 + yc^2; 
    r = real(sqrt(r_sq));

    %%%%%%%%%%%%%%%%%%%%%%%%%%%INVERSER FOR KUKA%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%55
    %theta_1 to theta_3 calculations using formulas given
    theta_1 = atan2(yc,xc); %- atan2(-myrobot.d(2),real(sqrt(r_sq - myrobot.d(2)^2)));

    %D = -( myrobot.a(2)^2 + myrobot.d(4)^2 - (r_sq - myrobot.d(2)^2  + (zc - myrobot.d(1))^2))/(2 * myrobot.a(2) * myrobot.d(4));
    B = real(sqrt((myrobot.d(4)^2 + myrobot.a(3)^2)));
    D = ((r - myrobot.a(1))^2 + (zc - myrobot.d(1))^2 - (myrobot.a(2)^2 + B^2))/(2*myrobot.a(2)*B);
    phi=atan2(real(sqrt(1-D^2)),D);    
    theta_3 = atan2(D, real(sqrt(1-D^2))) - atan2(myrobot.a(3), myrobot.d(4));
    %theta_3 = -0.7854;
    theta_2 = atan2(zc - myrobot.d(1), r - myrobot.a(1)) - atan2(-B*cos(theta_3), myrobot.a(2) -B*sin(theta_3)) ;
    %theta_2 = 1.0472;
    theta_2=atan2(zc - myrobot.d(1), r - myrobot.a(1)) + atan2(B*sin(phi),B*cos(phi)+myrobot.a(2));
    theta_3= pi/2 - phi-atan2(myrobot.a(3),myrobot.d(4));
    %compute H03, extrac R03, compute R36
    H01 = getMat(myrobot.alpha(1), theta_1, myrobot.d(1), myrobot.a(1));
    H12 = getMat(myrobot.alpha(2), theta_2, myrobot.d(2), myrobot.a(2));
    H23 = getMat(myrobot.alpha(3), theta_3, myrobot.d(3), myrobot.a(3));

    H03 = H01*H12*H23;
    
    R03 = H03(1:3,1:3);
    
    R36 = R03.'*Rd;
    
    %theta_4 to theta_6 calculations using formulas given (& computed R36)
    theta_5 = atan2(real(sqrt(1-R36(3,3)^2)), R36(3,3));
    theta_4 = atan2(R36(2,3), R36(1,3));
    theta_6 = atan2(R36(3,2), -R36(3,1));
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    %return joint vector
    q = [theta_1; theta_2; theta_3; theta_4; theta_5; theta_6].';
    
    
end

% % Inverse manipulator kinematics for the KUKA robot
% %
% % Q = inverse_kuka(myrobot,H)
% %
% % X =  3x1 column vector of end effector coordinates
% %
% % Returns 6x1 vector Q of joint variables
% 
% function Q = inverse_kuka(H, myrobot);
% 
%     %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%     % Preliminary Work
%     %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 
% ox = H(1,4);
% oy = H(2,4);
% oz = H(3,4);
% R = H(1:3,1:3);
%  
% DH=myrobot.dh;
% alpha=DH(:,1);
% A=DH(:,2);
% D=DH(:,4);
% 
% X=[ox;oy;oz]-R*[A(6);0;D(6)];    
% x=X(1);
% y=X(2);
% z=X(3);
% 
%     %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%     % Inverse Position Kinematics
%     %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 
% s=z-D(1);
% r=(sqrt(x^2+y^2)-A(1));
% 
% h=sqrt(D(4)^2+A(3)^2);
% dummy=(r^2+s^2-A(2)^2-h^2)/(2*A(2)*h);
% phi=atan2(sqrt(1-dummy^2),dummy); % ELBOW UP SOLUTION
% 
% q(1)=atan2(y,x);
% q(2)=atan2(s,r)+atan2(h*sin(phi),h*cos(phi)+A(2));
% q(3)=pi/2-phi-atan2(A(3),D(4));
% 
% 
%     %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%     % Inverse Orientation Kinematics
%     %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%     
% H = eye(4,4);
% for j=1:3
%      H = H*linktrans(q(j),myrobot.link{j});
% end
% R03 = H(1:3,1:3);    
% R36 = R03'*R;
% 
% q(5) = atan2(sqrt(1-R36(3,3)^2),R36(3,3));
% q(4) = atan2(R36(2,3),R36(1,3));
% q(6) = atan2(R36(3,2),-R36(3,1));
% 
% Q=[q(1);q(2);q(3);q(4);q(5);q(6)];
