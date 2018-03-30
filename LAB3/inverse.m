function q = inverse(H, myrobot)

    %Compute wrist center coordinates
    Od = [ H(1,4);H(2,4);H(3,4)];
    Rd = [ H(1,1) H(1,2) H(1,3); H(2,1) H(2,2) H(2,3); H(3,1) H(3,2) H(3,3)];
    d6_vec = [ 0; 0; myrobot.d(6)];
    
    Oc = Od - Rd*d6_vec;
    
    xc = Oc(1);
    yc = Oc(2);
    zc = Oc(3);
    
    %r square to be used in equations
    r_sq = xc^2 + yc^2; 

    %theta_1 to theta_3 calculations using formulas given
    theta_1 = atan2(yc,xc) - atan2(-myrobot.d(2),real(sqrt(r_sq - myrobot.d(2)^2)));

    D = -( myrobot.a(2)^2 + myrobot.d(4)^2 - (r_sq - myrobot.d(2)^2  + (zc - myrobot.d(1))^2))/(2 * myrobot.a(2) * myrobot.d(4));
    
    theta_3 = atan2(D, real(sqrt(1-D^2)));
    
    theta_2 = atan2(zc - myrobot.d(1), real(sqrt(r_sq - myrobot.d(2)^2))) - atan2(-myrobot.d(4)*cos(theta_3), myrobot.a(2) + myrobot.d(4)*sin(theta_3));

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
    
    %return joint vector
    q = [theta_1; theta_2; theta_3; theta_4; theta_5; theta_6].';
    
    
end
