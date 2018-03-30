function out = getMat(alpha, theta, d, a)

    %this functions returns the H(i-1)(i) matrix based on given DH values
    Amat = zeros(4,4);
    
    ct = cos(theta);
    st = sin(theta);
    ca = cos(alpha);
    sa = sin (alpha);
    
    Amat(1,1) = ct;
    Amat(1,2) = -st*ca;
    Amat(1,3) = st*sa;
    Amat(1,4) = a*ct;
    
    Amat(2,1) = st;
    Amat(2,2) = ct*ca;
    Amat(2,3) = -ct*sa;
    Amat(2,4) = a*st;
    
    Amat(3,1) = 0;
    Amat(3,2) = sa;
    Amat(3,3) = ca;
    Amat(3,4) = d;
    
    Amat(4,4) = 1;
    
    out = Amat;
    
end
