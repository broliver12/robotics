function hmat = forward(joint, myrobot)



for i = 1:length(joint)
    
    c_theta = cos(joint(i));
    s_theta = sin(joint(i));
    
    c_alpha = cos(myrobot.alpha(i));
    s_alpha = sin(myrobot.alpha(i));
    
    d = myrobot.d(i);
    a = myrobot.a(i);
    
    %compute H(i-1)(i) of each joint
    H_temp = [c_theta -s_theta*c_alpha s_theta*s_alpha a*c_theta;
              s_theta c_theta*c_alpha -c_theta*s_alpha a*s_theta;
              0 s_alpha c_alpha d;
              0 0 0 1;];
    if i==1
      H_total = H_temp;
        
        
    else 
        %multiply matrices into each other
       H_total = H_total*H_temp;
        
    end
    
    
    
end


hmat = H_total;



end
