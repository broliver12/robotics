function qref = motionplan(q0, qf, t1, t2, myrobot, obs)

    
    done=0;
    alpha_att=0.013;
    alpha_rep=0.01;
    i=1;
    curq = q0;
    temp_qs=zeros(10,6);
    
    
   %%%%%%%%%%%GRADIENT DESCENT ALGORITHM
   
    while(done==0)

       %SET THE TOLERANCE TO 0.005
       norm(curq(1:5)-qf(1:5))
       if(norm(curq(1:5)-qf(1:5))>=0.01)
           lastq = curq;
           curtau = att(curq, qf, myrobot).';
           
            repsum = zeros(1,6).';
           
               %SUM THE REPULSIVE FORCES
               f = rep(curq, myrobot, obs{1});
               if (f ~= [0 0 0 0 0 0 0])
                   f;
               end
               %THIS IS SET UP FOR SECTION 4.3 AND 4.4
               repsum = repsum + rep(curq, myrobot, obs{1});
               repsum = repsum + rep(curq, myrobot, obs{2});
               repsum = repsum + rep(curq, myrobot, obs{3});
    
           curq = lastq + alpha_att*curtau + alpha_rep*repsum.';
       
           temp_qs(i,1:6) = curq;
           i = i+1;
       else
           done=1;
       end
    
        
    end

    %%%%%%%%%%%RETURN SPLINE AS PER INSTRUCTIONS
    t =linspace(t1,t2,size(temp_qs,1));
    qref = spline(t,temp_qs');


end
