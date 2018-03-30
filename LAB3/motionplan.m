function qref = motionplan(q0, qf, t1, t2, myrobot, obs, accur, do)

    
    done=0;
    alpha=0.01;
    i=1;
    curq = q0;
    temp_qs=zeros(10,6);
    
    
   %%%%%%%%%%%GRADIENT DESCENT ALGORITHM
    while(done==0)

       %SET THE TOLERANCE TO 0.005
       if(norm(curq(1:5)-qf(1:5))>=0.005)
           lastq = curq;
           curtau = att(curq, qf, myrobot).';
           
           repsum = zeros(1,6).';
           if(do == 1)
               %SUM THE REPULSIVE FORCES
               repsum = repsum + rep(curq, myrobot, obs{1});
               repsum = repsum + rep(curq, myrobot, obs{2});
               repsum = repsum + rep(curq, myrobot, obs{3});
               repsum = repsum + rep(curq, myrobot, obs{4});
               repsum = repsum + rep(curq, myrobot, obs{5});
               repsum = repsum + rep(curq, myrobot, obs{6});
           end
           curq = lastq + alpha*(curtau + repsum.');
       
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
