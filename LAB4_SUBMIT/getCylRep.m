function out = getCylRep(obs, o01)
%repulsive force from cylinder with finite height
    x = obs.c(1);
    y = obs.c(2);
    N=1;
    a1 = [o01(1) - x; o01(2) - y; 0];
	b1 = o01(3) - obs.h;

    if (b1>0)
        if (norm(a1) > obs.R)
            %above and outside cylinder
            t = [o01(1) - x; o01(2) - y; 0];
            theta = atan2(t(2),t(1));
            o1b = [t(1) - obs.R*cos(theta); t(2) - obs.R*sin(theta); b1];
            rho1 = norm(o1b);
            if(rho1 <= obs.rho0)
                grad1 = o1b/norm(o1b);
                f1 = N*(1/(rho1) - 1/obs.rho0)*(1/(rho1^2))*grad1;
            else
                f1 = [0;0;0];
            end
        else
            %above cylinder
            o1b = [0; 0; b1];
            rho1 = norm(o1b);
            if(rho1 <= obs.rho0)
                grad1 = o1b/norm(o1b);
                f1 = N*(1/(rho1) - 1/obs.rho0)*(1/(rho1^2))*grad1;
            else
                f1 = [0;0;0];
            end
        end
    else 
        %z<h use lab 3 method
        o1b = [o01(1) - x; o01(2) - y; 0];
        
        rho1 = norm(o1b) - obs.R;
        if(rho1 <= obs.rho0)
            grad1 = o1b/norm(o1b);
            f1 = N*(1/(rho1) - 1/obs.rho0)*(1/(rho1^2))*grad1;
            
        else
            f1 = [0;0;0];
        end
    end

    
    out = f1;


end
