function out = getPlaneRep(obs, o01)
%repulsive force for a plane
    N=1;
	o1b = [0;0; o01(3)];
    rho1 = norm(o1b);
    if(rho1 <= obs.rho0)
        grad1 = o1b/norm(o1b);
        f1 = N*(1/(rho1) - 1/obs.rho0)*(1/(rho1^2))*grad1;
    else
        f1 = [0;0;0];
    end

    out = f1;

end
