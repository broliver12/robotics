% Obstacle 1: Cylinder
obs{1}.R = 100;
obs{1}.c = [620; 0];
obs{1}.rho0 = 150;
obs{1}.h = 572;
obs{1}.type = 'cyl';
% Obstacle 2: Cylinder
obs{2}.R = 100;
obs{2}.c = [620; -440];
obs{2}.rho0 = 150;
obs{2}.h = 572;
obs{2}.type = 'cyl';
%Obstacle 3: Ground Plane
obs{3}.type = 'pla'
obs{3}.rho0 = 150;