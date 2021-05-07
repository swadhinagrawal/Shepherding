% Animation Parameters
P.min_bound = 0;
P.max_bound = 10;
P.dt = 0.1;

% Model Initialization
P.num_herders = 4;
P.num_walkers = 6;
P.herd_radius = 10;
P.herd_center = rand(1,2)*(P.max_bound-P.min_bound-1)+P.min_bound+1;
P.sheep_body_size = 0.7;
P.offset = 2;
P.d_s_closeness = 10;
P.r0 = 10;
P.kp = 0.5;
P.kd = 2;
P.Rr = 1;
P.Ro = 3;
P.Ra = 12;
P.visibility = 2*pi;