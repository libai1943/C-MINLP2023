function InitializeParams()
% Table I in Li et al., IEEE T-IV 8(2), 1512-1522 (2023).
global params_
params_ = struct();
params_.vehicle.lf = 1.71;
params_.vehicle.lw = 5.73;
params_.vehicle.lr = 1.90;
params_.vehicle.lb = 3.50;
params_.vehicle.length = 9.34;
params_.vehicle.dual_disk_radius = hypot(9.34/4, 3.50/2);
params_.vehicle.r2p = 9.34/4 - 1.90;
params_.vehicle.f2p = 3*9.34/4 - 1.90;
params_.vehicle.vmax = 2;
params_.vehicle.vmin = -1;
params_.vehicle.amax = 0.2;
params_.vehicle.phymax = 0.49;
params_.vehicle.wmax = 0.14;
params_.vehicle.gamma = 0.6122;
params_.opti.nfe = 100; % INTERVALS: 101 configurations, indexed 0..100 in the paper.
params_.opti.weights = [1, 0.025, 20, 1];
params_.opti.min_cusp_distance = 5;
params_.opti.backward_length_threshold = 30;
params_.opti.num_candidates = 200;
% Numerical choices below are demo settings, not values claimed from Table I.
params_.opti.beta = 5;
params_.opti.corridor_step = 0.25;
params_.opti.corridor_max = 5;
params_.opti.validation_tolerance = 1e-5;
params_.search.step = 2.5;
params_.search.sample_step = 0.5;
params_.search.xy_resolution = 2.5;
params_.search.theta_resolution = 2*pi/48;
params_.search.num_steers = 5;
params_.search.connection_interval = 20;
params_.search.max_expansions = 120000;
params_.search.gear_penalty = 20;
params_.search.steer_penalty = 0.2;
params_.search.short_segment_penalty = 20;
end
