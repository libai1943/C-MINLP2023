function LoadCase(case_id)
% Constructed dumping-bay example; not one of the paper's 20 benchmark maps.
global params_
assert(case_id == 1, 'This release contains the constructed case_id = 1.');
params_.scenario.bounds = [0, 100, 0, 90];
params_.scenario.start = [20, 25, 0];
params_.scenario.goal = [70, 50, pi];
% Axis-aligned berm/rock rectangles [xmin xmax ymin ymax], in metres.
params_.scenario.obstacles = [60,84,38,44; 60,84,56,62; ...
    82,88,44,56; 30,43,5,15; 14,26,57,70];
end
