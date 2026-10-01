function [valid, clearance] = CheckPoseCollision(pose)
% Test the two discs covering the truck at the supplied configurations only.
global params_
q = params_.vehicle;
cx = [pose(:,1)+q.r2p*cos(pose(:,3)); pose(:,1)+q.f2p*cos(pose(:,3))];
cy = [pose(:,2)+q.r2p*sin(pose(:,3)); pose(:,2)+q.f2p*sin(pose(:,3))];
b = params_.scenario.bounds;
clearance = min([cx-b(1); b(2)-cx; cy-b(3); b(4)-cy])-q.dual_disk_radius;
for ii = 1 : size(params_.scenario.obstacles,1)
    o = params_.scenario.obstacles(ii,:);
    dx = max(max(o(1)-cx,0),cx-o(2));
    dy = max(max(o(3)-cy,0),cy-o(4));
    clearance = min(clearance,min(hypot(dx,dy))-q.dual_disk_radius);
end
valid = clearance >= -1e-9;
end
