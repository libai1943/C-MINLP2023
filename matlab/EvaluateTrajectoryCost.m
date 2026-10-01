function c = EvaluateTrajectoryCost(r)
% Equations (8)-(9); the squared acceleration sum has no dt multiplier.
global params_
sg=sign(r.v); sg(abs(r.v)<1e-6)=0;
nonzero=find(sg~=0); gs=sg(nonzero);
c.gear_shifts=sum(diff(gs)~=0);
% Segment lengths use the same left-Euler distance dt*|v_i| as model (2).
direction=r.direction; cuts=[1;find(diff(direction)~=0)+1;numel(direction)+1];
len=zeros(numel(cuts)-1,1); gears=len; dt=r.tf/(numel(r.v)-1);
for k=1:numel(len)
    ix=cuts(k):min(cuts(k+1)-1,numel(r.v)-1);
    len(k)=sum(abs(r.v(ix)))*dt; gears(k)=direction(cuts(k));
end
c.segment_lengths=len; c.segment_gears=gears;
c.components=[r.tf,sum(r.a.^2+r.v.^2.*r.w.^2),c.gear_shifts, ...
    sum(max(len(gears<0)-params_.opti.backward_length_threshold,0).^2)];
c.total=c.components*params_.opti.weights';
c.target=c.components(1:2)*params_.opti.weights(1:2)';
end
