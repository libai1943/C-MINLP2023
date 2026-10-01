function corridors = BuildSafeTravelCorridors(r)
% Fixed safe boxes for centres of the two covering discs, built only once.
global params_
q=params_.vehicle; n=numel(r.x); corridors=zeros(n,4,2);
for disk=1:2
    offsets=[q.r2p,q.f2p]; d=offsets(disk);
    for i=1:n
        cx=r.x(i)+d*cos(r.theta(i)); cy=r.y(i)+d*sin(r.theta(i));
        box=[cx,cx,cy,cy]; growth=zeros(1,4); done=false(1,4);
        assert(IsSafeBox(box),'Reference disc lies in collision.');
        while ~all(done)
            for side=1:4
                if done(side), continue; end
                trial=box; delta=params_.opti.corridor_step;
                trial(side)=trial(side)+delta*(-1)^side;
                if growth(side)+delta>params_.opti.corridor_max || ~IsSafeBox(trial)
                    done(side)=true;
                else
                    box=trial; growth(side)=growth(side)+delta;
                end
            end
        end
        corridors(i,:,disk)=box;
    end
end
end

function valid=IsSafeBox(b)
global params_
r=params_.vehicle.dual_disk_radius; limits=params_.scenario.bounds;
valid=b(1)>=limits(1)+r && b(2)<=limits(2)-r && b(3)>=limits(3)+r && b(4)<=limits(4)-r;
if ~valid, return; end
for j=1:size(params_.scenario.obstacles,1)
    o=params_.scenario.obstacles(j,:);
    dx=max([o(1)-b(2),b(1)-o(2),0]); dy=max([o(3)-b(4),b(3)-o(4),0]);
    if hypot(dx,dy)<r+1e-8, valid=false; return; end
end
end
