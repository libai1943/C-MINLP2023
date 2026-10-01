function r = PlanReferenceVelocity(points,direction)
% Rest-to-rest PMP bang/coast/bang speed on each constant-gear subpath.
global params_
q=params_.vehicle; n=params_.opti.nfe+1;
ds=hypot(diff(points(:,1)),diff(points(:,2)));
cuts=[1;find(diff(direction)~=0)+1;numel(direction)+1];
durations=zeros(numel(cuts)-1,1); sections=cell(size(durations));
for k=1:numel(durations)
    ix=cuts(k):cuts(k+1); sp=[0;cumsum(ds(ix(1:end-1)))];
    g=direction(ix(1)); vmax=q.vmax; if g<0, vmax=abs(q.vmin); end
    peak=min(vmax,sqrt(q.amax*sp(end))); ta=peak/q.amax;
    tc=max(0,(sp(end)-peak^2/q.amax)/peak);
    durations(k)=2*ta+tc;
    sections{k}=struct('points',points(ix,:),'s',sp,'gear',g,'peak',peak,'ta',ta,'tc',tc);
end
r.tf=sum(durations); r.t=linspace(0,r.tf,n)'; r.x=zeros(n,1); r.y=r.x; r.theta=r.x; r.v=r.x; r.a=r.x; r.phy=r.x;
times=[0;cumsum(durations)]; r.direction=ones(n,1);
for i=1:n
    k=find(r.t(i)>=times(1:end-1)-1e-10,1,'last'); z=sections{k}; tau=r.t(i)-times(k);
    if tau<z.ta
        v=q.amax*tau; s=0.5*q.amax*tau^2; a=q.amax;
    elseif tau<z.ta+z.tc
        v=z.peak; s=0.5*z.peak*z.ta+z.peak*(tau-z.ta); a=0;
    else
        remain=max(0,durations(k)-tau); v=q.amax*remain; s=z.s(end)-0.5*q.amax*remain^2; a=-q.amax;
    end
    [ss,ii]=unique(z.s); pp=z.points(ii,:); s=min(max(s,0),ss(end));
    pos=interp1(ss,pp,s,'linear'); r.x(i)=pos(1); r.y(i)=pos(2); r.theta(i)=pos(3);
    curvature=gradient(pp(:,3))./max(gradient(ss),1e-9);
    r.phy(i)=atan(q.lw*z.gear*interp1(ss,curvature,s,'linear'));
    r.v(i)=z.gear*v; r.a(i)=z.gear*a; r.direction(i)=z.gear;
end
r.w=gradient(r.phy,r.tf/(n-1));
r.v([1,end])=0; r.a([1,end])=0; r.phy([1,end])=0; r.w([1,end])=0;
end
