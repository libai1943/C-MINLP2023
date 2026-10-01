function WriteTargetNLPData(r,corridors)
global params_
p=params_; q=p.vehicle; n=p.opti.nfe+1;
if ~isfolder('AmplInputs'), mkdir('AmplInputs'); end
if ~isfolder('AmplResults'), mkdir('AmplResults'); end
fid=fopen(fullfile('AmplInputs','TargetData.dat'),'w'); assert(fid>=0);
cleaner=onCleanup(@()fclose(fid));
fprintf(fid,'data;\nparam N := %d;\n',n);
names={'lw','r2p','f2p','vmax','vmin','amax','wmax','beta','w1','w2'};
values=[q.lw,q.r2p,q.f2p,q.vmax,q.vmin,q.amax,q.wmax,p.opti.beta,p.opti.weights(1:2)];
for j=1:numel(names), fprintf(fid,'param %s := %.17g;\n',names{j},values(j)); end
fprintf(fid,'param: rx ry rt direction phimax :=\n');
for i=1:n
    limit=q.phymax; if r.direction(i)<0, limit=limit*q.gamma; end
    fprintf(fid,'%d %.17g %.17g %.17g %d %.17g\n',i,r.x(i),r.y(i),r.theta(i),r.direction(i),limit);
end
fprintf(fid,';\nparam rear: 1 2 3 4 :=\n');
for i=1:n, fprintf(fid,'%d %.17g %.17g %.17g %.17g\n',i,corridors(i,:,1)); end
fprintf(fid,';\nparam front: 1 2 3 4 :=\n');
for i=1:n, fprintf(fid,'%d %.17g %.17g %.17g %.17g\n',i,corridors(i,:,2)); end
fprintf(fid,';\n');
clear cleaner
fid=fopen(fullfile('AmplInputs','InitialGuess.run'),'w'); assert(fid>=0);
cleaner=onCleanup(@()fclose(fid));
fprintf(fid,'let tf := %.17g;\n',r.tf);
names={'x','y','theta','v','a','phy','w'};
for j=1:numel(names)
    values=r.(names{j});
    for i=1:n, fprintf(fid,'let %s[%d] := %.17g;\n',names{j},i,values(i)); end
end
for i=1:n
    fprintf(fid,'let xr[%d] := %.17g;\nlet yr[%d] := %.17g;\n',i,r.x(i)+q.r2p*cos(r.theta(i)),i,r.y(i)+q.r2p*sin(r.theta(i)));
    fprintf(fid,'let xf[%d] := %.17g;\nlet yf[%d] := %.17g;\n',i,r.x(i)+q.f2p*cos(r.theta(i)),i,r.y(i)+q.f2p*sin(r.theta(i)));
end
end
