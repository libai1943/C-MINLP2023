function DrawResults(reference,r)
% Exactly two static figures. Footprints are drawn before all trajectory lines.
global params_
q=params_.vehicle;
fig1=figure('Name','C-MINLP: trajectory and footprints','Color','w','Position',[80 80 1120 720]);
ax=axes(fig1); hold(ax,'on'); axis(ax,'equal'); box(ax,'on'); grid(ax,'on');
for k=1:size(params_.scenario.obstacles,1)
    o=params_.scenario.obstacles(k,:);
    patch(ax,[o(1),o(2),o(2),o(1)],[o(3),o(3),o(4),o(4)],[0.34,0.37,0.39], ...
        'EdgeColor',[0.2,0.23,0.25],'HandleVisibility','off');
end
% Lay the optimal footprints underneath reference and optimized centre lines.
for i=unique([1:4:numel(r.x),numel(r.x)])
    vertices=CreateVehiclePolygon(r.x(i),r.y(i),r.theta(i),2);
    patch(ax,vertices.x,vertices.y,[0.52,0.76,0.92],'FaceAlpha',0.10, ...
        'EdgeColor',[0.48,0.66,0.76],'LineWidth',0.7,'HandleVisibility','off');
end
h1=plot(ax,reference.x,reference.y,'--','Color',[0.87,0.47,0.12],'LineWidth',2);
forward=r.v>=-1e-6; backward=r.v<-1e-6;
xx=r.x; yy=r.y; xx(~forward)=nan; yy(~forward)=nan;
h2=plot(ax,xx,yy,'Color',[0,0.38,0.65],'LineWidth',2.5);
xx=r.x; yy=r.y; xx(~backward)=nan; yy(~backward)=nan;
h3=plot(ax,xx,yy,'Color',[0.78,0.13,0.20],'LineWidth',2.5);
h4=plot(ax,r.x(1),r.y(1),'o','MarkerFaceColor',[0.1,0.6,0.3],'MarkerEdgeColor','w','MarkerSize',9);
h5=plot(ax,r.x(end),r.y(end),'p','MarkerFaceColor',[0.78,0.13,0.20],'MarkerEdgeColor','w','MarkerSize',12);
legend(ax,[h1,h2,h3,h4,h5],{'GHA + PMP reference','Optimized: forward','Optimized: reverse','Start','Dumping pose'},'Location','northwest');
axis(ax,[8,92,0,88]); xlabel(ax,'x (m)'); ylabel(ax,'y (m)');
title(ax,sprintf('Chapter 6 | C-MINLP mining truck parking | T = %.2f s',r.tf));
subtitle(ax,'Constructed dumping-bay example; 101 configurations');
fig2=figure('Name','C-MINLP: states and controls','Color','w','Position',[120 100 1120 820]);
tiledlayout(fig2,4,2,'TileSpacing','compact','Padding','compact');
names={'x','y','theta','v','a','phy','w'};
labels={'x (m)','y (m)','\theta (rad)','v (m/s)','a (m/s^2)','\phi (rad)','\omega (rad/s)'};
for j=1:7
    ax=nexttile; hold(ax,'on'); grid(ax,'on'); box(ax,'on');
    plot(ax,reference.t,reference.(names{j}),'--','Color',[0.87,0.47,0.12],'LineWidth',1);
    plot(ax,r.t,r.(names{j}),'Color',[0,0.38,0.65],'LineWidth',1.6);
    xlabel(ax,'t (s)'); ylabel(ax,labels{j}); xlim(ax,[0,max(r.tf,reference.tf)]);
    if j==4, yline(ax,q.vmax,':'); yline(ax,q.vmin,':'); end
    if j==5, yline(ax,q.amax,':'); yline(ax,-q.amax,':'); end
    if j==6
        lim=q.phymax*ones(size(r.v)); lim(r.direction<0)=q.gamma*q.phymax;
        plot(ax,r.t,lim,':','Color',[0.4,0.4,0.4]); plot(ax,r.t,-lim,':','Color',[0.4,0.4,0.4]);
    end
    if j==7, yline(ax,q.wmax,':'); yline(ax,-q.wmax,':'); end
end
ax=nexttile; axis(ax,'off'); c=EvaluateTrajectoryCost(r);
text(ax,0,0.95,sprintf('Targeted NLP result\nT = %.3f s\nGear shifts = %d\nJ_{target} = %.3f\nJ_{full} = %.3f\n\nOrange dashed: GHA + PMP\nBlue solid: optimized',r.tf,c.gear_shifts,c.target,c.total), ...
    'VerticalAlignment','top','FontSize',12);
sgtitle(fig2,'Chapter 6 | Optimized states and controls');
folder=fullfile(rootPath(),'..','docs','images'); if ~isfolder(folder), mkdir(folder); end
exportgraphics(fig1,fullfile(folder,'trajectory.png'),'Resolution',160);
exportgraphics(fig2,fullfile(folder,'profiles.png'),'Resolution',160);
end

function folder=rootPath()
folder=fileparts(mfilename('fullpath'));
end
