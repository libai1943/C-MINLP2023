function r = SolveTargetNLP(reference)
% AMPL executable -> .run command file -> local TXT files; no MATLAB API.
names={'x','y','theta','v','a','phy','w','xr','yr','xf','yf','tf','objective','solve_result_num'};
for i=1:numel(names)
    file=fullfile('AmplResults',[names{i},'.txt']); if isfile(file), delete(file); end
end
ampl=getenv('AMPL_EXECUTABLE');
if isempty(ampl)
    if ispc && isfile('AMPL.exe'), ampl=fullfile(pwd,'AMPL.exe'); else, ampl='ampl'; end
end
assert(~contains(ampl,'"'),'AMPL_EXECUTABLE must contain an executable path without quotation marks.');
[status,output]=system(sprintf('"%s" SolveTargetNLP.run 2>&1',ampl));
fid=fopen(fullfile('AmplResults','solver.log'),'w'); fprintf(fid,'%s',output); fclose(fid);
assert(status==0,'CMINLP:SolverProcess','AMPL failed. See AmplResults/solver.log.\n%s',output);
file=fullfile('AmplResults','solve_result_num.txt');
assert(isfile(file),'CMINLP:SolverOutput','No solver status was produced. See AmplResults/solver.log.');
r.solve_result_num=readmatrix(file);
assert(r.solve_result_num>=0 && r.solve_result_num<100,'CMINLP:SolverFailed','IPOPT did not report success. See AmplResults/solver.log.');
for i=1:numel(names)-1, r.(names{i})=readmatrix(fullfile('AmplResults',[names{i},'.txt'])); end
r.t=linspace(0,r.tf,numel(r.x))'; r.direction=reference.direction;
end
