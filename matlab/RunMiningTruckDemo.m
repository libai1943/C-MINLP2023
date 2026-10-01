function demo = RunMiningTruckDemo(case_id)
root=fileparts(mfilename('fullpath')); previous=pwd;
cleanup=onCleanup(@()cd(previous)); cd(root);
assert(exist('dubinsConnection','class')==8,'Navigation Toolbox is required for GHA Dubins connections.');
InitializeParams(); LoadCase(case_id);
global params_
fprintf('C-MINLP: %d intervals / %d configurations, %d candidate paths.\n',params_.opti.nfe,params_.opti.nfe+1,params_.opti.num_candidates);
[reference,search_info]=SearchGlobalHybridAStar();
corridors=BuildSafeTravelCorridors(reference);
WriteTargetNLPData(reference,corridors);
started=tic; solution=SolveTargetNLP(reference); nlp_seconds=toc(started);
report=ValidateSolution(solution,reference,corridors);
DrawResults(reference,solution);
demo=struct('reference',reference,'solution',solution,'search',search_info, ...
    'report',report,'nlp_seconds',nlp_seconds,'parameters',params_);
save(fullfile('AmplResults','demo_result.mat'),'demo');
fid=fopen(fullfile('AmplResults','validation.json'),'w');
fprintf(fid,'%s',jsonencode(struct('report',report,'search',search_info,'nlp_seconds',nlp_seconds))); fclose(fid);
end
