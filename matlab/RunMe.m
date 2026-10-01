% C-MINLP2023: Chapter 6 companion example for the CHINESE-LANGUAGE book
% 《非结构化场景自动驾驶轨迹规划技术》
% English title translation: Trajectory Planning Techniques for Autonomous
% Driving in Unstructured Environments (this is not an English edition).
%
% Please read Chapter 6 and cite the algorithm paper when using this code:
% B. Li, Y. Ouyang, X. Li, D. Cao, T. Zhang, and Y. Wang,
% "Mixed-Integer and Conditional Trajectory Planning for an Autonomous Mining
% Truck in Loading/Dumping Scenarios: A Global Optimization Approach,"
% IEEE Transactions on Intelligent Vehicles, 8(2):1512-1522, 2023.
% DOI: 10.1109/TIV.2022.3214777 (first published online in 2022).
%
% This independently implemented MATLAB demonstration follows the paper's
% GHA -> PMP -> one targeted NLP pipeline. It uses a constructed dumping bay,
% not the authors' original benchmark map or student implementation.
% Requirements: MATLAB R2021b+, Navigation Toolbox, AMPL and IPOPT executables.
% Read ../README.md for installation, equations, parameters and P-code policy.
% The AMPL .run file writes TXT output for MATLAB; no AMPL-MATLAB API is used.
% One run computes a new solution and produces exactly two static figures.

clear; clc; close all;
case_id = 1;
demo = RunMiningTruckDemo(case_id);
