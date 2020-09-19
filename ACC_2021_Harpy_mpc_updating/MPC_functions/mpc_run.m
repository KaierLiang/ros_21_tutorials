clear all, close all,
clc
addpath(['generated_functions',filesep]) 

nx = 8;
ny = 3;
nu = 3;
nlobj = nlmpc(nx,ny,nu);
nlobj.Model.StateFcn = "model_2d";
nlobj.Model.IsContinuousTime = true;
nlobj.Model.OutputFcn = "func_output";
Ts = 0.01;
p = 20;
nlobj.Ts = Ts;
nlobj.PredictionHorizon = p;
nlobj.ControlHorizon = p;
nlobj.Optimization.CustomCostFcn = "cost_function";
nlobj.Optimization.ReplaceStandardCost = true;
%nlobj.Optimization.CustomEqConFcn = "myEqConFunction";
nlobj.Optimization.UseSuboptimalSolution = true;
%nlobj.Optimization.CustomIneqConFcn = "myinEqConFunction";

nlobj.MV(1).Min = 0;
nlobj.MV(1).Max = 60;
nlobj.MV(2).Min = 0;
nlobj.MV(2).Max = 5*9.8 + 60;
% nlobj.MV(3).Min = -5;
% nlobj.MV(3).Max = 5;
%nlobj.Optimization.UseSuboptimalSolution = false;

% ref_x = linspace(0,1,p);
% ref_y = -4*(ref_x - 0.5).^2 + 0.5^2*4;
% ref = [ref_x',ref_y'];
ref = func_ref(1);
x0 = [0;0;0;0;0;0;0;0];  
u0 = [0.2388;   31.9234;   -0.1355];
x0 = [         0
         0
            0
   -0.4025
    0.0032
   -0.0009
    0.0042
    0.0056];
% x0 = [    0.6657
%     0.2402
%    -0.2748
%    -0.4058
%     1.3115
%    -0.4476
%    -7.4526
%   -24.1242];


% u0 = [  -93.4413
%    13.5910
%  -201.8898];
%validateFcns(nlobj,x0,u0);
tic
[mv,~,info] = nlmpcmove(nlobj,x0,u0,ref);
toc
