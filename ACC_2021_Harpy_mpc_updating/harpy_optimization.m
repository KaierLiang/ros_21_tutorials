close all, clear all, clc

%% Simulation Setup

% addpath('generated_functions\') % filesep makes this usable on all OS
addpath(['generated_functions',filesep]) 
addpath(['utility_functions',filesep])    
addpath(['simulation_data',filesep])   

% Simulation options
p.use_lugre = 0;            % 1 = enable lugre friction model
p.use_damped_rebound = 0;   % 1 = enable damped rebound, very plastic impact
p.use_planar_dynamics = 0;  % 0 = 3d motion

p.g = 9.8;        % Gravitational constant

% Mass properties
p.mb = 2;     % Body mass
p.mh = 0.5;   % Sagittal hip motor mass
p.mk = 0.5;   % Knee motor mass

% Body inertia
p.Ib_1 = 0.001;  % xx
p.Ib_2 = 0.001;  % yy
p.Ib_3 = 0.001;  % zz

% Motor inertia (assuming cylinder)
p.Ic_xx = 0.0001;
p.Ic_yy = 0.0001;
p.Ic_zz = 0.0001;

% Leg morphology (left side)
p.L1_L = [0; 0.1; -0.1];  % body to hipF in body frame
p.L2_L = [0; 0.05; 0];    % hipF to hipS in hipF frame
p.L3_L = [0; 0; -0.3];    % hipS to knee in hipS frame
p.L6_L = [0; 0; 0.1];     % hip to hip parallel mechanism
p.l4a = 0.2;
p.l4b = 0.2;

% Thruster positions
p.Lt_L = [0.0; 0.1;0]; % Left
p.Lt_R = [0.0;-0.1;0]; % Right

% Leg morphology (right side), invert y direction
p.L1_R = p.L1_L; 
p.L2_R = p.L2_L; 
p.L3_R = p.L3_L; 
p.L6_R = p.L6_L; 
p.L1_R(2) = -p.L1_R(2);
p.L2_R(2) = -p.L2_R(2);

% Joint damping
p.damping = 5;    % Joint damping
p.kb_ground = 5;  % Ground z rotational damping 

% Compliant ground model ==================================================

p.ground_z = 0;                 % Ground vertical position
p.kp_g = 8000;                  % 2000; %  Spring coefficient
p.kd_g = sqrt(p.kp_g)*2*1.5;    % 200 Damping coefficient

% Ground friction model ===================================================

% Basic friction model parameters
p.kfs = 0.8;   % Static friction coefficient
p.kfc = p.kfs*0.9;    % Coulomb friction coefficient
p.kfb = 0.85;   % Viscous friction coefficient

% LuGre model parameters
p.s0 = 10;                 % LuGre bristle stiffness
p.s1 = sqrt(p.s0)*2*1.5;    % LuGre bristle damping
p.gv_min = 1e-3;    % static/coulomb friction threshold for no contact assumption
p.zd_max = 100;    % z decay rate during no contact
p.vs = 1e-2;        % Stribeck velocity (m/s)

% Thruster parameters =====================================================

p.v_th_min = 4;   % Minimum voltage
p.v_th_max = 16;  % Maximum voltage
p.f_th_max = 100;  % maximum thrust force at V_max (N)

% Controller parameters ===================================================

% Joint controller gains (PID)
% Order each leg: hip frontal, hip sagittal, knee sagittal
p.kp = diag([1,1,3, 1,1,3, 1, 1])*1000;   % leg L, leg R, thruster LR
p.kd = diag([1,1,3, 1,1,3, 1, 1])*20;     
p.ki = diag([1,1,3, 1,1,3, 1, 1])*40;

% Thruster controller (PD) for roll adjustment
p.kp_th = 200;
p.kd_th = 20;

% ERG thruster controller gains
p.kp_th_erg = diag([1,1,1])*200*2;
p.kd_th_erg = diag([1,1,1])*40*0.5;
p.kp_r = 200;
p.kd_r = 40;

% ERG gains
p.kr = 20;
p.kn = 20;

p.spd_scaler = 0.7;

%% Optimization setup

% Timing parameters
p.t_start = 0;
p.gait_period = 0.5 * 1.5;
p.v_des = 0.30; % Target walking rate

p.t_end = 10*p.gait_period + p.t_start;

p.dt = 0.0005;
t_sim = 0:p.dt:p.t_end;
N = length(t_sim);

% Parameters to be optimized, all positive values
k0_0 = zeros(13,1);

% Parameters for the leg (positive values for simplicity) initial and 
% final foot pos b0 [x y z], assume symmetric gait
k0_0(1:3) = [0.1; 0.15; 0.6];    

% parameters for b2 [x y z]
k0_0(4:6) = [0.0; 0.15; 0.5];    % swing parameter
k0_0(7:9) = [0.0; 0.15; 0.6];    % stance parameter

% parameters or x adjustment about body center
k0_0(10) = 0.025;   % x foot pos adjustment from body center
k0_0(11) = 0.2;     % body inital velocity x 
k0_0(12) = 0.1;     % body inital velocity z
k0_0(13) = 1.7;     % body initial pitch rate

% Body initial states (ang vel y)
% k(11) = 0;

% Cost weightings
p.Qu = 0;     % Input cost
p.Qvy = 0;    % Body lateral vel cost
p.Qvx = 10;   % Body forward vel cost
p.Qw = 1;     % Body ang vel cost
p.Qh = 10;    % Body height cost

% Parameter upper and lower bounds
k_lb = [0.01;  0.14; 0.59;... % initial
        -0.2; 0.14; 0.45;...  % swing 
        -0.2; 0.14; 0.55;...  % stance
        -0.1; 0; 0; -6];      % x adjustment and init states
      
k_ub = [0.2;  0.16; 0.61;...
        0.2; 0.16; 0.55;...
        0.2; 0.16; 0.7;...
        0.1; 0.3; 0.3; 6]; 


%% Optimization

options = optimoptions(@fmincon,'Algorithm','interior-point', ...
          'OutputFcn', @outfun, 'MaxIterations',20,'Display','iter');

        
% opt_param_v2_7 is the best, gait period = 0.5
% load(['simulation_data',filesep,'opt_param_v2_7.mat'], 'k_opt');

% Kaier's result
% load(['simulation_data',filesep,'opt_param_v4_0.mat'], 'k_opt');
load(['simulation_data',filesep,'opt_param_v4_2.mat'], 'k_opt');

k0 = k_opt;
p.run_optimizer = 0;
p.use_planar_dynamics = 1;
      
if (p.run_optimizer == 1)
  p.flag_record_data = 0;

  k_opt = fmincon(@(k) harpy_opt_cost(k,p),...
        k0, [], [], [], [], k_lb, k_ub, @(k) nonlin_const(k), options);

  save(['simulation_data',filesep,'opt_param_v5_0.mat'], 'k_opt');
  k0 = k_opt;
end

%% Begin Simulation (debug)

% Debug
% tic
p.flag_record_data = 1;
% [cost, data] = harpy_opt_cost(k_opt,p);
[cost, data] = harpy_opt_cost(k0,p);
% [cost, data] = harpy_opt_cost(k0_0,p);
% toc

%%







close all
if (p.flag_record_data == 1)
  plot_harpy_animation
  plot_harpy_data
  
  
end



%%

function stop = outfun(k,optimValues,state)
  stop = false;
  
  k
  optimValues.fval
  
  opt_data.k = k;
  opt_data.cost = optimValues.fval;
  
  save(['simulation_data',filesep,'opt_iter_data.mat'],'opt_data');
  
end

function [c,ceq] = nonlin_const(k)

% c <= 0
c(1) = norm(k(4)) - k(1);
c(2) = norm(k(7)) - k(1);

% ceq = 0
ceq(1) = k(2) - 0.15;
ceq(2) = k(5) - 0.15;
ceq(3) = k(8) - 0.15;
ceq(4) = k(3) - 0.60;




end





