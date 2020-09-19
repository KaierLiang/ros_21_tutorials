close all, clear all, clc

%% Simulation Setup

% addpath('generated_functions\') % filesep makes this usable on all OS
addpath(['generated_functions',filesep]) 
addpath(['utility_functions',filesep])    
addpath(['simulation_data',filesep])   
addpath(['MPC_functions',filesep])

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
p.kfs = 0.6;   % Static friction coefficient
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

% Thruster controller (PD) for roll and yaw adjustment
p.kp_th = 200;
p.kd_th = 20;

p.kp_th_pitch = 0;
p.kd_th_pitch = 0;

p.kp_th_pos = diag([2,1,5])*400;
p.kd_th_pos = diag([2,1,2])*40;



% ERG thruster controller gains
p.kp_th_erg = diag([1,1,1])*200*2; % 2
p.kd_th_erg = diag([1,1,1])*40*1; % 0.5
p.kp_r = 200;
p.kd_r = 40;

% ERG gains %20
p.kr = 5; % rate of convergence
p.kt = 5; % rate of convergence null space
p.kn1 = 5; % rate of pushing out (into xr)
p.kn2 = 10; % rate of pushing out (away from xr)

%% Optimization setup

% Timing parameters
p.t_start = 0;
p.gait_period = 0.75;
p.v_des = 0.30; % Target walking rate

p.t_end = 20*p.gait_period + p.t_start;

p.dt = 0.0005;
t_sim = 0:p.dt:p.t_end;
N = length(t_sim);



%% Optimization Parameter

% Optimization walking gait (no thrusters)
load(['simulation_data',filesep,'opt_param_v4_2.mat'], 'k_opt');
k = k_opt;

% Optimization parameters, assume swing = right leg

state = 0;    % 0 = left swing, 1 = right swing

% Bezier polynomial parameters for left leg
% k(1:3) = initial pos (front foot), assume symmetric gait


% Swing and stance leg bezier parameters, assuming left leg
B_swing  = zeros(3,5);
B_stance = zeros(3,5);

% Initial and final states bezier parameter
B_swing(:,1)  = [-k(1) + k(10); k(2); -k(3)];
B_swing(:,5)  = [ k(1) + k(10); k(2); -k(3)];

B_stance(:,1) = [ k(1) + k(10); k(2); -k(3)];
B_stance(:,5) = [-k(1) + k(10); k(2); -k(3)];

% Zero velocity constraint
B_swing(:,2) = B_swing(:,1);
B_swing(:,4) = B_swing(:,5);
B_stance(:,2) = B_stance(:,1);
B_stance(:,4) = B_stance(:,5);

% Bezier midpoint parameters
B_swing(:,3)  = [k(4) + k(10); k(5); -k(6)];
B_stance(:,3) = [k(7) + k(10); k(8); -k(9)];

% Bezier for stopping and start moving (assuming left leg)
B_stance_stop = B_stance;
B_swing_stop = B_swing;
B_stance_start = B_stance;
B_swing_start = B_swing;

B_stance_stop(:,5) = [0; k(5) ; -k(3)];
B_stance_stop(:,4) = B_stance_stop(:,5);
B_stance_stop(:,3) = [B_stance_stop(1,1)/2; k(8); -k(9)];
B_swing_stop(:,5) = [0; k(5) ; -k(3)];
B_swing_stop(:,4) = B_swing_stop(:,5);
B_swing_stop(:,3) = [B_swing_stop(1,1)/2; k(5); -k(6)];

B_stance_start(:,1) = [0; k(5) ; -k(3)];
B_stance_start(:,2) = B_stance_start(:,1);
B_stance_start(:,3) = [B_stance_start(1,5)/2; k(8); -k(9)];
B_swing_start(:,1) = [0; k(5) ; -k(3)];
B_swing_start(:,2) = B_swing_start(:,1);
B_swing_start(:,3) = [B_swing_start(1,5)/2; k(5); -k(6)];

% Update Bezier poly for left and right legs
if (state == 0)
  % Left swing, right stance
  BL = B_swing;
  BR = B_stance;
else
  % Right swing, left stance
  BR = B_swing;
  BL = B_stance;
end
BR(2,:) = -BR(2,:);   % Invert BR y axis

%% Simulation setup

% simulation states
Nx = 38;            % state size
x = zeros(Nx,1);    
t_sim = 0:p.dt:p.t_end;
N = length(t_sim);

% Initial states
R0 = rot_y(0);      % Initial rotation matrix
x(18:26) = R0(:);

% Body states
x(7) = k(3) - (p.mb+2*(p.mh+p.mk))*p.g/p.kp_g; % Initial vertical position
x(12) = k(11);
x(14) = k(12);
x(16) = k(13);

% Initial joint states
pos_foot_L_ref_0 = bezier4(BL,0);
pos_foot_R_ref_0 = bezier4(BR,0);
q_des_L = inverse_kinematics(pos_foot_L_ref_0,p,'l');
q_des_R = inverse_kinematics(pos_foot_R_ref_0,p,'r');

% Joint states
x(1)  = q_des_L(1);
x(2)  = q_des_L(2);
x(27) = q_des_L(3);
x(3)  = q_des_R(1);
x(4)  = q_des_R(2);
x(28) = q_des_R(3);

% Joint state error
xe = zeros(8,1);
xe_i = zeros(8,1);

% Roll states
roll_D = 0; % time derivative
pitch_D = 0; % time derivative
yaw_D = 0; % time derivative
roll_0 = 0; % previous state value
pitch_0 = 0; % previous state value
yaw_0 = 0; % previous state value

% Other
s = 0;
s0 = 0;
u = zeros(14,1);
target_spd = 0.3;
sigma = [1;0];


% Params for the model
params = [p.mb; p.mh; p.mk; p.g; p.Ib_1; p.Ib_2; p.Ib_3; ...
            p.Ic_xx; p.Ic_yy; p.Ic_zz; p.L1_L; p.L2_L; p.L3_L; ...
            p.L1_R; p.L2_R; p.L3_R;p.l4a;p.l4b;p.kb_ground;...
            p.Lt_L; p.Lt_R];

% Data recording struct
data.t = t_sim;             % Simulation time
data.x = zeros(Nx,N);       % States
data.xfL = zeros(3,N);      % Left foot position
data.xfR = zeros(3,N);      % Right foot position
data.vfL = zeros(3,N);
data.vfR = zeros(3,N);
data.qa_ref = zeros(8,N);   % Joint reference
data.xfL_ref = zeros(3,N);  % Joint reference
data.xfR_ref = zeros(3,N);  % Joint reference
data.ug = zeros(6,N);       % Ground reaction forces [L;R]
data.euler = zeros(3,N);    % Body Euler angles
data.f_th = zeros(6,N);
data.i_end = N;
data.cop = zeros(3,N);      % center of pressure
data.xw = zeros(6,N);
data.xr = zeros(6,N);
data.v = zeros(4,N);
data.hw = zeros(3,N);
data.hr = zeros(3,N);
data.wd_r = zeros(6,N);
data.wd_n = zeros(6,N);
data.wd_t = zeros(6,N);
%% MPC set up
mpc_t = 0;
nx = 8;
ny = 3;
nu = 3;
nlobj = nlmpc(nx,ny,nu);
nlobj.Model.StateFcn = "model_2d";
nlobj.Model.IsContinuousTime = true;
nlobj.Model.OutputFcn = "func_output";
Ts = 0.01;
p_h = 10;
nlobj.Ts = Ts;
nlobj.PredictionHorizon = p_h;
nlobj.ControlHorizon = p_h;
nlobj.Optimization.CustomCostFcn = "cost_function";
nlobj.Optimization.ReplaceStandardCost = true;
%nlobj.Optimization.CustomEqConFcn = "myEqConFunction";
nlobj.Optimization.UseSuboptimalSolution = true;
%nlobj.Optimization.CustomIneqConFcn = "myinEqConFunction";

nlobj.MV(1).Min = -7;
nlobj.MV(1).Max = 8;
nlobj.MV(2).Min = 0;
nlobj.MV(2).Max = 5*9.8 + 5;
nlobj.MV(3).Min = -10;
nlobj.MV(3).Max = 10;
%% Begin Simulation

tic
i_end = N;
record = 0;
k_step = 1; % Step counter
k_step_old = 1;

% Trajectory define
xr = zeros(3,N);
k_step = 1;

B_first = B_stance_stop(:,5);
% assuming 15 degree estimated landing angle; 
B_last = [0.6*sin(0.262);0.15;-0.6*cos(0.262)];
B_mid = (B_first + B_last)/2;
B_land_L = [B_first,B_first,B_mid,B_last,B_last];
B_land_R = B_land_L;
B_land_R(2,:) = -B_land_R(2,:);
land_count = 0;
s_land = 0;
s_land0 = 0;
key = 0;
b_number = 8;
B_stance_walk = B_stance_start;
B_swing_walk = B_swing_start;

% Start the simulation
for i = 1:N
  
  % Upkeep ----------------------------------------------------------------
  
  t = t_sim(i);
  Rb = reshape(x(18:26),[3,3]);   % body rotation matrix
  [xd,ug] = harpy_model(x,u,p);   % Ground reaction forces
  [pos_foot_L, pos_foot_R] = func_foot_pos(x,params);
  [vel_foot_L, vel_foot_R] = func_foot_vel(x,params);
  
  % Extract roll pitch and yaw
  P = [1,0,0; 0,0,1; 0,1,0];
  R_test = P'*Rb*P;
  roll = -atan2(R_test(3,2),R_test(3,3));
  yaw = -asin(R_test(3,1));
  pitch = atan2(R_test(2,1),R_test(1,1));
  
%   % Check for tipover, end simulation early
%   if (norm([roll;pitch]) > pi/3)
%     disp('Tipover, simulation is terminated.')
% %     cost = cost + (N-i)^2*1000; % HEAVILY penalize the tipover
%     i_end = i;
%     break;
%   end
  
  % Rate of changes
  roll_D = (roll - roll_0)/p.dt;
  pitch_D = (pitch - pitch_0)/p.dt;
  yaw_D = (yaw - yaw_0)/p.dt;
  roll_0 = roll;
  pitch_0 = pitch;
  yaw_0 = yaw;
  
  % State machine and joint controller ------------------------------------
  
  
  % Order of the controller
  % u = [u_hipF_L; u_hipS_L; u_kneeS_L; 
  %      u_hipF_R; u_hipS_R; ; ; u_kneeS_R;
  %      u_th_L; u_th_R; f_th_L; f_th_R];
  
  % Gait timing phase / state machine
  if (t < p.t_start)
    % Wait until p.t_start to begin walking
    s = 0;  

    s0 = 0;
    
  else
      
        % Gait timing (time based), s = [0,1]
        s = (mod(t - p.t_start, p.gait_period))/p.gait_period;
        s_land = (mod(t - p.t_start, (p.gait_period/b_number)))/(p.gait_period/b_number);

        % End of gait phase (s = 0, s0 = 1)
        % Switch walking state and update the Bezier polynomial
        if (s < s0)
          k_step = k_step + 1;
              % Switch the swing leg state, 0 = left swing, 1 = right swing
              if (state == 0)
                state = 1;
              else
                state = 0;
              end

              if (k_step_old == 3 && k_step == 4)
                BL_stop = BL;
                BR_stop = BR;
              end
              
              % Update Bezier polynomial
              if (state == 0)
                % Left swing, right stance
                BL = B_swing;
                BR = B_stance;
              else
                % Right swing, left stance
                BR = B_swing;
                BL = B_stance;
              end

              if (k_step == 3)
                if (state == 0)
                  BL = B_swing_stop;
                  BR = B_stance_stop;
                else
                  BR = B_swing_stop;
                  BL = B_stance_stop;
                end
              end
              if k_step == 10
                if (state == 0)
                  BL = B_swing_stop;
                  BR = B_stance_stop;
                else
                  BR = B_swing_stop;
                  BL = B_stance_stop;
                end
              end
              if (k_step == 7 || k_step ==14)   
                  % Resume to normal gait posture after landing
                  l_pitch = asin(0.1*sin(pitch)/0.5) + pitch;
                  B_swing_walk(:,1) = B_last;
                  B_swing_walk(:,2) = B_last;
                  B_swing_walk(:,4) = [0.5*sin(-l_pitch + 0.1935);0.15;-0.6]; %normal swing angle: 0.1935 = atan2(B_swing(1,5),0.5)
                  B_swing_walk(:,5) = B_swing_walk(:,4); 
                  B_swing_walk(:,3) = [(B_swing_walk(1,5) + B_swing_walk(1,1))/2; 0.15;-0.4];
                  B_stance_walk(:,1) = B_last;
                  B_stance_walk(:,2) = B_last;
                  B_stance_walk(:,4) = [0.5*sin(-l_pitch - 0.3367);0.15;-0.6]; %normal stance angle: 0.3576 = atan2(B_stance(1,5),0.5)
                  B_stance_walk(:,5) = B_stance_walk(:,4);
                  B_stance_walk(:,3) = [(B_swing(1,5) + B_swing(1,1))/2; 0.15;(B_swing(3,5) + B_swing(3,1))/2];
                if (state == 0)
                  BL = B_swing_walk;
                  BR = B_stance_walk;
                else
                  BR = B_swing_walk;
                  BL = B_stance_walk;
                end
              end
              if k_step == 8 || k_step == 15
                 % Resuming to normal walking gait after normal posture
                B_swing_walk_2(:,1) = B_stance_walk(:,5);
                B_swing_walk_2(:,2) = B_stance_walk(:,4);
                B_swing_walk_2(:,4) = B_swing(:,4);
                B_swing_walk_2(:,5) = B_swing(:,5);
                B_swing_walk_2(:,4) = B_swing(:,4);
                B_swing_walk_2(:,3) = [(B_swing_walk_2(1,5) + B_swing_walk_2(1,1))/2;k(5);-k(6)];
                B_stance_walk_2(:,1) = B_swing_walk(:,5);
                B_stance_walk_2(:,2) = B_swing_walk(:,4);
                B_stance_walk_2(:,4) = B_stance(:,4);
                B_stance_walk_2(:,5) = B_stance(:,5);
                B_stance_walk_2(:,3) = [(B_stance_walk_2(1,5) + B_stance_walk_2(1,1))/2; k(8); -k(9)];
              if (state == 0)
                % Left swing, right stance
                BL = B_swing_walk_2;
                BR = B_stance_walk_2;
              else
                % Right swing, left stance
                BR = B_swing_walk_2;
                BL = B_stance_walk_2;
              end
                  
              end

              BR(2,:) = -BR(2,:); % Invert BR y axis
              k_step_old = k_step;  
          else
            
            
        end
%     s_land
%     s_land0
    s0 = s; % old phase value

  end
  
  if (k_step == 4 || k_step == 5 || k_step == 11 || k_step == 12)
%   if (k_step >= 4 && k_step <= 7)
    % Target foot position (body frame)
    pos_foot_L_ref = bezier4(BL_stop,1)*sigma(1);
    pos_foot_R_ref = bezier4(BR_stop,1)*sigma(1);  
  elseif k_step == 6
        B_last = [0;0.15;-0.6];
%         B_last = roty(pitch)*(pos_foot_L - x(5:7))
  elseif k_step == 7 || k_step == 8 || k_step == 14 || k_step == 15
      pos_foot_L_ref = bezier4(BL,s);
      pos_foot_R_ref = bezier4(BR,s);
  elseif k_step == 13
       B_last = [0;0.15;-0.6];
  else
    % Target foot position (body frame)
    pos_foot_L_ref = bezier4(BL,s)*sigma(1);
    pos_foot_R_ref = bezier4(BR,s)*sigma(1);   
  end

  % Inverse kinematics
  q_des_L  = inverse_kinematics(pos_foot_L_ref, p,'l');
  q_des_R = inverse_kinematics(pos_foot_R_ref, p,'r');
    
  % ERG -------------------------------------------------------------------
  
  % Center of pressure
  ugn_sum = ug(3) + ug(6);
  if (i == 1 || ugn_sum > 1e-20)
    % update only if either foot is touching the ground
    uc = (ug(3)*pos_foot_L + ug(6)*pos_foot_R)/ugn_sum;
  end
  
  xb = [x(5:7);x(12:14)]; % body pos ; vel (inertial)
  
  % Target center of mass
  if (i == 1)
    % Set initial positions
    xc_0 = xb(1:3); 
    xc_t = xc_0;
    xc_td = [target_spd;0;0];
    xw = xb;
    xr = xb;
  else
    % Target trajectory
       
    if (k_step < 5)
      xc_t_start = xc_t;
      t0 = t;
    end
    if (k_step < 12)
       t1 = t;
    end
    if (k_step <= 3 || (k_step >= 8 && k_step<=9) || (k_step >= 15))
      xc_t = xc_t + [target_spd; 0;0]*p.dt;
    elseif (k_step >= 5 && k_step <= 6)
      xc_t = xc_t + [target_spd*3; 0; (1)*sin(2*pi*(t-t0)/1.5)]*p.dt;
    elseif (k_step >= 12 && k_step <=13)
      xc_t = xc_t + [target_spd*3; 0; (0.3)*sin(2*pi*(t-t1)/1.5)]*p.dt; 
      
    elseif k_step == 7
    elseif k_step == 3 
              xc_t = xc_t + [(0.53 - 0.45)/p.gait_period; 0;0]*p.dt;
    elseif k_step == 10
       % disp('here')
            xc_t = xc_t + [(0.4)/p.gait_period; 0;0]*p.dt;
    end
      
    % Target center of mass velocity
    k_ddiff = 62.83; %628.3;
    wc_ddiff = 0.9937; %0.9391;
    xc_td = xc_td*wc_ddiff + (xc_t - xc_t0)*k_ddiff;
    
    xr = [xc_t; xc_td*0];
  end
  xc_t0 = xc_t;
  
  if ((k_step >= 5 && k_step <= 8) || (k_step >=12))
     %xw = xr; % uncomment to disable ERG
  end
  
  % Thruster controller
  v = zeros(4,1);
  y = xb(1:3) - uc(1:3);
  
  if (norm(y) < 1e-10)
    Y = zeros(3,3);
  else
    Y = y*inv(y.'*y)*y.';
  end
  
  v(1:3) = (eye(3) - Y)*( p.kp_th_erg*(xw(1:3) - xb(1:3)) + ...
                          p.kd_th_erg*(xw(4:6) - xb(4:6)));
  v(4) = y.'*(p.kp_r*(xw(1:3) - xb(1:3)) + p.kd_r*(xw(4:6) - xb(4:6))); % unused
  
  % Ground forces
  zr = xr;
  zw = xw;
  param_erg = [ p.mb + 2*p.mh + 2*p.mk ; p.g; p.kp_th_erg(:);...
                p.kd_th_erg(:); p.kp_r; p.kd_r];
  [dr, Jr] = func_groundforce_c(xb, xr, uc, param_erg);
  [dw, Jw] = func_groundforce_c(xb, xw, uc, param_erg);
  fg_est_r = dr + Jr*zr;
  fg_est_w = dw + Jw*zw;
  
  % ERG constraints
  Sr = [-sign(fg_est_r(1)), 0, p.kfs;...
       0, -sign(fg_est_r(2)), p.kfs;...
       0, 0, 1];
  Cw_r = Sr*Jr;
  Cl_r = Sr*fg_est_r - [0;0;10];
  Sw = [-sign(fg_est_w(1)), 0, p.kfs;...
       0, -sign(fg_est_w(2)), p.kfs;...
       0, 0, 1];
  Cw_w = Sw*Jw;
  Cl_w = Sw*fg_est_w - [0;0;10];
  
  hr = Cw_r*zr + Cl_r;
  hw = Cw_w*zw + Cl_w;
  
  % Evaluate the constraint row spaces and rate of change
  Nc = length(Cl_r);
  Cr = [];
  
  % Rates of changes
  wd_r = zw*0;  % Straight to r
  wd_n = zw*0;  % Normal to constraint equation
  wd_t = zw*0;  % Tangential to the constraint equation
  
  % Track the target reference as is with wd_r
  if min(hw) >= 0 || min(hr) >= 0
    wd_r = p.kr*(zr - zw);
  end
    
  % Track target reference tangentially with wd_t
  if min(hr) < 0
    % Create the basis for the nullspace of violated constraints
    for kk = 1:Nc
      if hr(kk) < 0 && hw(kk) >= 0
        Cr = [Cr;Cw_w(kk,:)];
      end
    end
    [Nr,~] = size(Cr);
    if (Nr == 0)
      Cn = null(Jr);
    else
      Cn = null(Cr);        % Evaluate the constraint null spaces
    end
    [~,Nn] = size(Cn);

    for kk = 1:Nn
        cn = Cn(:,kk);
        cn = cn/norm(cn);
        wd_t = wd_t + p.kt*(cn*cn')*(zr - zw);
    end  
  end
  
  % Both hw and hr have violated components
  % Push constraint into positive region if there is a violated component
  if min(hw) < 0 && min(hr) < 0
    [hw_min,kk] = min(hw);
    cn = Cw_w(kk,:)';
    cn = cn/norm(cn);

    if hr(kk) >= hw(kk)
      wd_n = p.kn1 * (cn*cn')*(zr - zw);
    else
      wd_n = -p.kn2 * (cn*cn')*(zr - zw);
    end
  end
%   wd_n = wd_n*0;
  
  % ERG update
  wd = wd_r + wd_n + wd_t;
  
  % Thruster force by ERG
  f_th_erg = v(1:3);
  
  
  
  % Adjust body pitch
  %   q_des_L(2) = q_des_L(2) - pitch/2;
%    q_des_R(2) = q_des_R(2) - pitch/2;  
  
  % Thruster action and angles calculations
  q_des_t = [pitch;pitch]; % points up

  thrust_roll = -p.kp_th*roll - p.kd_th*roll_D;
  thrust_yaw  = -p.kp_th*yaw - p.kd_th*yaw_D;  
  thrust_pitch = Rb'*[(-p.kp_th_pitch*pitch - p.kd_th_pitch*pitch_D);0;0];
  
  thrust_pos = p.kp_th_pos*(xr(1:3) - xb(1:3)) + ...
                          p.kd_th_pos*(xr(4:6) - xb(4:6));
  
  % Controller states calculations
  xj_des = [q_des_L(1:3);q_des_R(1:3);q_des_t]; % Target states
  if k_step == 5 || k_step == 6 || k_step == 12 || k_step ==13
      if mpc_t >0
          xj_des(2) = ref(1,3);
          xj_des(5) = ref(1,3);
      end
  end
  xj  = [x(1:2);x(27);x(3:4);x(28);x(35:36)];    % Current states
  
  xe_d = ((xj_des - xj) - xe)/p.dt;
  xe_i = xe_i + (xj_des - xj)*p.dt;
  xe = (xj_des - xj);
 
  %% Run MPC step == 5
  if k_step == 10
      % Reset
      mpc_t = 0;
  end
  
  
  if k_step == 5 || k_step == 6 || k_step == 12 || k_step == 13
      if mod (t,Ts) == 0 
         mpc_t = mpc_t + 1;
          if mpc_t == 1
              % record the jumping position as starting position
              mpc_x = [x(5);x(7)];
              %thrust_pos(3) = 52;
              mv = [thrust_pos(1);thrust_pos(3);u(2) + u(5)];
          end
          % Pitch defined opposite in the rom
          x0 = [x(5) - mpc_x(1);x(7) - mpc_x(2);-pitch;x(2);
                x(12);x(14);-pitch_D;x(9)];
          %u0 = [thrust_pos(1);thrust_pos(3);u(2)];
          if k_step == 5 || k_step ==6
            ref = func_ref(mpc_t,p_h,pitch,10);
          else
              ref = func_ref(mpc_t,p_h,pitch,3);
          end
         [mv,~,info] = nlmpcmove(nlobj,x0,mv,ref); 
     end
     % Hold the previous value;
     thrust_pos(1) = mv(1);
     thrust_pos(3) = mv(2);     
  end
  
  if mpc_t == 150
      mpc_t = 0;
  end
  %%
  % input states
  u(1:8) = p.kp*xe + p.kd*xe_d + p.ki*xe_i; % Joint PID controller
  % MPC input
  if k_step == 5 || k_step == 6 || k_step == 12 || k_step == 13
      u(2) = mv(3)/2;
      u(5) = mv(3)/2;
  end
  if (k_step >= 5)
    u(9:11) = [thrust_yaw;0;thrust_roll] + thrust_pos/2; % left thrusters
    u(12:14) = [-thrust_yaw;0;-thrust_roll] + thrust_pos/2; % right thrusters
  else
    u(9:11) = [thrust_yaw;0;thrust_roll] + f_th_erg/2 + thrust_pitch/2; % left thrusters
    u(12:14) = [-thrust_yaw;0;-thrust_roll] + f_th_erg/2 + thrust_pitch/2; % right thrusters
  end
        
  
  % Thrusters (inertial frame)
  
  
%   u(9) = thruster_model(volt_L,p);  % Thruster force model L
%   u(10) = thruster_model(volt_R,p);  % Thruster force model R
  
  % Record data -----------------------------------------------------------
  
  data.x(:,i) = x;
  data.xfL(:,i) = pos_foot_L;
  data.xfR(:,i) = pos_foot_R;
  data.vfL(:,i) = vel_foot_L;
  data.vfR(:,i) = vel_foot_R;
  data.qa_ref(:,i) = xj_des;
  data.xfL_ref(:,i) = pos_foot_L_ref;
  data.xfR_ref(:,i) = pos_foot_R_ref;
  data.ug(:,i) = ug;
  data.euler(:,i) = [roll;pitch;yaw];
  data.f_th(:,i) = u(9:14);
  data.cop(:,i) = uc;
  data.v(:,i) = v;
  data.xw(:,i) = xw;
  data.xr(:,i) = xr;
  data.hw(:,i) = hw;
  data.hr(:,i) = hr;
  data.wd_r(:,i) = wd_r;
  data.wd_n(:,i) = wd_n;
  data.wd_t(:,i) = wd_t;
  
  % March simulation ------------------------------------------------------
  
  % March states in time (RK4 integration)
  xk = march_rk4(x,u,p); % x_{k+1}  
  x = xk;     % Update the states for the next time step
  % March applied reference in time
  xw = xw + wd*p.dt;
  
  % TEST
%   sigma_d = [sigma(2); v(4)];
%   sigma = sigma + sigma_d*p.dt;
  
  % Check if simulation becomes unstable
  if (sum(isnan(x)) > 0)
    disp('ERROR: time integration unstable. Simulation is terminated.')
    break;
  end
  
end

data.i_end = i_end;
toc


%% Animate and Plot data

close all
plot_harpy_animation
% plot_harpy_data
%   

%% Local functions


% RK4 integration scheme
function xk = march_rk4(x,u,p)

  f1 = harpy_model(x, u, p);
  f2 = harpy_model(x + f1*p.dt/2,u,p);
  f3 = harpy_model(x + f2*p.dt/2,u,p);
  f4 = harpy_model(x + f3*p.dt,u,p);
  xk = x + (f1/6 + f2/3 + f3/3 + f4/6)*p.dt;
  
end

function x = bezier4(B,s)

% 4th order bezier curve
x = zeros([3,1]);
for i = 0:4
  x = x + nchoosek(4,i)*(1-s)^(4-i)*s^i*B(:,i+1);
end

end

function f = thruster_model(v,p)

if (v < p.v_th_min)
  f = 0;
elseif (v > p.v_th_max)
  f = p.f_th_max;
else
  f = (v - p.v_th_min) / (p.v_th_max - p.v_th_min) * p.f_th_max;
end

end










