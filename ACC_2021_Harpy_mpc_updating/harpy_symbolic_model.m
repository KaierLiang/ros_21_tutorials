% Harpy symbolic model

clear all, close all, clc

addpath(['generated_functions',filesep]) 
addpath(['utility_functions',filesep])    

%% Symbolic parameters

% Constants
syms mb mh mk g Ib_1 Ib_2 Ib_3 t real
syms Ic_xx Ic_yy Ic_zz real  % Generic Cylinder Actuator
syms l4a l4b kb_ground real

% Constant states
L1_L = sym('L1_L_', [3,1], 'real');   % Body to pelvis
L2_L = sym('L2_L_', [3,1], 'real');   % Pelvis to hip
L3_L = sym('L3_L_', [3,1], 'real');   % hip to knee

L1_R = sym('L1_R_', [3,1], 'real');   % Body to pelvis
L2_R = sym('L2_R_', [3,1], 'real');   % Pelvis to hip
L3_R = sym('L3_R_', [3,1], 'real');   % hip to knee

Lt_L = sym('Lt_L_', [3,1], 'real');   % body to thruster L
Lt_R = sym('Lt_R_', [3,1], 'real');   % body to thruster R

Ib = diag([Ib_1,Ib_2,Ib_3]); %MoI for the Body (Body frame)
Ih = diag([Ic_xx,Ic_yy,Ic_zz]); % MoI for hip w.r.t. hip frame
Ik = diag([Ic_xx,Ic_yy,Ic_zz]); % MoI for knee w.r.t. hip frame

% Body states
x_body = sym('xb_', [3,1], 'real');   % Body linear position
R_body = sym('Rb_', [3,3], 'real');   % Body rotation matrix

% Body states velocities
xD_body = sym('vb_', [3,1], 'real');  % Body linear velocity
w_body = sym('wb_', [3,1], 'real');   % Body angular velocity (body frame)
RD_body = R_body * skew(w_body);      % Rotation matrix time derivative

% Body states accelerations
xDD_body = sym('vbD_', [3,1], 'real');  % Body linear velocity
wD_body = sym('wbD_', [3,1], 'real');   % Body angular velocity (body frame)

% Joint states 
q_hipF = sym('q_hF_', [2,1], 'real');   % Hip frontal (Left ; Right)
q_hipS = sym('q_hS_', [2,1], 'real');   % Hip sagittal (Left ; Right)
q_kneeS = sym('q_kS_', [2,1], 'real');  % Knee sagittal (Left ; Right)

% Joint states velocities
qD_hipF = sym('qD_hF_', [2,1], 'real');   % Hip frontal (Left ; Right)
qD_hipS = sym('qD_hS_', [2,1], 'real');   % Hip sagittal (Left ; Right)
qD_kneeS = sym('qD_kS_', [2,1], 'real');  % Knee sagittal (Left ; Right)

% Joint states acceleration
qDD_hipF = sym('qDD_hF_', [2,1], 'real');   % Hip frontal (Left ; Right)
qDD_hipS = sym('qDD_hS_', [2,1], 'real');   % Hip sagittal (Left ; Right)
qDD_kneeS = sym('qDD_kS_', [2,1], 'real');  % Knee sagittal (Left ; Right)

syms theta thetaD thetaDD real

% Thruster states
f_th = sym('ft_', [2,1], 'real');  % Thruster force (Left ; Right)
q_th = sym('qt_', [2,1], 'real');  % Thruster angle (Left ; Right)

% Setup time derivatives
qq = [x_body; xD_body; w_body; q_hipF; q_hipS; q_kneeS; ...
      qD_hipF; qD_hipS; qD_kneeS; R_body(:)];

qqd = [xD_body; xDD_body; wD_body; qD_hipF; qD_hipS; qD_kneeS; ...
      qDD_hipF; qDD_hipS; qDD_kneeS; RD_body(:)];

Nq = length(qq);
    
syms x real
ft = symfun('x(t)',[x,t]);
xx0 = sym('x_', [Nq,1], 'real');
xx = xx0;
xxd = xx;
for i = 1:Nq
  xx(i) = ft(xx0(i),t);
  xxd(i) = diff(xx(i),t);
end

% the workaround
% test = v_body;
% test_dxdt = time_derivative(v_body,qq,qqd,xx,xxd,t);

%% Kinematic formulation

% Inertial positions (center of mass)
pos_body = x_body;

pos_hip_L = pos_body + R_body*L1_L + R_body*rot_x(q_hipF(1))*L2_L;
pos_hip_R = pos_body + R_body*L1_R + R_body*rot_x(q_hipF(2))*L2_R;

pos_knee_L = pos_hip_L + R_body*rot_x(q_hipF(1))*rot_y(q_hipS(1))*L3_L;
pos_knee_R = pos_hip_R + R_body*rot_x(q_hipF(2))*rot_y(q_hipS(2))*L3_R;


L4_L = [-l4a*cos(q_kneeS(1)) ; 0 ; -(l4b + l4a*sin(q_kneeS(1)))];
L4_R = [-l4a*cos(q_kneeS(2)) ; 0 ; -(l4b + l4a*sin(q_kneeS(2)))];
pos_foot_L = pos_knee_L + R_body*rot_x(q_hipF(1))*rot_y(q_hipS(1))*L4_L;
pos_foot_R = pos_knee_R + R_body*rot_x(q_hipF(2))*rot_y(q_hipS(2))*L4_R;

vel_foot_L = time_derivative(pos_foot_L, qq, qqd, xx, xxd, t);
vel_foot_R = time_derivative(pos_foot_R, qq, qqd, xx, xxd, t);
acc_foot_L = time_derivative(vel_foot_L, qq, qqd, xx, xxd, t);
acc_foot_R = time_derivative(vel_foot_R, qq, qqd, xx, xxd, t);


vel_body = time_derivative(pos_body, qq, qqd, xx, xxd, t);
vel_hip_L = time_derivative(pos_hip_L, qq, qqd, xx, xxd, t);
vel_hip_R = time_derivative(pos_hip_R, qq, qqd, xx, xxd, t);

vel_knee_L = time_derivative(pos_knee_L, qq, qqd, xx, xxd, t);
vel_knee_R = time_derivative(pos_knee_R, qq, qqd, xx, xxd, t);


ang_hip_L_body = w_body + time_derivative([q_hipF(1);0;0], qq, qqd, xx, xxd, t);
ang_hip_R_body = w_body + time_derivative([q_hipF(2);0;0], qq, qqd, xx, xxd, t);
ang_hip_L = rot_x(q_hipF(1))'*ang_hip_L_body;
ang_hip_R = rot_x(q_hipF(2))'*ang_hip_R_body;

ang_knee_L = ang_hip_L + time_derivative([0;q_hipS(1);0], qq, qqd, xx, xxd, t);
ang_knee_R = ang_hip_R + time_derivative([0;q_hipS(2);0], qq, qqd, xx, xxd, t);

% configuration
q = [q_hipF(1);q_hipS(1);q_hipF(2);q_hipS(2);x_body];
qd = time_derivative(q,qq,qqd,xx,xxd,t);

%% Torque

%L = 1/2*mb*vel_body'*vel_body - mb*g*pos_body(3);

% dL_dx = jacobian(L, x_body).';     % del L / del q
% dL_dv = jacobian(L, xD_body).';    % del L / del q_dot
% dtdt_dLdv = time_derivative(dL_dv,qq,qqd,xx,xxd,t); % d/dt(del L / del q_dot)
% eom = dtdt_dLdv - dL_dx;
%%eom1

%% for eom1
Kl = 1/2*(mb*vel_body'*vel_body + ...
          mh*vel_hip_L'*vel_hip_L     + mh*vel_hip_R'*vel_hip_R + ...
          mk*vel_knee_R'*vel_knee_R   + mk*vel_knee_L'*vel_knee_L);

P = mb*g*pos_body(3)+mh*g*pos_hip_L(3)+mh*g*pos_hip_R(3)+mk*g*pos_knee_L(3)+mk*g*pos_knee_R(3);

Ka = 1/2*(w_body'*Ib*w_body + ...
          ang_hip_R'*Ih*ang_hip_R     + ang_hip_L'*Ih*ang_hip_L + ...
          ang_knee_R'*Ik*ang_knee_R   + ang_knee_L'*Ik*ang_knee_L);

L = Kl+Ka-P;

% eom 1
dL_dx=jacobian(L,q); % del L / del q
dL_dv=jacobian(L,qd); %del L / del q_dot
dtdt_dLdv = time_derivative(dL_dv,qq,qqd,xx,xxd,t); % d/dt(del L / del q_dot)
eom1 = (dtdt_dLdv - dL_dx)';

%for eom2
dL_dw_body = jacobian(L,w_body); %del L / del w_body;
dtdt_dLdw_body = time_derivative(dL_dw_body,qq,qqd,xx,xxd,t);% d/dt(del L / del q_dot)
term1 = dtdt_dLdw_body; 
term2 = cross(w_body,dL_dw_body);
term3 = zeros(3,1);
for i = 1:3
    term3 = term3 + cross( R_body(i,:) , jacobian(L , R_body(i,:)) )';
end
eom2 = term1' + term2' + term3;

%%

qd_eom = [qd; w_body];

% Form M*accel + h = 0
eom = [eom1;eom2];
qdd_eom = time_derivative(qd_eom,qq,qqd,xx,xxd,t);
M = jacobian(eom,qdd_eom);
h = subs(eom,qdd_eom,qdd_eom*0);

%% Constraint equation

% acc_foot = Js*acc_states + hs = 0
Js_L = jacobian(acc_foot_L, qdd_eom);
hs_L = subs(acc_foot_L, qdd_eom, qdd_eom*0);
Js_R = jacobian(acc_foot_R, qdd_eom);
hs_R = subs(acc_foot_R, qdd_eom, qdd_eom*0);


%% Joint actuation

z_hip_L = (ang_hip_L - rot_x(q_hipF(1))'*w_body);
z_hip_R = (ang_hip_R - rot_x(q_hipF(2))'*w_body);

z_knee_L = (ang_knee_L - ang_hip_L);
z_knee_R = (ang_knee_R - ang_hip_R);

syms damping real;

B_hip_L = -jacobian(z_hip_L,qd_eom)'*z_hip_L*damping;
B_hip_R = -jacobian(z_hip_R,qd_eom)'*z_hip_R*damping;
B_knee_L = -jacobian(z_knee_L,qd_eom)'*z_knee_L*damping;
B_knee_R = -jacobian(z_knee_R,qd_eom)'*z_knee_R*damping;

u_damping = B_hip_L + B_knee_L + B_hip_R + B_knee_R;

%% Ground reaction forces

% Ground contact forces on the foot
Jg_L = jacobian(vel_foot_L, qd_eom)';
Jg_R = jacobian(vel_foot_R, qd_eom)';
Bg = [Jg_L,Jg_R];

% Ground z rotation damping
alpha_L = ( l4b + l4a*sin(q_kneeS(1)) ) / (l4a*cos(q_kneeS(1)));
alpha_R = ( l4b + l4a*sin(q_kneeS(2)) ) / (l4a*cos(q_kneeS(2)));
alphaD_L = time_derivative(alpha_L, qq, qqd, xx, xxd, t);
alphaD_R = time_derivative(alpha_R, qq, qqd, xx, xxd, t);

% Knee angular velocity
qD_foot_L = 1/(1 + alpha_L^2)*alphaD_L;
qD_foot_R = 1/(1 + alpha_R^2)*alphaD_R;

% Foot angular velocity in inertial frame
angvel_foot_L = R_body*(w_body + [qD_hipF(1);0;0] + ...
                rot_x(q_hipF(1))*[0;qD_hipS(1) + qD_foot_L;0]  );
angvel_foot_R = R_body*(w_body + [qD_hipF(2);0;0] + ...
                rot_x(q_hipF(2))*[0;qD_hipS(2) + qD_foot_R;0]  );

% Generalized torque due to floor z rotational damping
B_uzL = jacobian(angvel_foot_L(3), qd_eom)';
B_uzR = jacobian(angvel_foot_R(3), qd_eom)';
tau_gz_L = -kb_ground*(B_uzL*angvel_foot_L(3));
tau_gz_R = -kb_ground*(B_uzR*angvel_foot_R(3));

%% Thruster dynamics

% Thruster positions and velocities
pos_thruster_L = pos_body + R_body*(Lt_L);
pos_thruster_R = pos_body + R_body*(Lt_R);
vel_thruster_L = time_derivative(pos_thruster_L, qq, qqd, xx, xxd, t);
vel_thruster_R = time_derivative(pos_thruster_R, qq, qqd, xx, xxd, t);

% Generalized thruster forces
Bt_L = jacobian(vel_thruster_L, qd_eom)';
Bt_R = jacobian(vel_thruster_R, qd_eom)';
ut_L = Bt_L*R_body*rot_y(q_th(1))*[0;0;f_th(1)];
ut_R = Bt_R*R_body*rot_y(q_th(2))*[0;0;f_th(2)];
ut = ut_L + ut_R;

%%
% System states
states = [q ;qd_eom; R_body(:); q_kneeS; qD_kneeS];
extra_states = qDD_kneeS;

% System parameters
params = [ mb; mh; mk; g; Ib_1; Ib_2; Ib_3; Ic_xx; Ic_yy; Ic_zz;...
          L1_L; L2_L; L3_L; L1_R; L2_R; L3_R; l4a; l4b; ...
          kb_ground; Lt_L; Lt_R];
      
% NOTE: using filesep makes folder names usable on all OS

% Dynamic equation of motion for the massed system
matlabFunction(M, h, Bg, 'File', ...
  ['generated_functions', filesep, 'func_MhBg'], 'Vars', {states, params},...
   'Optimize',false);

matlabFunction(tau_gz_L, tau_gz_R, 'File', ...
  ['generated_functions', filesep, 'func_ground_rot_damping'], 'Vars', {states, params},...
   'Optimize',false);
 
matlabFunction(ut, 'File', ...
  ['generated_functions', filesep, 'func_thrusters'], 'Vars', {states, params, q_th, f_th},...
   'Optimize',false);
 
 
 
 
% matlabFunction(Js_L, hs_L, 'File', ...
%   ['generated_functions', filesep, 'func_foot_Jshs_L'], 'Vars', {states, params, qDD_kneeS},...
%    'Optimize',false);
% 
% matlabFunction(Js_R, hs_R, 'File', ...
%   ['generated_functions', filesep, 'func_foot_Jshs_R'], 'Vars', {states, params, qDD_kneeS},...
%    'Optimize',false);
 
matlabFunction(pos_foot_L, pos_foot_R, 'File', ...
 ['generated_functions', filesep, 'func_foot_pos'], 'Vars', {states, params},...
  'Optimize',false);

matlabFunction(vel_foot_L, vel_foot_R, 'File', ...
 ['generated_functions', filesep, 'func_foot_vel'], 'Vars', {states, params},...
  'Optimize',false);
 
% matlabFunction(Jv_L, hv_L, 'File', ...
%   ['generated_functions', filesep, 'func_foot_Jshs_vel_L'], 'Vars', {states, params},...
%    'Optimize',false);
% 
% matlabFunction(Jv_R, hv_R, 'File', ...
%   ['generated_functions', filesep, 'func_foot_Jshs_vel_R'], 'Vars', {states, params},...
%    'Optimize',false);



disp('done!')



%% local functions

function dfdt = time_derivative(f,qq,qqd,xx,xxd,t)
% Time derivative work around  
dfdt = subs( subs( diff( subs(f,qq,xx) ,t), xxd,qqd), xx,qq );
end



















