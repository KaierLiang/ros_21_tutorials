% Symbollic derivations

clear all, close all, clc
addpath(['generated_functions',filesep])
addpath(['utility_functions',filesep])

%% Variable declarations

% parameters
syms m g mu_s t real
syms kpr kdr real

Kp = sym('KP_', [3,3], 'real');       % proportial gain
Kd = sym('KD_', [3,3], 'real');       % derivative gain


% time varying states
xc  = sym('xc_', [3,1], 'real');    % Body CoM position (inertial)
xcD = sym('xcD_', [3,1], 'real');   % Body CoM velocity (inertial)
xcDD = sym('xcDD_', [3,1], 'real'); % Body CoM acceleration (inertial)
rcDD = sym('rcDD_', [3,1], 'real'); % Body CoM acceleration reference (inertial)

xct  = sym('xct_', [3,1], 'real');    % target states
xctD = sym('xctD_', [3,1], 'real');   % target states vel

u  = sym('u_', [3,1], 'real');      % Center of pressure
uD = sym('uD_', [3,1], 'real');     % Center of pressure rate of change
uDD = u*0;                          % Assume zero CoP acceleration

xw = sym('w_', [6,1], 'real');      % applied reference

% Inputs
ft  = sym('ft_', [3,1], 'real');      % Thruster forces
syms vr real                          % pendulum Radial acceleration
 
%% Setup time derivatives


qq = [xc; xcD; u; uD];
qqd = [xcD; xcDD; uD; uDD];

Nq = length(qq);
    
syms x real
fxt = symfun('x(t)',[x,t]);
xx0 = sym('x_', [Nq,1], 'real');
xx = xx0;
xxd = xx;
for i = 1:Nq
  xx(i) = fxt(xx0(i),t);
  xxd(i) = diff(xx(i),t);
end


%% Equation of motion

% Setup constraint
Js = (xc - u).';
hs = vr;

% Dynamic: M*xcDD = h
M = m*eye(3);
lambda = inv(Js*inv(M)*Js.')*(Js*inv(M)*(-ft + [0;0;m*g] + hs));
h = simplify(ft - [0;0;m*g] + Js.'*lambda);
accel = inv(M)*h;

% Closed loop 
y = xc - u;
Y = y*inv(y.'*y)*y.';
ft_cl = (eye(3) - Y)*(Kp*(xct - xc) + Kd*(xctD - xcD));
vr_cl = y.'*(kpr*(xct - xc) + kdr*(xctD - xcD));

% Substitute the closed loop dynamics
h_cl = subs( subs( h, ft, ft_cl) ,vr ,vr_cl );

% Ground reaction forces
fg = Js.'*lambda;
fg_cl = subs( subs( fg, ft, ft_cl) ,vr ,vr_cl );

% Taylor expansion for the GRF
z = [xct;xctD;u(1:2)];
Jr = simplify(jacobian(fg_cl, z));
dr = fg_cl - Jr*z;


%% Output Matlab functions

states = [xc;xcD];
inputs = [ft;vr];
cop = u;
params = [m;g;Kp(:);Kd(:);kpr;kdr];
ref = [xct;xctD];


% Simulation equation of motion
matlabFunction(accel,'File', ...
 ['generated_functions', filesep, 'func_dynamics_c'], ...
  'Vars', {states, inputs, cop, params},...
  'Optimize',false);

% Ground forces
matlabFunction(dr, Jr, 'File', ...
 ['generated_functions', filesep, 'func_groundforce_c'], ...
  'Vars', {states, ref, cop, params},...
  'Optimize',false);


disp('Done!')

%% local functions

function dfdt = time_derivative(f,qq,qqd,xx,xxd,t)
% Time derivative work around  
dfdt = subs( subs( diff( subs(f,qq,xx) ,t), xxd,qqd), xx,qq );
end





