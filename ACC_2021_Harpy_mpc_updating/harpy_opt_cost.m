function [cost, data] = harpy_opt_cost(k,p)

% Optimization parameters, assume swing = right leg

state = 0;    % 0 = left swing, 1 = right swing

% Bezier polynomial parameters for left leg

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


% Simulation setup --------------------------------------------------------

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


% Params for the model
params = [p.mb; p.mh; p.mk; p.g; p.Ib_1; p.Ib_2; p.Ib_3; ...
            p.Ic_xx; p.Ic_yy; p.Ic_zz; p.L1_L; p.L2_L; p.L3_L; ...
            p.L1_R; p.L2_R; p.L3_R;p.l4a;p.l4b;p.kb_ground;...
            p.Lt_L; p.Lt_R];

% Record data for plotting if flag is set. 
if (p.flag_record_data == 1)
  % Data recording struct
  data.t = t_sim;             % Simulation time
  data.x = zeros(Nx,N);       % States
  data.xfL = zeros(3,N);  % Left foot position
  data.xfR = zeros(3,N);  % Right foot position
  data.vfL = zeros(3,N);
  data.vfR = zeros(3,N);
  data.qa_ref = zeros(8,N); % Joint reference
  data.xfL_ref = zeros(3,N); % Joint reference
  data.xfR_ref = zeros(3,N); % Joint reference
  data.ug = zeros(6,N); % Ground reaction forces [L;R]
  data.euler = zeros(3,N); % Body Euler angles
  data.f_th = zeros(6,N);
  
  data.i_end = N;
  data.cop = zeros(3,N); % center of pressure
  data.xw = zeros(6,N);
  data.xr = zeros(6,N);
  data.v = zeros(4,N);
  data.hw = zeros(3,N);
  data.hr = zeros(3,N);
  
  data.wd_r = zeros(6,N);
  data.wd_n = zeros(6,N);
  
else
  data = [];
end

cost = 0;   % Cost function: control action
s = 0;
s0 = 0;
u = zeros(14,1);



target_spd = norm((B_stance(:,5) - B_stance(:,1)))/p.gait_period*p.spd_scaler;
sigma = [1;0];


%% Begin Simulation

tic

i_end = N;

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
  
  % Check for tipover, end simulation early
  if (norm([roll;pitch]) > pi/4)
    disp('Tipover, simulation is terminated.')
    cost = cost + (N-i)^2*1000; % HEAVILY penalize the tipover
    i_end = i;
    break;
  end
  
  % Rate of changes
  roll_D = (roll - roll_0)/p.dt;
  pitch_D = (pitch - pitch_0)/p.dt;
  yaw_D = (yaw - yaw_0)/p.dt;
  roll_0 = roll;
  pitch_0 = pitch;
  yaw_0 = yaw;
  
%   % ERG -------------------------------------------------------------------
%   
%   % Center of pressure
%   ugn_sum = ug(3) + ug(6);
%   if (i == 1 || ugn_sum > 1e-20)
%     % update only if either foot is touching the ground
%     uc = (ug(3)*pos_foot_L + ug(6)*pos_foot_R)/ugn_sum;
%   end
%   
%   xb = [x(5:7);x(12:14)]; % body pos ; vel (inertial)
%   
%   % Target center of mass
%   if (i == 1)
%     % Set initial positions
%     xc_0 = xb(1:3); 
%     xc_t = xc_0;
%     xc_td = [target_spd;0;0];
%     xw = xb;
%     xr = xb;
%   else
%     % Target trajectory
%     xc_t = xc_0 + [ t*target_spd;...
%                 -0.025*sin(t*pi/p.gait_period);...
%                 abs(0.025*sin(t*pi/p.gait_period))*0];
%               
%     % Target center of mass velocity
%     k_ddiff = 62.83; %628.3;
%     wc_ddiff = 0.9937; %0.9391;
%     xc_td = xc_td*wc_ddiff + (xc_t - xc_t0)*k_ddiff;
%     xr = [xc_t; xc_td*0];
%   end
%   xc_t0 = xc_t;
%   
% %   xw = xr; % disable ERG
%   
%   
%   % Thruster controller
%   v = zeros(4,1);
%   y = xb(1:3) - uc(1:3);
%   
%   if (norm(y) < 1e-10)
%     Y = zeros(3,3);
%   else
%     Y = y*inv(y.'*y)*y.';
%   end
%   
%   v(1:3) = (eye(3) - Y)*( p.kp_th_erg*(xw(1:3) - xb(1:3)) + ...
%                           p.kd_th_erg*(xw(4:6) - xb(4:6)));
%   v(4) = y.'*(p.kp_r*(xw(1:3) - xb(1:3)) + p.kd_r*(xw(4:6) - xb(4:6)));
%   
%   % Ground forces
%   zr = xr;
%   zw = xw;
%   param_erg = [ p.mb + 2*p.mh + 2*p.mk ; p.g; p.kp_th_erg(:);...
%                 p.kd_th_erg(:); p.kp_r; p.kd_r];
%   [dr, Jr] = func_groundforce_c(xb, xr, uc, param_erg);
%   [dw, Jw] = func_groundforce_c(xb, xw, uc, param_erg);
%   fg_est_r = dr + Jr*zr;
%   fg_est_w = dw + Jw*zw;
%   
%   % ERG constraints
%   Sr = [-sign(fg_est_r(1)), 0, p.kfs;...
%        0, -sign(fg_est_r(2)), p.kfs;...
%        0, 0, 1];
%   Cw_r = Sr*Jr;
%   Cl_r = Sr*fg_est_r - [0;0;10];
%   Sw = [-sign(fg_est_w(1)), 0, p.kfs;...
%        0, -sign(fg_est_w(2)), p.kfs;...
%        0, 0, 1];
%   Cw_w = Sw*Jw;
%   Cl_w = Sw*fg_est_w - [0;0;10];
%   
%   hr = Cw_r*zr + Cl_r;
%   hw = Cw_w*zw + Cl_w;
%   
%   % Evaluate the constraint row spaces and rate of change
%   Nc = length(Cl_r);
%   Cr = [];
%   wd_n = zw*0;
%   
%   for kk = 1:Nc
%     if hr(kk) < 0 && hw(kk) >= 0
%       Cr = [Cr;Cw_r(kk,:)];
%     end
%   end
%   
% %   [rank(Jr),  rank(Jw)]
%   
%   
%   % Calculate vn
%   [Nr,~] = size(Cr);
%   if (Nr == 0)
%     Cn = null(Jr);
%   else
%     Cn = null(Cr);        % Evaluate the constraint null spaces
%   end
%   [~,Nn] = size(Cn);
%   
%   for kk = 1:Nn
%       cn = Cn(:,kk);
%       cn = cn/norm(cn);
%       wd_n = wd_n + p.kn*(cn*cn')*(zr - zw);
%   end
%   
%   % Calculate vr
%   wd_r = zr*0;
%   
%   rho = abs(min(hw));
%   if min(hw) < 0 
%     if min(hr) < 0 
%       
%       [hw_min,kk] = min(hw);
% %       rho = 1;
%       cn = Cw_w(kk,:)';
%       cn = cn/norm(cn);
%       
%       if hr(kk) >= hw(kk)
%         wd_r = wd_r + rho * p.kr * (cn*cn')*(zr - zw);
%       else
%         wd_r = wd_r - rho * p.kr * (cn*cn')*(zr - zw);
%       end
%       
% %       hw_neg_sum = 0;
% %       for kk = 1:length(Cl_w)
% %         if hw(kk) < 0
% %           hw_neg_sum = hw_neg_sum + norm(hw(kk));
% %         end
% %       end
% %       
% %       for kk = 1:length(Cl_w)
% %         rho = norm(hw(kk))/hw_neg_sum;
% %         
% %         
% %         cn = Cw_r(kk,:)';
% %         cn = cn/norm(cn);
% %         
% %         if (hw(kk) < 0)
% % %         [~,kk] = min(hw);
% %           
% %           if hr(kk) >= hw(kk)
% %             wd_r = wd_r + rho * p.kr * (cn*cn')*(zr - zw);
% %           else
% %             wd_r = wd_r - rho * p.kr * (cn*cn')*(zr - zw);
% %           end
% %         else
% % %           wd_r = wd_r + p.kr*(cn*cn')*(zr - zw);
% %         end
% %           
% %       end
%       
%     else
%       % Negative hw, Positive hr
%       wd_r = rho*p.kr*(zr - zw);
%     end
%   else
%     % Positive hw
%     wd_r = rho*p.kr*(zr - zw);
%   end
%   
%   
%   % ERG update
%   wd = wd_r + wd_n;
%   
%   
%   % Thruster force by ERG
%   f_th_erg = v(1:3);
  
  % Controller (zero-order hold) ------------------------------------------
  
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
    
    % End of gait phase (s = 0, s0 = 1)
    % Switch walking state and update the Bezier polynomial
    if (s < s0)
      
      % Switch the swing leg state, 0 = left swing, 1 = right swing
      if (state == 0)
        state = 1;
      else
        state = 0;
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
      BR(2,:) = -BR(2,:); % Invert BR y axis
    end
    
    s0 = s; % old phase value
  end
  
  % Target foot position
  pos_foot_L_ref = bezier4(BL,s)*sigma(1);
  pos_foot_R_ref = bezier4(BR,s)*sigma(1);
  
  % Inverse kinematics
  q_des_L  = inverse_kinematics(pos_foot_L_ref, p,'l');
  q_des_R = inverse_kinematics(pos_foot_R_ref, p,'r');
  
  % Adjust body pitch
  %   q_des_L(2) = q_des_L(2) - pitch/2;
%    q_des_R(2) = q_des_R(2) - pitch/2;  
  
  % Thruster action and angles calculations
  q_des_t = [pitch;pitch]; % points up

  thrust_roll = -p.kp_th*roll - p.kd_th*roll_D;
  thrust_yaw  = -p.kp_th*yaw - p.kd_th*yaw_D;
  
  
  
  
  
  
  % Controller states calculations
  xj_des = [q_des_L(1:3);q_des_R(1:3);q_des_t]; % Target states
  xj  = [x(1:2);x(27);x(3:4);x(28);x(35:36)];    % Current states
  
  xe_d = ((xj_des - xj) - xe)/p.dt;
  xe_i = xe_i + (xj_des - xj)*p.dt;
  xe = (xj_des - xj);
  
  % input states
  u(1:8) = p.kp*xe + p.kd*xe_d + p.ki*xe_i; % Joint PID controller
  
%   u(9:11) = [thrust_yaw;0;thrust_roll] + f_th_erg/2;
%   u(12:14) = [-thrust_yaw;0;-thrust_roll] + f_th_erg/2;
  
  
%   u(9) = thruster_model(volt_L,p);  % Thruster force model L
%   u(10) = thruster_model(volt_R,p);  % Thruster force model R
  
  % Record data -----------------------------------------------------------
  
  if (p.flag_record_data == 1)
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
%     data.cop(:,i) = uc;
%     data.v(:,i) = v;
%     data.xw(:,i) = xw;
%     data.xr(:,i) = xr;
%     data.hw(:,i) = hw;
%     data.hr(:,i) = hr;
%     
%     data.wd_r(:,i) = wd_r;
%     data.wd_n(:,i) = wd_n;
  end
  
  % March simulation ------------------------------------------------------
  
  xk = march_rk4(x,u,p); % x_{k+1}  
  x = xk;     % Update the states for the next time step
  
%   xw = xw + wd*p.dt;
%   
%   sigma_d = [sigma(2); v(4)]*0;
%   sigma = sigma + sigma_d*p.dt;
  
  % Check if simulation becomes unstable
  if (sum(isnan(x)) > 0)
    disp('ERROR: time integration unstable. Simulation is terminated.')
    cost = cost + (N-i)*100;
    break;
  end
  
  % Update cost -----------------------------------------------------------
  
  vb = Rb'*x(12:14); % Body frame velocity
  
  if (t >= p.t_start + 1*p.gait_period)
    cost = cost + norm(u)*p.Qu;                   % input
    cost = cost + norm(vb(2))*p.Qvy;              % sideways velocity
    cost = cost + norm(x(12) - p.v_des)*p.Qvx;    % forward velocity
    cost = cost + norm(x(15:17))*p.Qw;            % ang vel
    cost = cost + norm(x(7) - k(3))*p.Qh;         % height
  end
end

if (p.flag_record_data == 1)
  data.i_end = i_end;
end


toc


end

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


















