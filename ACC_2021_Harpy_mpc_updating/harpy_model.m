function [xd, ug] = harpy_model(x,u,p)

  % Solve dx/dt
  
  % states = [q ;qd_eom; R_body(:); q_kneeS; qD_kneeS];
  % params = [ mb; mh; mk; g; Ib_1; Ib_2; Ib_3; Ic_xx; Ic_yy; Ic_zz;...
  %        L1_L; L2_L; L3_L; L1_R; L2_R; L3_R; l4a; l4b];
  
  % x(1:2) = [q_hipF Left; q_hipS Left; ];
  % x(3:4) = [q_hipF Right; q_hipS Right];
  % x(5:7) = body pos (inertial)
  
  % x(8:9) = [qD_hipF Left; qD_hipS Left];
  % x(10:11) = [qD_hipF Right; qD_hipS Right];
  % x(12:14) = body vel(inertial)
  % x(15:17) = body angular velocity (body frame)
  
  % x(18:26) = rotation matrix R_body = reshape(x(18:26),[3,3])
  
  % x(27:28) = [q_kneeS Left; q_kneeS Right];
  % x(29:30) = [qD_kneeS Left; qD_kneeS Right];  
  % x(31:34) = lugre friction model bristles parameters
  % x(35:36) = thruster sagittal angles
  % x(37:38) = thruster sagittal ang vel  
  
  % u = input = [u_left_leg; u_right_leg, u_thruster, f_thruster], 
  % leg order: [u_hipF; u_hipS; u_kneeS]  
  % u_thruster = thruster joint accel
  % f_thruster = thruster force
  
  params = [p.mb; p.mh; p.mk; p.g; p.Ib_1; p.Ib_2; p.Ib_3; ...
            p.Ic_xx; p.Ic_yy; p.Ic_zz; p.L1_L; p.L2_L; p.L3_L; ...
            p.L1_R; p.L2_R; p.L3_R;p.l4a;p.l4b;p.kb_ground;...
            p.Lt_L; p.Lt_R];
  
  % Dynamic model        
  [M,h,Bg] = func_MhBg(x,params);
  Rb = reshape(x(18:26), [3,3]);
  
  % Joint actuation (damping + controller torque)
  u_jointL = -p.damping*x(8:9) + u(1:2);
  u_jointR = -p.damping*x(10:11) + u(4:5);
  u_kneeS = [u(3); u(6)]; % knee acceleration (massless component)
  u_dyn = [u_jointL; u_jointR; zeros(6,1)];
  
  % Thruster actuation
  qdd_th = u(7:8);  % thruster joint acceleration
  f_th = u(9:14);   % thruster force
%   u_th = func_thrusters(x,params,x(35:36),f_th); % Generalized force
  u_th = func_thrusters(x,params,f_th); % Generalized force
  
  % Foot positions and velocities
  [pos_foot_L, pos_foot_R] = func_foot_pos(x,params);
  [vel_foot_L, vel_foot_R] = func_foot_vel(x,params);
  
  % Ground reaction forces
  z = x(31:34);
  if (p.use_lugre == 1)
    % LuGre friction model
    [fg_L, zd_L] = ground_force_model_lugre(pos_foot_L, vel_foot_L, z(1:2), p);
    [fg_R, zd_R] = ground_force_model_lugre(pos_foot_R, vel_foot_R, z(3:4), p);
    zd = [zd_L; zd_R];
  else
    % Coulomb and viscous friction model
    fg_L = ground_force_model(pos_foot_L, vel_foot_L, p);
    fg_R = ground_force_model(pos_foot_R, vel_foot_R, p);
    zd = zeros(4,1);
  end
  ug = [fg_L; fg_R];
  
%   u_th = zeros(10,1);
%   u_th(7) = 60;

  % Ground yaw damping acting on the foot
  [tau_gz_L, tau_gz_R] = func_ground_rot_damping(x,params);
  if (pos_foot_L(3) > 0)
    tau_gz_L = tau_gz_L*0;
  end
  if (pos_foot_R(3) > 0)
    tau_gz_R = tau_gz_R*0;
  end
  tau_gz = tau_gz_L + tau_gz_R;
  
  % Calculate the dynamic states acceleration
  h0 = -h + u_dyn + Bg*ug + tau_gz + u_th;
  
  if (p.use_planar_dynamics == 1)
%   if (1)
    % Constraint body
%     Jc = [zeros(6,4),eye(6)];
%     Mc = [M, -Jc'; Jc, zeros(6,6)];
%     hc = [h0;zeros(6,1)];
%     temp = Mc\hc;     % [qdd; lambda]
%     qdd = temp(1:10);
        
    % Constraint y position, roll and yaw    
    Jc = [0,0,0,0, 0,1,0,0,0,0; ...
          0,0,0,0, 0,0,0,1,0,0; ...
          0,0,0,0, 0,0,0,0,0,1];
    Mc = [M, -Jc'; Jc, zeros(3,3)];
    hc = [h0;zeros(3,1)];
    temp = Mc\hc;         % [qdd; lambda]
    qdd = temp(1:10);     % Dynamic state acceleration
  else
    qdd = M\h0;
  end
  
  % dx/dt  
  xd = x*0;
  xd(1:7) = x(8:14);              % Dynamic velocity
  xd(8:17) = qdd;                 % M\(-h + u_dyn) = dynamic acceleration
  xd(18:26) = Rb*skew(x(15:17));  % Body rotation matrix ang vel in SO(3)
  xd(27:28) = x(29:30);           % Knee ang vel (massless system)
  xd(29:30) = u_kneeS;            % Knee acceleration (massless system)
  xd(31:34) = zd;                 % LuGre bristle model
  xd(35:36) = x(37:38);           % Thruster angle (sagittal, L/R)
  xd(37:38) = qdd_th;             % Thruster ang vel
end

%% Extra local functions



function f = ground_force_model(x,v,p)
  % Ground force model using basic friction model

  % x = foot inertial position vector
  % v = foot inertial velocity vector
  
  if (x(3) <= p.ground_z)
    fz = ground_model(x(3), v(3), p);
    fx = friction_model(v(1), fz, p);
    fy = friction_model(v(2), fz, p);
    f = [fx;fy;fz];
  else
    f = [0;0;0];
  end
  
end

function f = friction_model(v,N,p)

  % v = velocity along the surface, N = normal force 
  % Coulomb and viscous friction model
  
  fc = p.kfc * norm(N) * sign(v);
  fs = p.kfs * norm(N) * sign(v);
%   f = -(p.kfc * norm(N) * sign(v) + p.kfb * v);

  f = -(fc + (fs-fc)*exp(-norm(v/p.vs)^2) + p.kfb * v);
  
end

function [f,zd] = ground_force_model_lugre(x,v,z,p)

  % LuGre friction model. Solve for friction force and dz/dt
  % x = foot inertial position vector
  % v = foot inertial velocity vector
  % z = vector of average bristle displacement in Dahl and LuGre model
  %       in x and y directions
  
  if (x(3) <= p.ground_z)
    fz = ground_model(x(3), v(3), p);
    flag = true;
  else
    fz = 0;
    flag = false;
  end
    
  % Calculate zdx
  [fx,zdx] = friction_model_lugre(v(1), z(1), fz, p);
  [fy,zdy] = friction_model_lugre(v(2), z(2), fz, p);
  
  % fx and fy are zero if not contacting the ground
  if (flag == false)
    fx = 0;
    fy = 0;
  end
  
  f = [fx;fy;fz];
  zd = [zdx;zdy];
end

function [f,zd] = friction_model_lugre(v,z,N,p)

  % v = velocity along the surface, N = normal force 
  % z = average bristle displacement and rate (Dahl and LuGre model)
  
  fc = p.kfc * norm(N); % Coulomb friction
  fs = p.kfs * norm(N); % Static friction  
  
  gv = fc + (fs-fc)*exp(-norm(v/p.vs)^2);
  
  % If gv smaller than certain threshold, assume leg no longer contacting
  % the ground. Rapidly decay the dz/dt in that case.
  if (gv < p.gv_min)
    % Lose contact or barely any normal force
    zd = - p.zd_max * z;         
    f = 0;
  else
    % LuGre model
    zd = v - p.s0 * norm(v) / gv * z;         % dz/dt
    f = -(p.s0 * z + p.s1 * zd + p.kfb * v);  % friction force
  end
  
end

function f = ground_model(x,v,p)
  % Assume x < ground_z


  if (p.use_damped_rebound == 1)
    
    % damped rebound model
    f = -p.kp_g * (x - p.ground_z) - p.kd_g * v;
  else 
    % undamped rebound model
    if (v < 0)
      f = -p.kp_g * (x - p.ground_z) - p.kd_g * v;
    else
      f = -p.kp_g * (x - p.ground_z);
    end
  end
end















