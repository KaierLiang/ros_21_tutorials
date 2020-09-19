% harpy's inverse kinematic
% input: pos_foot (swing foot position in body frame) 3x1
% output: close-form equations

function F= inverse_kinematics(pos_foot, p, side)

% Hip frontal
q_hipF = -solve_q_hipF(pos_foot, p, side);
                  
% Knee sagittal
if side == 'l'
  L3 = norm(p.L3_L);
  Lf2 = rot_x(q_hipF)'*(pos_foot - p.L1_L) - p.L2_L;
else
  L3 = norm(p.L3_R);
  Lf2 = rot_x(q_hipF)'*(pos_foot - p.L1_R) - p.L2_R;
end
num = norm(Lf2)^2 - (L3+p.l4b)^2 - p.l4a^2;
den = 2*p.l4a* (L3 + p.l4b);
q_kneeS = asin(num/den);
                  
% Hip sagittal
delta_hip_s = atan2(p.l4a*cos(q_kneeS), (L3 + p.l4a*sin(q_kneeS) + p.l4b));
q_hipS = atan2(Lf2(1), - Lf2(3)) + delta_hip_s;

% invert the angles rotate about y axis
F=[q_hipF; -q_hipS; q_kneeS];

end

function q = solve_q_hipF(pos_foot, p, side)

if side == 'l'
    L2 = norm(p.L2_L);
    Lf = pos_foot - p.L1_L;
    c3 = L2;
     
%     F = Lf(2)*cos(q_hipF) + Lf(3)*sin(q_hipF) - L2;
else
    L2 = norm(p.L2_R);
    Lf = pos_foot - p.L1_R;
    c3 = -L2;
    
%     F = Lf(2)*cos(q_hipF) + Lf(3)*sin(q_hipF) + L2;
end
c1 = Lf(3);
c2 = Lf(2);

% Find roots
r = roots([c2 + c1*i, -2*c3, -i*c1 + c2]);
q1 = real(log(r(1))*(-i));
q2 = real(log(r(2))*(-i));

% The solution angles must be within +- pi/2
if (q1 > -pi/2 && q1 < pi/2)
  q = q1;
else
  q = q2;
end

end


function Rx = rot_x(theta)
Rx = [1, 0, 0; ...
  0, cos(theta), -sin(theta); ...
  0, sin(theta), cos(theta)];
end
