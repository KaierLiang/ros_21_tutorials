clear all, close all
clc
%% parameters
%syms mb mk L_k Ib Ik g t L_h
syms theta q t real% pitch, shift foot angle
syms theta_D q_D real
syms theta_DD q_DD real

g = 9.8;
mb = 2;
mk = 0.5;
Ib = 0.01;
Ik = 0.01;
L_h = 0.1;
L_k = 0.3;
mh = 1;

% syms u_h
% u = sym('u_',[2,1],'real');
c_body = sym('cb_', [2,1], 'real');
cD_body = sym('vb_', [2,1], 'real');  % Body linear velocity
cDD_body = sym('ab_',[2,1],'real');

p_k = c_body + rot2_y(theta)*[0;-L_h] + rot2_y(theta)*rot2_y(q)*[0;-L_k];
p_h = c_body + rot2_y(theta)*[0;-L_h];

qq = [c_body; cD_body; theta; theta_D; q; q_D];
qqd = [cD_body; cDD_body; theta_D; theta_DD; q_D; q_DD];
% % State
% x = [c_body;theta;q];
% x_d = [cD_body;theta_d;q_d];
%Energy
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
pd_k = time_derivative(p_k, qq, qqd, xx, xxd, t);
pd_h = time_derivative(p_h, qq, qqd, xx, xxd, t);

K = 1/2*mb*(cD_body'*cD_body) + 1/2*(2*mk)*(pd_k'*pd_k) + 1/2*Ib*theta_D^2 + 1/2*Ik*(theta_D + q_D)^2 + 1/2*(2*mh)*(pd_h'*pd_h);
V = mb*g*c_body(2) + 2*mk*g*p_k(2) + 2*mh*g*p_h(2);
L = K - V;
%
%x = [c_body;theta;q];
%
q_s = [c_body;theta;q];
q_sd = time_derivative(q_s,qq,qqd,xx,xxd,t);
q_sdd = time_derivative(q_sd,qq,qqd,xx,xxd,t);
% Equation of motion
dL_dx=jacobian(L,q_s);
dL_dv=jacobian(L,q_sd);
dtdt_dLdv = time_derivative(dL_dv,qq,qqd,xx,xxd,t);
eom = (dtdt_dLdv - dL_dx)';

% M*q_sdd + h = 0
M = simplify(jacobian(eom,q_sdd));
h = simplify(subs(eom,q_sdd,q_sdd*0));


states = [q_s;q_sd];
%params = [mb,mk,Ib,Ik,L_k,L_h,g];

disp('done');


matlabFunction(M, h, 'File', ...
  ['generated_functions', filesep, 'func_Mh'], 'Vars', {states},...
   'Optimize',false);

function dfdt = time_derivative(f,qq,qqd,xx,xxd,t)
% Time derivative work around  
dfdt = subs( subs( diff( subs(f,qq,xx) ,t), xxd,qqd), xx,qq );
end


function Ry = rot2_y(theta)
Ry = [ cos(theta), sin(theta); ...
  -sin(theta), cos(theta)];
end



