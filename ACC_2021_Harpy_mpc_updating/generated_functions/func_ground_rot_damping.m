function [tau_gz_L,tau_gz_R] = func_ground_rot_damping(in1,in2)
%FUNC_GROUND_ROT_DAMPING
%    [TAU_GZ_L,TAU_GZ_R] = FUNC_GROUND_ROT_DAMPING(IN1,IN2)

%    This function was generated by the Symbolic Math Toolbox version 8.4.
%    27-Jul-2020 06:26:28

Rb_3_1 = in1(20,:);
Rb_3_2 = in1(23,:);
Rb_3_3 = in1(26,:);
kb_ground = in2(31,:);
l4a = in2(29,:);
l4b = in2(30,:);
qD_hF_1 = in1(8,:);
qD_hF_2 = in1(10,:);
qD_hS_1 = in1(9,:);
qD_hS_2 = in1(11,:);
qD_kS_1 = in1(29,:);
qD_kS_2 = in1(30,:);
q_hF_1 = in1(1,:);
q_hF_2 = in1(3,:);
q_kS_1 = in1(27,:);
q_kS_2 = in1(28,:);
wb_1 = in1(15,:);
wb_2 = in1(16,:);
wb_3 = in1(17,:);
tau_gz_L = [-Rb_3_1.*kb_ground.*(Rb_3_2.*(wb_2+cos(q_hF_1).*(qD_hS_1+(qD_kS_1+(qD_kS_1.*1.0./cos(q_kS_1).^2.*sin(q_kS_1).*(l4b+l4a.*sin(q_kS_1)))./l4a)./(1.0./l4a.^2.*1.0./cos(q_kS_1).^2.*(l4b+l4a.*sin(q_kS_1)).^2+1.0)))+Rb_3_3.*(wb_3+sin(q_hF_1).*(qD_hS_1+(qD_kS_1+(qD_kS_1.*1.0./cos(q_kS_1).^2.*sin(q_kS_1).*(l4b+l4a.*sin(q_kS_1)))./l4a)./(1.0./l4a.^2.*1.0./cos(q_kS_1).^2.*(l4b+l4a.*sin(q_kS_1)).^2+1.0)))+Rb_3_1.*(qD_hF_1+wb_1));-kb_ground.*(Rb_3_2.*cos(q_hF_1)+Rb_3_3.*sin(q_hF_1)).*(Rb_3_2.*(wb_2+cos(q_hF_1).*(qD_hS_1+(qD_kS_1+(qD_kS_1.*1.0./cos(q_kS_1).^2.*sin(q_kS_1).*(l4b+l4a.*sin(q_kS_1)))./l4a)./(1.0./l4a.^2.*1.0./cos(q_kS_1).^2.*(l4b+l4a.*sin(q_kS_1)).^2+1.0)))+Rb_3_3.*(wb_3+sin(q_hF_1).*(qD_hS_1+(qD_kS_1+(qD_kS_1.*1.0./cos(q_kS_1).^2.*sin(q_kS_1).*(l4b+l4a.*sin(q_kS_1)))./l4a)./(1.0./l4a.^2.*1.0./cos(q_kS_1).^2.*(l4b+l4a.*sin(q_kS_1)).^2+1.0)))+Rb_3_1.*(qD_hF_1+wb_1));0.0;0.0;0.0;0.0;0.0;-Rb_3_1.*kb_ground.*(Rb_3_2.*(wb_2+cos(q_hF_1).*(qD_hS_1+(qD_kS_1+(qD_kS_1.*1.0./cos(q_kS_1).^2.*sin(q_kS_1).*(l4b+l4a.*sin(q_kS_1)))./l4a)./(1.0./l4a.^2.*1.0./cos(q_kS_1).^2.*(l4b+l4a.*sin(q_kS_1)).^2+1.0)))+Rb_3_3.*(wb_3+sin(q_hF_1).*(qD_hS_1+(qD_kS_1+(qD_kS_1.*1.0./cos(q_kS_1).^2.*sin(q_kS_1).*(l4b+l4a.*sin(q_kS_1)))./l4a)./(1.0./l4a.^2.*1.0./cos(q_kS_1).^2.*(l4b+l4a.*sin(q_kS_1)).^2+1.0)))+Rb_3_1.*(qD_hF_1+wb_1));-Rb_3_2.*kb_ground.*(Rb_3_2.*(wb_2+cos(q_hF_1).*(qD_hS_1+(qD_kS_1+(qD_kS_1.*1.0./cos(q_kS_1).^2.*sin(q_kS_1).*(l4b+l4a.*sin(q_kS_1)))./l4a)./(1.0./l4a.^2.*1.0./cos(q_kS_1).^2.*(l4b+l4a.*sin(q_kS_1)).^2+1.0)))+Rb_3_3.*(wb_3+sin(q_hF_1).*(qD_hS_1+(qD_kS_1+(qD_kS_1.*1.0./cos(q_kS_1).^2.*sin(q_kS_1).*(l4b+l4a.*sin(q_kS_1)))./l4a)./(1.0./l4a.^2.*1.0./cos(q_kS_1).^2.*(l4b+l4a.*sin(q_kS_1)).^2+1.0)))+Rb_3_1.*(qD_hF_1+wb_1));-Rb_3_3.*kb_ground.*(Rb_3_2.*(wb_2+cos(q_hF_1).*(qD_hS_1+(qD_kS_1+(qD_kS_1.*1.0./cos(q_kS_1).^2.*sin(q_kS_1).*(l4b+l4a.*sin(q_kS_1)))./l4a)./(1.0./l4a.^2.*1.0./cos(q_kS_1).^2.*(l4b+l4a.*sin(q_kS_1)).^2+1.0)))+Rb_3_3.*(wb_3+sin(q_hF_1).*(qD_hS_1+(qD_kS_1+(qD_kS_1.*1.0./cos(q_kS_1).^2.*sin(q_kS_1).*(l4b+l4a.*sin(q_kS_1)))./l4a)./(1.0./l4a.^2.*1.0./cos(q_kS_1).^2.*(l4b+l4a.*sin(q_kS_1)).^2+1.0)))+Rb_3_1.*(qD_hF_1+wb_1))];
if nargout > 1
    tau_gz_R = [0.0;0.0;-Rb_3_1.*kb_ground.*(Rb_3_2.*(wb_2+cos(q_hF_2).*(qD_hS_2+(qD_kS_2+(qD_kS_2.*1.0./cos(q_kS_2).^2.*sin(q_kS_2).*(l4b+l4a.*sin(q_kS_2)))./l4a)./(1.0./l4a.^2.*1.0./cos(q_kS_2).^2.*(l4b+l4a.*sin(q_kS_2)).^2+1.0)))+Rb_3_3.*(wb_3+sin(q_hF_2).*(qD_hS_2+(qD_kS_2+(qD_kS_2.*1.0./cos(q_kS_2).^2.*sin(q_kS_2).*(l4b+l4a.*sin(q_kS_2)))./l4a)./(1.0./l4a.^2.*1.0./cos(q_kS_2).^2.*(l4b+l4a.*sin(q_kS_2)).^2+1.0)))+Rb_3_1.*(qD_hF_2+wb_1));-kb_ground.*(Rb_3_2.*cos(q_hF_2)+Rb_3_3.*sin(q_hF_2)).*(Rb_3_2.*(wb_2+cos(q_hF_2).*(qD_hS_2+(qD_kS_2+(qD_kS_2.*1.0./cos(q_kS_2).^2.*sin(q_kS_2).*(l4b+l4a.*sin(q_kS_2)))./l4a)./(1.0./l4a.^2.*1.0./cos(q_kS_2).^2.*(l4b+l4a.*sin(q_kS_2)).^2+1.0)))+Rb_3_3.*(wb_3+sin(q_hF_2).*(qD_hS_2+(qD_kS_2+(qD_kS_2.*1.0./cos(q_kS_2).^2.*sin(q_kS_2).*(l4b+l4a.*sin(q_kS_2)))./l4a)./(1.0./l4a.^2.*1.0./cos(q_kS_2).^2.*(l4b+l4a.*sin(q_kS_2)).^2+1.0)))+Rb_3_1.*(qD_hF_2+wb_1));0.0;0.0;0.0;-Rb_3_1.*kb_ground.*(Rb_3_2.*(wb_2+cos(q_hF_2).*(qD_hS_2+(qD_kS_2+(qD_kS_2.*1.0./cos(q_kS_2).^2.*sin(q_kS_2).*(l4b+l4a.*sin(q_kS_2)))./l4a)./(1.0./l4a.^2.*1.0./cos(q_kS_2).^2.*(l4b+l4a.*sin(q_kS_2)).^2+1.0)))+Rb_3_3.*(wb_3+sin(q_hF_2).*(qD_hS_2+(qD_kS_2+(qD_kS_2.*1.0./cos(q_kS_2).^2.*sin(q_kS_2).*(l4b+l4a.*sin(q_kS_2)))./l4a)./(1.0./l4a.^2.*1.0./cos(q_kS_2).^2.*(l4b+l4a.*sin(q_kS_2)).^2+1.0)))+Rb_3_1.*(qD_hF_2+wb_1));-Rb_3_2.*kb_ground.*(Rb_3_2.*(wb_2+cos(q_hF_2).*(qD_hS_2+(qD_kS_2+(qD_kS_2.*1.0./cos(q_kS_2).^2.*sin(q_kS_2).*(l4b+l4a.*sin(q_kS_2)))./l4a)./(1.0./l4a.^2.*1.0./cos(q_kS_2).^2.*(l4b+l4a.*sin(q_kS_2)).^2+1.0)))+Rb_3_3.*(wb_3+sin(q_hF_2).*(qD_hS_2+(qD_kS_2+(qD_kS_2.*1.0./cos(q_kS_2).^2.*sin(q_kS_2).*(l4b+l4a.*sin(q_kS_2)))./l4a)./(1.0./l4a.^2.*1.0./cos(q_kS_2).^2.*(l4b+l4a.*sin(q_kS_2)).^2+1.0)))+Rb_3_1.*(qD_hF_2+wb_1));-Rb_3_3.*kb_ground.*(Rb_3_2.*(wb_2+cos(q_hF_2).*(qD_hS_2+(qD_kS_2+(qD_kS_2.*1.0./cos(q_kS_2).^2.*sin(q_kS_2).*(l4b+l4a.*sin(q_kS_2)))./l4a)./(1.0./l4a.^2.*1.0./cos(q_kS_2).^2.*(l4b+l4a.*sin(q_kS_2)).^2+1.0)))+Rb_3_3.*(wb_3+sin(q_hF_2).*(qD_hS_2+(qD_kS_2+(qD_kS_2.*1.0./cos(q_kS_2).^2.*sin(q_kS_2).*(l4b+l4a.*sin(q_kS_2)))./l4a)./(1.0./l4a.^2.*1.0./cos(q_kS_2).^2.*(l4b+l4a.*sin(q_kS_2)).^2+1.0)))+Rb_3_1.*(qD_hF_2+wb_1))];
end
