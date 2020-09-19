%load MPC_result4.mat
fig1 = figure(1);

ref_x = ref(1:p,1);
ref_y = ref(1:p,2);
set(fig1,'position',[0,50,500,600]);
for i = 1:p
    t(i) = Ts*i;
end
subplot(3,2,1)
plot(t,info.Xopt(2:end,1),...
    t,ref_x,'k--');
title('Body CoM C_x trajectory')
legend('state','target ref');
grid on

subplot(3,2,2)
plot(t,info.Xopt(2:end,2),...
     t, ref_y,'k--')
title('Body CoM C_y trajectory')
%legend('state','target ref');
grid on

subplot(3,2,3)
plot(t,info.Xopt(2:end,3) - info.Xopt(2:end,4))
yline(20*pi/180,'--r');
yline(-20*pi/180,'--b');
% yline(-30*pi/180,'--b');
% yline(-50*pi/180,'--r');
title('theta - q')
legend({'theta - q'})%,' lower bound','upper bound'})
grid on

subplot(3,2,4)
plot(t,info.MVopt(1:p,1),...
     t,info.MVopt(1:p,2),...
     t,info.MVopt(1:p,3))
title('Thruster input')
legend('Horizonal force','Vertical force','Torque')
grid on
 
subplot(3,2,5)
plot(t,info.Xopt(2:end,5),'r',...
    t,info.Xopt(2:end,6),'b')
title('Body velocity')
legend('V_x','V_y')
grid on

subplot(3,2,6)
plot(t,info.Xopt(2:end,3), ...
     t,info.Xopt(2:end,4))
yline(30*pi/180,'--b');
yline(20*pi/180,'--b');
yline(10*pi/180,'--b');

legend('pitch','q')
grid on
