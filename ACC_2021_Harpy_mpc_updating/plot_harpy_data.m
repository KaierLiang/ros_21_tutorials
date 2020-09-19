
close all
addpath(['utility_functions',filesep])  
%Plotting Options
l_width = 1.5; % Solid line thickness
time_axis = [data.t(1),data.t(end)];
p_width = 500;
p_height = 500;

%set(fig3,'position',[500,100,p_width,400]);

%Smoothen the ground reaction forces
fg_L_smooth = [ smooth(data.ug(1,:),19)';...
                smooth(data.ug(2,:),19)';... 
                smooth(data.ug(3,:),19)'];
             
fg_R_smooth = [ smooth(data.ug(4,:),19)';...
                smooth(data.ug(5,:),19)';...
                smooth(data.ug(6,:),19)'];
total_forces = fg_L_smooth + fg_R_smooth;

% Body states -------------------------------------------------------------

fig1 = figure(1);
set(fig1,'position',[0,50,p_width,p_height]);

subplot(4,1,1)
plot( data.t, data.x(5,:),...
      data.t, data.xr(1,:),'k--',...
      'LineWidth',l_width)
title('Body CoM position x')
ylabel('Pos (m)')
axis([time_axis,-0.5,6])
legend('state','ref','Location','northwest',...
        'NumColumns',1,'FontSize',7)


subplot(4,1,2)
plot( data.t, data.x(6,:),...
      data.t, data.xr(2,:),'k--',...
      'LineWidth',l_width)
title('Body CoM position y')
ylabel('Pos (m)')
axis([time_axis,-0.2,0.2])


subplot(4,1,3)
plot( data.t, data.x(7,:),...
      data.t, data.xr(3,:),'k--',...
      'LineWidth',l_width)
title('Body CoM position z')
ylabel('Pos (m)')
axis([time_axis,0.5,1.2])

subplot(4,1,4)
plot(data.t, fg_R_smooth(3,:) + fg_L_smooth(3,:),'LineWidth',l_width)
hold on
title('Sum of normal forces')
xlabel('Time(s)'), ylabel('Force (N)')
axis([time_axis,0,150])


tightfig(fig1);
print(fig1,'body_states.eps','-depsc','-r600');

% Ground forces -----------------------------------------------------------
fig2 = figure(2);
set(fig2,'position',[0,50,p_width,p_height]);

% subplot(4,1,1)
% plot( data.t, data.x(12,:),...
%       data.t, data.x(13,:),...
%       data.t, data.x(14,:),...
%       'LineWidth',l_width)
% title('Body CoM velocity')
% legend('x','y','z','FontSize',7,'NumColumns',3)
% % legend('v_x','v_y','v_z','Location','northwest','NumColumns',1,...
% %        'FontSize',7,'NumColumns',3)
% ylabel('Vel (m/s)')
% axis([time_axis,-1,2])
% 
% 
% subplot(4,1,2)
% plot( data.t, data.x(6,:),...
%       data.t, data.xr(2,:),'k--',...
%       'LineWidth',l_width)
% title('Body CoM position y')
% ylabel('Pos (m)')
% axis([time_axis,-0.2,0.2])
% 
% 
% 
% subplot(4,1,3)
% plot( data.t, data.x(7,:),...
%       data.t, data.xr(3,:),'k--',...
%       'LineWidth',l_width)
% title('Body CoM position z')
% ylabel('Pos (m)')
% axis([time_axis,0.5,1])
% 
% 
% subplot(4,1,4)
% plot(data.t, data.euler(:,:)/pi*180,'LineWidth',l_width)
% title('Body orientation')
% ylabel('Angle (rad)')
% axis([time_axis,-80,40])
% 
% legend('roll','pitch','yaw','Location','northwest','NumColumns',1,...
%        'FontSize',7)



%Ground forces


fig2 = figure(2);
set(fig2,'position',[0,50,p_width,p_height]);
subplot(4,1,1)
plot( data.t, data.x(12,:),...
      data.t, data.x(13,:),...
      data.t, data.x(14,:),...
      'LineWidth',l_width)
title('Body CoM velocity')
legend('x','y','z','FontSize',7,'NumColumns',3)
% legend('v_x','v_y','v_z','Location','northwest','NumColumns',1,...
%        'FontSize',7,'NumColumns',3)
ylabel('Vel (m/s)')
axis([time_axis,-1,2])


subplot(4,1,2)
plot(data.t, data.euler(:,:)/pi*180,'LineWidth',l_width)
title('Body orientation')
legend('roll','pitch','yaw','Location','northwest','NumColumns',1,...
       'FontSize',7)
ylabel('Angle (rad)')

subplot(4,1,3)
plot(data.t, data.x(15:17,:),'LineWidth',l_width)
title('Body angular vel (body frame)')
ylabel('Ang vel (rad/s)')
legend('x','y','z','FontSize',7,'NumColumns',3)

axis([time_axis,-5,5])

subplot(4,1,4)
plot(data.t, data.v(1:3,:),'LineWidth',l_width)
title('Sum of thruster forces (CoM)')
ylabel('Force (N)'),xlabel('Time(s)')
axis([time_axis,-150,120])
legend('x','y','z','FontSize',7,'NumColumns',3)





axis([time_axis,-150,120])


hold off
%legend('force','Location','northwest','NumColumns',2)
tightfig(fig2);
print(fig2,'ground_thruster.eps','-deps','-r600');







