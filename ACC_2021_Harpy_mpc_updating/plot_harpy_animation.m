% Animate
addpath(['utility_functions',filesep]) 
if (ishandle(10))
  close(10);
end

%% animation vertex

% Box vertex
p.lh_FR = [  0.1;  -0.1;    -0.1];
p.lh_FL = [  0.1;   0.1;    -0.1];
p.lh_HR = [ -0.1;  -0.1;    -0.1];
p.lh_HL = [ -0.1;   0.1;    -0.1];

anim.pos_body = zeros(3,N);   
anim.pos_box = zeros(24,N); 
anim.pos_hip = zeros(6,N);  
anim.pos_knee = zeros(6,N);  
anim.pos_lowerleg = zeros(6,N);  
anim.pos_foot = zeros(6,N);   

% parallell leg vertex
anim.pos_parallel1 = zeros(6,N); 
anim.pos_parallel2 = zeros(6,N);
anim.pos_th = zeros(6,N);

N = length(data.t);
for i = 1:N
    %gather relevant states
    x = data.x(:,i);
    x_body = x(5:7);
    R_body = reshape(x(18:26),[3,3]);
    q_hipF = [x(1); x(3)];
    q_hipS = [x(2); x(4)];
    q_kneeS = x(27:28);
    
    %animated positions
    pos_body = x_body; %body position
    pos_pelvis_L = x_body + R_body*p.L1_L;
    pos_pelvis_R = x_body + R_body*p.L1_R;
    
    pos_hip_L = pos_pelvis_L + R_body*rot_x(q_hipF(1))*p.L2_L;
    pos_hip_R = pos_pelvis_R + R_body*rot_x(q_hipF(2))*p.L2_R;
    
    pos_knee_L = pos_hip_L + R_body*rot_x(q_hipF(1))*rot_y(q_hipS(1))*p.L3_L;
    pos_knee_R = pos_hip_R + R_body*rot_x(q_hipF(2))*rot_y(q_hipS(2))*p.L3_R;

    
    L4_L1 = [-p.l4a*cos(q_kneeS(1)) ; 0 ; -(p.l4b + p.l4a*sin(q_kneeS(1)))];
    L4_R1 = [-p.l4a*cos(q_kneeS(2)) ; 0 ; -(p.l4b + p.l4a*sin(q_kneeS(2)))];
    
    L4_L2 = [-p.l4a*cos(q_kneeS(1)) ; 0 ; -p.l4a*sin(q_kneeS(1))];
    L4_R2 = [-p.l4a*cos(q_kneeS(2)) ; 0 ; -p.l4a*sin(q_kneeS(2))];
    
    
    pos_foot_L = pos_knee_L + R_body*rot_x(q_hipF(1))*rot_y(q_hipS(1))*L4_L1;
    pos_foot_R = pos_knee_R + R_body*rot_x(q_hipF(2))*rot_y(q_hipS(2))*L4_R1;
    
    pos_lowerleg_L = pos_knee_L + R_body*rot_x(q_hipF(1))*rot_y(q_hipS(1))*L4_L2;
    pos_lowerleg_R = pos_knee_R + R_body*rot_x(q_hipF(2))*rot_y(q_hipS(2))*L4_R2;
    
%     pos_lowerleg_L = pos_knee_L + R_body*rot_x(q_hipF(1))*rot_y(q_hipS(1))*rot_y(q_kneeS(1))*p.L4_L;
%     pos_lowerleg_R = pos_knee_R + R_body*rot_x(q_hipF(2))*rot_y(q_hipS(2))*rot_y(q_kneeS(2))*p.L4_R;
        
%     pos_foot_L = pos_lowerleg_L + R_body*rot_x(q_hipF(1))*rot_y(q_hipS(1))*p.L5_L;
%     pos_foot_R = pos_lowerleg_R + R_body*rot_x(q_hipF(2))*rot_y(q_hipS(2))*p.L5_R;
    
    pos_parallel1_L = pos_hip_L + R_body*rot_x(q_hipF(1))*rot_y(q_hipS(1))*(p.L3_L + p.L6_L);
    pos_parallel1_R = pos_hip_R + R_body*rot_x(q_hipF(2))*rot_y(q_hipS(2))*(p.L3_R + p.L6_R);
    
    pos_parallel2_L = pos_lowerleg_L + R_body*rot_x(q_hipF(1))*rot_y(q_hipS(1))*p.L6_L;
    pos_parallel2_R = pos_lowerleg_R + R_body*rot_x(q_hipF(2))*rot_y(q_hipS(2))*p.L6_R;
    
    anim.pos_pelvis(:,i) = [pos_pelvis_L; pos_pelvis_R];
    anim.pos_hip(:,i) = [pos_hip_L; pos_hip_R];
    anim.pos_knee(:,i) = [pos_knee_L; pos_knee_R];
    anim.pos_lowerleg(:,i) = [pos_lowerleg_L; pos_lowerleg_R];
    anim.pos_foot(:,i) = [pos_foot_L; pos_foot_R];
    
    
    anim.pos_parallel1(:,i) = [pos_parallel1_L; pos_parallel1_R];
    anim.pos_parallel2(:,i) = [pos_parallel2_L; pos_parallel2_R];
  % body shape 
  %     7-------- 6
  %     /\       /\
  %    8--\-----5  \
  %     \  4--------2
  %      \ /       \/
  %       3--------1
    anim.pos_body(:,i) = pos_body;  
    p1 = pos_body + R_body*p.lh_FR;
    p2 = pos_body + R_body*p.lh_FL;
    p3 = pos_body + R_body*p.lh_HL;
    p4 = pos_body + R_body*p.lh_HR;
    p5 = pos_body + R_body*(p.lh_FR - [0;0;p.lh_FR(3)*2]);
    p6 = pos_body + R_body*(p.lh_FL - [0;0;p.lh_FL(3)*2]);
    p7 = pos_body + R_body*(p.lh_HL - [0;0;p.lh_HR(3)*2]);
    p8 = pos_body + R_body*(p.lh_HR - [0;0;p.lh_HL(3)*2]);
    temp = [p1,p2,p3,p4,p5,p6,p7,p8]';
    anim.pos_box(:,i) = reshape(temp, [24,1]);
    
    % Thruster positions
    pos_th_L = pos_body + R_body*p.Lt_L;
    pos_th_R = pos_body + R_body*p.Lt_R;
    anim.pos_th(:,i) = [pos_th_L;pos_th_R];
    
end

%% Animate

fig10 = figure(10);
set(fig10,'Position',[100,100,800,250])
axis_range = [-0.5,6, -.5,.5, -0.2,1.4];

% Animate plot, choose the frame skip values here


frame_skip = 500;
alpha = 1;
for i = 1500*1:frame_skip:data.i_end
    clf
%%
%     j = [500,1500*3,1500*5,1500*6,1500*9 + 500,1500*12,1500*13,1500*16];
%     len_k = length(j);
%     trans = linspace(0.1,1,len_k);
% for k = 1:len_k
%     i = j(k);
%     alpha = trans(k);
%%
    % draw body (sphere and box)
    draw_sphere(anim.pos_body(:,i),0.03, [0,0,1], alpha);
    hold on
    draw_box(anim.pos_box(:,i));
    
    % draw cylinders for four actuators
    % 'p', 'h', 'k' for pelvis hip and knee, 'l', 'r' for left and right
    draw_cylinder(anim.pos_pelvis(:,i), data.x(:,i), 0.02, [0,0,1], alpha, 'p', 'l'); 
    draw_cylinder(anim.pos_pelvis(:,i), data.x(:,i), 0.02, [0,0,1], alpha, 'p', 'r');
    
    draw_cylinder(anim.pos_hip(:,i), data.x(:,i), 0.02, [0,0,1], alpha, 'h', 'l');
    draw_cylinder(anim.pos_hip(:,i), data.x(:,i), 0.02, [0,0,1], alpha, 'h', 'r');
    
    draw_cylinder(anim.pos_knee(:,i), data.x(:,i), 0.02, [0,0,1], alpha, 'k', 'l');
    draw_cylinder(anim.pos_knee(:,i), data.x(:,i), 0.02, [0,0,1], alpha, 'k', 'r');
    P1 = [1.3,0,0.18] ;   % you center point 
    L1 = [0.6,1,0.36] ;
    O1 = P1-L1/2;
    plotcube(L1,O1,.8,[1 0 0]);
    P2 = [1,0,-0.05];
    L2 = [4,1,0.1];
    O2 = P2-L2/2;
    plotcube(L2,O2,.8,[210,105,30]/255);
    
    P3 = [5,0,-0.05];
    L3 = [2,1,0.1];
    O3 = P3-L3/2;
    plotcube(L3,O3,.8,[210,105,30]/255);
    
    
    
    % draw legs
    draw_legs(anim.pos_pelvis(:,i), anim.pos_hip(:,i), anim.pos_knee(:,i), anim.pos_lowerleg(:,i),...
                anim.pos_parallel1(:,i), anim.pos_parallel2(:,i), anim.pos_foot(:,i),alpha);
    
    % Draw ground contact forces
    mArrow3(anim.pos_foot(1:3,i) - data.ug(1:3,i)*0.05, anim.pos_foot(1:3,i), ...
            'color','red','stemWidth',0.01,'facealpha',0.5);
    mArrow3(anim.pos_foot(4:6,i) - data.ug(4:6,i)*0.05, anim.pos_foot(4:6,i), ...
            'color','red','stemWidth',0.01,'facealpha',0.5);
    
    % Draw thruster forces
    mArrow3(anim.pos_th(1:3,i), anim.pos_th(1:3,i) + data.f_th(1:3,i)*0.05, ...
            'color','blue','stemWidth',0.01,'facealpha',0.5);
    mArrow3(anim.pos_th(4:6,i), anim.pos_th(4:6,i) + data.f_th(4:6,i)*0.05,  ...
            'color','blue','stemWidth',0.01,'facealpha',0.5);
    
    %
    %
              
              
              
    axis(axis_range);
    daspect([1 1 1]);
 %   title(['Time = ', num2str((i-1)*p.dt), ' s'])
    xlabel('x (m)')
    ylabel('y (m)')
    zlabel('z (m)')
% 
    view(45,20)
%     view(90,90) % top
     view(90,0) % front
 
view(0,0) % side
    
    pause(1e-15)
    
end
tightfig(fig10)


%clf
% for i = 1500*1:frame_skip:data.i_end
%     clf
%%
    j = [500,1500*3,1500*5,1500*6,1500*9 + 500,1500*12,1500*13,24000];
    len_k = length(j);
    trans = linspace(0.15,1,len_k);
    trans = [0.32    0.2714    0.38    0.42    0.7    0.55    0.60    1.0000];
for k = 1:len_k
    i = j(k);
    alpha = trans(k);
%%
    % draw body (sphere and box)
    draw_sphere(anim.pos_body(:,i),0.03, [0,0,1], alpha);
    hold on
    draw_box(anim.pos_box(:,i));
    
    % draw cylinders for four actuators
    % 'p', 'h', 'k' for pelvis hip and knee, 'l', 'r' for left and right
    draw_cylinder(anim.pos_pelvis(:,i), data.x(:,i), 0.02, [0,0,1], alpha, 'p', 'l'); 
    draw_cylinder(anim.pos_pelvis(:,i), data.x(:,i), 0.02, [0,0,1], alpha, 'p', 'r');
    
    draw_cylinder(anim.pos_hip(:,i), data.x(:,i), 0.02, [0,0,1], alpha, 'h', 'l');
    draw_cylinder(anim.pos_hip(:,i), data.x(:,i), 0.02, [0,0,1], alpha, 'h', 'r');
    
    draw_cylinder(anim.pos_knee(:,i), data.x(:,i), 0.02, [0,0,1], alpha, 'k', 'l');
    draw_cylinder(anim.pos_knee(:,i), data.x(:,i), 0.02, [0,0,1], alpha, 'k', 'r');
    P1 = [1.3,0,0.18] ;   % you center point 
    L1 = [0.6,1,0.36] ;
    O1 = P1-L1/2;
    plotcube(L1,O1,.8,[1 0 0]);
    P2 = [1,0,-0.1];
    L2 = [4,1,0.2];
    O2 = P2-L2/2;
    plotcube(L2,O2,.3,[210,105,30]/255);
    
    P3 = [5,0,-0.1];
    L3 = [2,1,0.2];
    O3 = P3-L3/2;
    plotcube(L3,O3,.3,[210,105,30]/255);
    
    
    
    % draw legs
    draw_legs(anim.pos_pelvis(:,i), anim.pos_hip(:,i), anim.pos_knee(:,i), anim.pos_lowerleg(:,i),...
                anim.pos_parallel1(:,i), anim.pos_parallel2(:,i), anim.pos_foot(:,i),alpha);
    
    % Draw ground contact forces
    mArrow3(anim.pos_foot(1:3,i) - data.ug(1:3,i)*0.02, anim.pos_foot(1:3,i), ...
            'color','red','stemWidth',0.01,'facealpha',0.5);
    mArrow3(anim.pos_foot(4:6,i) - data.ug(4:6,i)*0.02, anim.pos_foot(4:6,i), ...
            'color','red','stemWidth',0.01,'facealpha',0.5);
    
    % Draw thruster forces
    mArrow3(anim.pos_th(1:3,i), anim.pos_th(1:3,i) + data.f_th(1:3,i)*0.02, ...
            'color','blue','stemWidth',0.01,'facealpha',0.5);
    mArrow3(anim.pos_th(4:6,i), anim.pos_th(4:6,i) + data.f_th(4:6,i)*0.02,  ...
            'color','blue','stemWidth',0.01,'facealpha',0.5);
    
    %
    %
              
              
              
    axis(axis_range);
    daspect([1 1 1]);
 %   title(['Time = ', num2str((i-1)*p.dt), ' s'])
    xlabel('x (m)')
    ylabel('y (m)')
    zlabel('z (m)')
% 
    view(45,20)
%     view(90,90) % top
     view(90,0) % front
 
view(0,0) % side
    
    pause(1e-15)
 grid off   
end
tightfig(fig10)
%% Local functions for drawing

function draw_sphere(pos, radius, color, alpha)
  % Draw a sphere
  [x,y,z] = sphere;
  hSurface = surf(radius*x + pos(1), ...
                  radius*y + pos(2), ...
                  radius*z + pos(3));  
                
  % Blue circle, no edge
  set(hSurface,'FaceColor', color, ...
      'FaceAlpha',alpha,'FaceLighting','gouraud','EdgeColor','none');
end

function draw_box(pos)
  % Draw a box
  px = pos(1:8,1);
  py = pos(9:16,1);
  pz = pos(17:24,1);
  
  % Draw all 6 faces
  face = cell(6,1);
  face{1} = [1,5,6,2]; % Front side
  face{2} = [2,6,7,3];
  face{3} = [1,2,3,4];
  face{4} = [5,6,7,8];
  face{5} = [4,8,7,3];
  face{6} = [1,5,8,4];   
  
  % front is red
  fill3(px(face{1}),py(face{1}),pz(face{1}),'r', 'FaceAlpha',0.5) 
  
  % the rest is white
  for k = 2:6
    fill3(px(face{k}),py(face{k}),pz(face{k}),'w','FaceAlpha',0.5)  
  end 
end

function draw_legs(pos_pelvis, pos_hip, pos_knee, pos_lowerleg, pos_parallel1, pos_parallel2, pos_foot,alpha)
  % Draw pelvis, hip, foot and foot positions
  
  % Draw hip positions (black)
  
  draw_sphere(pos_lowerleg(1:3), 0.015, [0,0,0], alpha);
  draw_sphere(pos_lowerleg(4:6), 0.015, [0,0,0], alpha);
  
  draw_sphere(pos_parallel1(1:3), 0.015, [0,0,0], alpha);
  draw_sphere(pos_parallel1(4:6), 0.015, [0,0,0], alpha);
  
  draw_sphere(pos_parallel2(1:3), 0.015, [0,0,0], alpha);
  draw_sphere(pos_parallel2(4:6), 0.015, [0,0,0], alpha);
  
  % Draw foot positions and lines (black)
  draw_sphere(pos_foot(1:3), 0.015, [0,0,0], alpha);
  draw_sphere(pos_foot(4:6), 0.015, [0,0,0], alpha);
  
  draw_line(pos_pelvis(1:3), pos_hip(1:3), [0,0,0], alpha);
  draw_line(pos_pelvis(4:6), pos_hip(4:6), [0,0,0], alpha);  
  
  draw_line(pos_hip(1:3), pos_knee(1:3), [0,0,0], alpha);
  draw_line(pos_hip(4:6), pos_knee(4:6), [0,0,0], alpha);    
  
  draw_line(pos_knee(1:3), pos_lowerleg(1:3), [0,0,0], alpha);
  draw_line(pos_knee(4:6), pos_lowerleg(4:6), [0,0,0], alpha);  
  
  draw_line(pos_lowerleg(1:3), pos_foot(1:3), [0,0,0], alpha);
  draw_line(pos_lowerleg(4:6), pos_foot(4:6), [0,0,0], alpha);  
  
  draw_line(pos_lowerleg(1:3), pos_parallel2(1:3), [0,0,0], alpha);
  draw_line(pos_lowerleg(4:6), pos_parallel2(4:6), [0,0,0], alpha);
  
  draw_line(pos_parallel1(1:3), pos_parallel2(1:3), [0,0,0], alpha);
  draw_line(pos_parallel1(4:6), pos_parallel2(4:6), [0,0,0], alpha);
end

function draw_line(pos1,pos2, color, alpha,line_width)
  % Draw straight line
  px = [pos1(1), pos2(1)];
  py = [pos1(2), pos2(2)];
  pz = [pos1(3), pos2(3)];
  if nargin < 5
    plot3(px,py,pz,'LineWidth', 3, 'Color', [color, alpha]);
  else
    plot3(px,py,pz,'LineWidth', line_width, 'Color', [color, alpha]);
  end
end

function draw_cylinder(pos, x, radius, color, alpha, position, side)
        R_body = reshape(x(18:26),[3,3]);
        q_hipF = [x(1); x(3)];
        if position == 'p'
            [z,y,x] = cylinder; 
            x = radius*(x - 0.5); %shift to the middle
            z = radius*z ;
            y = radius*y;
            x = x(:)';
            y = y(:)';
            z = z(:)';
            %Capital X Y Z for cylinder points after rotation
            X = zeros(1,length(x)); Y = X; Z = Y;
            for i =1:length(x)
                % cylinder vector after rotation
                if side == 'l'
                    vector = R_body*[x(i);y(i);z(i)] + pos(1:3);
                elseif side == 'r'
                    vector = R_body*[x(i);y(i);z(i)] + pos(4:6);
                end
                X(i) = vector(1);
                Y(i) = vector(2);
                Z(i) = vector(3);
            end
       
        elseif position == 'h'
            [x,z,y] = cylinder; 
            x = radius*x;
            y = radius*(y - 0.5); %shift to the middle
            z = radius*z;
            x = x(:)';
            y = y(:)';
            z = z(:)';
            %Capital X Y Z for cylinder points after rotation
            X = zeros(1,length(x)); Y = X; Z = Y;
            for i =1:length(x)
                % cylinder vector after rotation
                if side == 'l'
                    vector = pos(1:3) + R_body*rot_x(q_hipF(1))*[x(i);y(i);z(i)];
                elseif side == 'r'
                    vector = pos(4:6) + R_body*rot_x(q_hipF(2))*[x(i);y(i);z(i)];
                end
                X(i) = vector(1);
                Y(i) = vector(2);
                Z(i) = vector(3);
            end    
            
        elseif position == 'k'
            [x,z,y] = cylinder; 
            x = radius*x;
            y = radius*(y - 0.5); %shift to the middle
            z = radius*z;

            x = x(:)';
            y = y(:)';
            z = z(:)';
            %Capital X Y Z for cylinder points after rotation
            X = zeros(1,length(x)); Y = X; Z = Y;
            for i =1:length(x)
                % cylinder vector after rotation
                if side == 'l'
                    vector = pos(1:3) + R_body*rot_x(q_hipF(1))*[x(i);y(i);z(i)];
                elseif side == 'r'
                    vector = pos(4:6) + R_body*rot_x(q_hipF(2))*[x(i);y(i);z(i)];
                end
                X(i) = vector(1);
                Y(i) = vector(2);
                Z(i) = vector(3);
            end    
        end     

        X = reshape(X,[2,21]);
        Y = reshape(Y,[2,21]);
        Z = reshape(Z,[2,21]);
        hSurface = surf(X,Y,Z);   
      % fill the bases of cylinder (red)
      hold on
      fill3(X(1,:),Y(1,:),Z(1,:),'r', 'FaceAlpha',alpha);
      fill3(X(2,:),Y(2,:),Z(2,:),'r', 'FaceAlpha',alpha)     
 
  % Blue cylinder side
  set(hSurface,'FaceColor', color, ...
      'FaceAlpha',alpha,'FaceLighting','gouraud','EdgeColor','none');
end















