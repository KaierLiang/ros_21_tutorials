
%load MPC_result4.mat;
pa.lh_FR = [  0.1;  -0.1;    -0.1];
pa.lh_FL = [  0.1;   0.1;    -0.1];
pa.lh_HR = [ -0.1;  -0.1;    -0.1];
pa.lh_HL = [ -0.1;   0.1;    -0.1];

anim.body = zeros(3,22);
anim.pos_leg = zeros(3,22);
anim.pos_hip = zeros(3,22);
for i = 1:p
    state = info.Xopt(i+1,:);
    pos_body = state(1:2);
    pos_leg = func_leg(state);
    R_body = roty(info.Xopt(i,3)*180/pi);
    anim.pos_body(:,i) = [pos_body(1);0;pos_body(2)];
    anim.pos_leg(:,i) = pos_leg;
    p1 = [pos_body(1);0;pos_body(2)] + R_body*pa.lh_FR;
    p2 = [pos_body(1);0;pos_body(2)] + R_body*pa.lh_FL;
    p3 = [pos_body(1);0;pos_body(2)] + R_body*pa.lh_HL;
    p4 = [pos_body(1);0;pos_body(2)] + R_body*pa.lh_HR;
    p5 = [pos_body(1);0;pos_body(2)] + R_body*(pa.lh_FR - [0;0;pa.lh_FR(3)*2]);
    p6 = [pos_body(1);0;pos_body(2)] + R_body*(pa.lh_FL - [0;0;pa.lh_FL(3)*2]);
    p7 = [pos_body(1);0;pos_body(2)] + R_body*(pa.lh_HL - [0;0;pa.lh_HR(3)*2]);
    p8 = [pos_body(1);0;pos_body(2)] + R_body*(pa.lh_HR - [0;0;pa.lh_HL(3)*2]);  
    temp = [p1,p2,p3,p4,p5,p6,p7,p8]';
    anim.pos_box(:,i) = reshape(temp, [24,1]);
    anim.pos_hip(:,i) = anim.pos_body(:,i) + R_body*[0;0;-0.1];
end

fig10 = figure(10);
set(fig10,'Position',[100,100,600,600])
axis_range = [-.5,1.5, -.5,.5, -0.5,2];
% ref_x = linspace(0,5,21);
% ref_y = -1/2*(ref_x - 0.5).^2 + 0.5^2/2;


% plot(ref_x,ref_y);
% hold on
% plot(X(:,1),X(:,2));


for i = 1:p
    clf
    draw_sphere(anim.pos_body(:,i),0.03, [0,0,1], 1);
    hold on
    draw_box(anim.pos_box(:,i));
    draw_sphere(anim.pos_leg(:,i),0.02,[0,0,1],1);
    draw_sphere(anim.pos_hip(:,i),0.02,[0,0,1],1);
   draw_line(anim.pos_hip(:,i),anim.pos_leg(:,i),[0,0,0], 1);
    
    plot3(ref_x,zeros(p,1),ref_y);
    axis(axis_range);
    daspect([1 1 1]);
   title(['Time = ', num2str(Ts*i), ' s'])
    xlabel('x (m)')
    ylabel('y (m)')
    zlabel('z (m)')
% 
     view(45,20)
%     view(90,90) % top
%     view(90,0) % front
    view(0,0) % side
    
    pause(1e-6)
end

%% Local functions
function pos_leg = func_leg(X)
    pos_leg = [X(1);0;X(2)] + roty(X(3)*180/pi)*[0;0;-0.1] + roty(X(3)*180/pi)*roty(X(4)*180/pi)*[0;0;-0.3];
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

function draw_line(pos1,pos2, color, alpha)
  % Draw straight line
  px = [pos1(1), pos2(1)];
  py = [pos1(2), pos2(2)];
  pz = [pos1(3), pos2(3)];
  plot3(px,py,pz,'LineWidth', 3, 'Color', [color, alpha]);
end

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