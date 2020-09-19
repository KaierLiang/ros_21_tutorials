function ref = func_ref(t,p_h,pitch,v)
% % Entire reference
% % ref_x = linspace(0,1,200);
% % ref_y = -1*(ref_x - 0.5).^2 + 0.5^2*1;
% % ref_t = [ref_x',ref_y'];
% % % Update reference
% % ref = ref_t(t:t+15-1,:);
% 
% ref_x = linspace(0,1.35,150);
% ref_y = -0.5*(ref_x - 1.35/2).^2 + (1.35/2)^2*0.5;
% ref_t = [ref_x',ref_y'];
% % Update reference
% ref = ref_t(t:t+15-1,:);
% %ref = ref_t;

% ref_x = linspace(0,1.35,150);
% ref_y = -0.5*(ref_x - 1.35/2).^2 + (1.35/2)^2*0.5;
%ref_theta = ones(1,150)*(-0.40 + pitch);


% pitch = -0.69;
% %ref_theta = ones(1,150)*(-0.4027 + asin(0.1*sin(pitch)/0.5) + pitch);
% 
% ref_theta(1:75) = -0.4;
% theta_des = (-0.4027 + asin(0.1*sin(pitch)/0.5) + pitch);
% ref_theta(76:150) = linspace(-0.4,theta_des,75);


%ref_theta = ones(1,150)*(-0.4027 + asin(0.1*sin(pitch)/0.5) + pitch);
pitch_t = pitch + asin(0.1*sin(pitch)/0.5);
if (pitch_t) >= 20*pi/180
    pitch_ref = 20*pi/180;
elseif pitch_t < -60*pi/180
    pitch_ref = -60*pi/180;
else
    pitch_ref = pitch_t;
end

%pitch_ref = 0;
ref_theta(1:150) = pitch_ref - 0.4027;
%theta_des = (-0.4027 + asin(0.1*sin(pitch)/0.5) + pitch);
%ref_theta(100:150) = linspace(-0.4,-1,30);
% ref_theta = zeros(1,150);
%ref_theta = zeros(1,150)*(-0.40);
ref_x = zeros(1,150);
ref_y = zeros(1,150);
% Update reference

    
for i = 1:1.5/0.01
    t1 = i/100;
    if i == 1
        ref_x(1) = 0.3*3*0.01;
        ref_y(1) = (v/10)*sin(2*pi*(t1)/1.5)*0.01;
    else  
    ref_x(i) = ref_x(i-1) + 0.3*3*0.01;
    ref_y(i) = ref_y(i-1) + (v/10)*sin(2*pi*(t1)/1.5)*0.01;
    end
end
ref_t = [ref_x',ref_y',ref_theta'];


if (t + p_h) < 150
    ref = ref_t(t:t + p_h -1,:);
else
    ref = ref_t(t:end,:);
    temp = size(ref,1);
    ref(temp:p_h,:) = repmat(ref_t(end,:),p_h - temp +1,1);
end
end

