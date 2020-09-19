function cineq = myinEqConFunction(X,~,~,~)
%MYINEQ Summary of this function goes here
%   Detailed explanation goes here
% cineq =  [abs(X(:,3))  + abs(X(:,4)) - 0.5];
cineq = [abs(X(2:end,3)) - 50*pi/180];

% cineq = [-X(1:12,3) %+ 30*pi/180;
%         X(1:12,3) - 30*pi/180;% before apex -60 < theta + q < 0 
%          %X(13:end,3);% + 30*pi/180;
%          X(13:end,3) - 30*pi/180]; % after apex 0 < theta + q <60

% cineq = [X(2:12,3) - 20*pi/30;
%          X(2:12,3) + 30*pi/180];
% cineq = [X(1:12,3) + X(1:12,4) ;%+ 30*pi/180;
%         -(X(1:12,3) + X(1:12,4)) - 30*pi/180;% before apex -60 < theta + q < 0 
%          -(X(13:end,3) + X(13:end,4));% + 30*pi/180;
%         (X(13:end,3) + X(13:end,4)) - 30*pi/180]; % after apex 0 < theta + q <60
% %cineq = [cineq;abs(X(:,3)) - 30*pi/180];


% cineq = [X(12:end,3)  + 30*pi/180;
%          -X(12:end,3)  - 60*pi/180;% before apex -60 < theta + q < - 30 
%          -X(13:end,3)  + 30*pi/180;
%         X(13:end,3)  - 60*pi/180]; % after apex 30 < theta + q <60
%cineq = [cineq;abs(X(:,3)) - 30*pi/180];
%cineq = [X(:,3) + X(:,4)]
% ceq = [X(2:12,3) - X(2:12,4) + 20*pi/180;
%        X(13,3) - X(13:4);
%        X(14:end,3) - X(14:end,4) - 20*pi/180];
%    
% ceq = [X(2:12,3) - X(2:12,4) + 20*pi/180;
%        X(13,3) - X(13:4);
%        X(14:end,3) - X(14:end,4) - 20*pi/180];
cineq = [abs(X(2:end,3)) - pi/6];

end

