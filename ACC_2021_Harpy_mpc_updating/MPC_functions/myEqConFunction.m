function ceq = myEqConFunction(X,U,data)
%MYEQCONFUNCTION Summary of this function goes here
%   Detailed explanation goes here
%ceq = [x(:,3) + x(:,4) - 30*pi/180;x(end,1) - 2;x(end,2) - 2];
% x(end,1) - 0.8
% x(end,2) - 0.8
%ceq = [X(end,1) - 0.8;X(end,2) - 0.8];
%ceq = [X(end,1) - 0.5;X(end,2) - 0.5];
% ceq = X(2:end,1) - data.References(:,1);
% ceq = [ceq;X(2:end,2) - data.References(:,2)];
%  ceq = [X(end,1) - 5; X(end,2)];
%  ceq = [ceq; 
%      X(12,1) - 2.5; 
%      X(12,2) - 3.125];
   
ceq = [X(2:end,3) - X(2:end,4)]; %- 30*pi/180];
%        X(13,3) - X(13:4)];
%        %X(14:end,3) - X(14:end,4) + 30*pi/180];
% for i = 14:21
%     if X(14:end,3) <0 
%         ceq = [ceq;X(i,4)];
%     else
%         ceq = [ceq;X(i,3) - X(i,4) + 30*pi/180];
%     end
% end
end
