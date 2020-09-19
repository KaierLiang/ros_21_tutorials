function J = cost_function(X,U,e,data)
%COST_FUNCTION Summary of this function goes here
%   Detailed explanation goes here

% for i=1:p+1
%     Y(i,:) = func_output(X(i,:));
% end
% 
% p = data.PredictionHorizon;
%  ref = data.References;
%  X
% e1 = ref(1:p,1) - X(2:p+1,1);
% e2 = ref(1:p,2) - X(2:p+1,2);
% J = 50*sum(sum(e1'*e1)) + 50*sum(sum(e2'*e2)) + 0.5*sum(sum(U(1:p,:)));


p = data.PredictionHorizon;
 ref = data.References;

%  u1 = U(1:p,1);
%  u2 = U(1:p,2);
%  u3 = U(1:p,3);
e1 = ref(1:p,1) - X(2:p+1,1);
e2 = ref(1:p,2) - X(2:p+1,2);
%e3 = ref(1:p,3) - (X(2:p+1,3) + X(2:p+1,4)); 
 %e3 = ref(1:p,3) - (X(2:p+1,4) + X(1,3)); 
 e3 = ref(1:p,3) - X(2:p+1,4); 
e4 = diff(U);

% sum(e1'*e1) 
% sum(e2'*e2) 
% sum(u1'*u1)
% 1/1000*sum(u2'*u2)
% 100*sum(u3'*u3) 
%J = 20*sum(e1'*e1) + 20*sum(e2'*e2) + 20*sum(e3'*e3) + (1/5*sum(e4(:,1)'*e4(:,1)) + 1/10*sum(e4(:,2)'*e4(:,2)) +  1/10*sum(e4(:,3)'*e4(:,3)));
% J = 20*sum(e1'*e1) + 20*sum(e2'*e2) + 20*sum(e3'*e3) + (1/5*sum(e4(:,1)'*e4(:,1)) + 1/10*sum(e4(:,2)'*e4(:,2)));


%J = 20*sum(e1'*e1) + 20*sum(e2'*e2) + 5*sum(e3'*e3) + ( 1/5*sum(e4(:,1)'*e4(:,1)) + 1/5*sum(e4(:,2)'*e4(:,2)) + 1/5*sum(e4(:,3)'*e4(:,3)));
J = 5*sum(e1'*e1) + 20*sum(e2'*e2) + 5*sum(e3'*e3) + (1/10*sum(e4(:,1)'*e4(:,1)) + 1/10*sum(e4(:,2)'*e4(:,2))) + 1/5*sum(e4(:,3)'*e4(:,3));
% if J < 0.010
%     J = 0;
% end
end

