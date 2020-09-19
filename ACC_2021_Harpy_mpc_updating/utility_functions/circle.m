function x = circle(x0,r,N)
  ang_max = 4*pi; 
  ang = 0:(ang_max)/(N-1):ang_max;
  x = zeros(2,N);
  x(1,:) = r*cos(ang) + x0(1);
  x(2,:) = r*sin(ang) + x0(2);
end
