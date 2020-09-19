

function xd = model_2d(x,u)
    [M,h] = func_Mh(x);
    acc = M\([u(1);u(2);0;u(3)] - h);
    xd = zeros(8,1);
    xd(1:4) = x(5:8);
    xd(5:8) = acc;
end