function vc = E113_wrapper_func1(T2W, hturn, mu, Rk, make_plot, planet)
%OPTIMIZE_APS Summary of this function goes here
%   Detailed explanation goes here

if nargin < 6
    make_plot = 0;
end

n = fminbnd(@(n) E113_wrapper_func2(T2W, hturn, n, mu, Rk, 0, planet),1.1,15);

if strcmp(planet,'kerbin')
    [vD, vG, v, h, gamma] = Example_11_3(T2W, hturn, n, make_plot);
    vc = v_cost(vD, vG, v, h, gamma, mu, Rk, 80);
else
    [vD, vG, v, h, gamma] = Example_11_3_Earth(T2W, hturn, n, make_plot);
    vc = v_cost(vD, vG, v, h, gamma, mu, Rk, 250);
end

end

