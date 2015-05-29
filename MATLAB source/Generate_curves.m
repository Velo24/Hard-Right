function Generate_curves(n, T2W_range, planet, filter, filenom)
%GENERATE_CURVES Summary of this function goes here
%   Detailed explanation goes here
if nargin < 3
    planet = 'kerbin';
end

if nargin < 4
    filter = 0;
end

if nargin < 5
    if strcmp(planet,'kerbin')
        mu = 3.5316e12;
        Rk = 600e3;
    else
        mu = 3.986e14;
        Rk = 6371e3;
    end
    
    if n > 1
        T2W   = linspace(T2W_range(1),T2W_range(2),n);
    else
        T2W = T2W_range(1);
    end
    
    dv    = zeros(n,1);
    ht    = zeros(n,1);
    
    for i = 1:n
        hturn = pso(@(hturn) E113_wrapper_func1(T2W(i), hturn, mu, Rk, 0, planet),0,5000,100, num2str(i));
        
        dv(i) = E113_wrapper_func1(T2W(i), hturn, mu, Rk, 0, planet);
        
        ht(i) = hturn;
    end
else
    load([filenom '.mat'])
    n = length(T2W); %#ok<NODEF>
end

if n > 1
    figure(1)
    hold on
    if filter
        plot(T2W,filter_signal(dv,filter),'bo')
    else
        plot(T2W,dv,'o')
    end
    xlabel('TWR')
    ylabel('dV (km/s)')
    
    figure(2)
    hold on
    if filter
        plot(T2W,filter_signal(ht,filter),'bo')
    else
        plot(T2W,ht,'o')
    end
    xlabel('TWR')
    ylabel('Pitchover Altitude (m)')
else
    figure(1)
    hold on
    plot(T2W,dv,'o')
    xlabel('TWR')
    ylabel('dV (km/s)')
    
    figure(2)
    hold on
    plot(T2W,ht,'o')
    xlabel('TWR')
    ylabel('Pitchover Altitude (m)')
end

save([planet 'results' num2str(n) '.mat'],'dv','ht','T2W')
end

