% ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
function [vD, vG, v, h, gamma] = Example_11_3(T2W, hturn, n, make_plot)
% ~~~~~~~~~~~~~~~~~~~~~~
%{
 This program numerically integrates Equations 11.6 through
 11.8 for a gravity turn trajectory.
 User M-functions required: rkf45
 User subfunction requred: rates
%}
% ----------------------------------------------

if nargin < 4
    make_plot = 0;
end

deg    = pi/180;            % ...Convert degrees to radians
g0     = 9.81;              % ...Sea-level acceleration of gravity (m/s)
Re     = 600e3;             % ...Radius of the earth (m)
hscale = 5e3;               % ...Density scale height (m)
h_atm  = 69e3;              % ...Cut-off altitude of atmosphere (m)
rho0   = 1.225;             % ...Sea level density of atmosphere (kg/m^3)

% diam   = 2;                 % ...Vehicle diameter (m)

% A      = pi/4*(diam)^2;     % ...Frontal area (m^2)
CDA    = 5e-5;              % ...Drag coefficient (assumed constant)
m0     = 1;                 % ...Lift-off mass (kg)
% n      = 8;                 % ...Mass ratio
% T2W    = 1.4;             % ...Thrust to weight ratio

Isp    = @(rho) 360-40*(rho/rho0);     % ...Specific impulse (s)

mfinal = m0/n;              % ...Burnout mass (kg)
Thrust = T2W*m0*g0;         % ...Rocket thrust (N)
m_dot  = Thrust/Isp(rho0)/g0;  % ...Propellant mass flow rate (kg/s)
mprop  = m0 - mfinal;       % ...Propellant mass (kg)
tburn  = mprop/m_dot;     % ...Burn time (s)
% hturn  = 130;               % ...Height at which pitchover begins (m)
t0     = 0;                 % ...Initial time for the numerical integration
tf     = 6*tburn;           % ...Final time for the numerical integration
tspan  = [t0,tf];           % ...Range of integration

% ...Initial conditions:
v0     = 0;                 % ...Initial velocity (m/s)
gamma0 = 89*deg;            % ...Initial flight path angle (rad)
x0     = 0;                 % ...Initial downrange distance (km)
h0     = 0;                 % ...Initial altitude (km)
vD0    = 0;                 % ...Initial value of velocity loss due % to drag (m/s)
vG0    = 0;                 % ...Initial value of velocity loss due % to gravity (m/s)

%...Initial conditions vector:
f0 = [v0; gamma0; x0; h0; vD0; vG0; m0];

%...Call to Runge-Kutta numerical integrator 'rkf45'
% rkf45 solves the system of equations df/dt = f(t):
[t,f] = ode45(@rates, tspan, f0);

%...t is the vector of times at which the solution is evaluated
%...f is the solution vector f(t)
%...rates is the embedded function containing the df/dt's
% ...Solution f(t) returned on the time interval [t0 tf]:
v     = f(:,1)*1.e-3;       % ...Velocity (km/s)
gamma = f(:,2)/deg;         % ...Flight path angle (degrees)
x     = f(:,3)*1.e-3;       % ...Downrange distance (km)
h     = f(:,4)*1.e-3;       % ...Altitude (km)
vD    = -f(:,5)*1.e-3;      % ...Velocity loss due to drag (km/s)
vG    = -f(:,6)*1.e-3;      % ...Velocity loss due to gravity (km/s)

q = zeros(length(t),1);

for i = 1:length(t)
    Rho = rho0 * exp(-h(i)*1000/hscale);   %...Air density
    q(i) = 1/2*Rho*v(i)^2;                 %...Dynamic pressure
end

if make_plot
    output
end

return

%~~~~~~~~~~~~~~~~~~~~~~~~~
function dydt = rates(~,y)
%~~~~~~~~~~~~~~~~~~~~~~~~~
% Calculates the time rates df/dt of the variables f(t)
% in the equations of motion of a gravity turn trajectory.
%-------------------------

%...Initialize dfdt as a column vector:
% dfdt = zeros(6,1);

v     = y(1); % ...Velocity
gamma = y(2); % ...Flight path angle
x     = y(3); % ...Downrange distance
h     = y(4); % ...Altitude
vD    = y(5); % ...Velocity loss due to drag
vG    = y(6); % ...Velocity loss due to gravity
m     = y(7); % ...Spacecraft mass

%...When time t exceeds the burn time, set the thrust
% and the mass flow rate equal to zero:
if m > mfinal
    T = 9.81*m0*T2W;       % ...Current thrust
else
    T = 0;                 % ...Current thrust
end

%...Check atmosphere drop off height
if h < 0
    rho = rho0;
elseif h < h_atm
    rho = rho0 * exp(-h/hscale);    % ...Exponential density variation % with altitude
else
    rho = 0;
end

m_dot = -T/Isp(rho)/g0;

g   = g0/(1 + h/Re)^2;              % ...Gravitational variation % with altitude h
    
D   = 1/2 * rho*v^2 * CDA;   % ...Drag [Equation 11.1]

%...Define the first derivatives of v, gamma, x, h, vD and vG
% ("dot" means time derivative):
%v_dot = T/m - D/m - g*sin(gamma); % ...Equation 11.6

%...Start the gravity turn when h = hturn:
if h < 0
    gamma_dot = 0;
    v_dot     = 0;
    x_dot     = 0;
    h_dot     = 0;
    vG_dot    = 0;
elseif h <= hturn && gamma >= 89*deg
    gamma_dot = 0;
    v_dot     = T/m - D - g;
    x_dot     = 0;
    h_dot     = v;
    vG_dot    = -g;
elseif gamma > gamma0/deg
    v_dot     = T/m - D - g*sin(gamma);
    gamma_dot = -1/deg;                             % ...Equation 11.7
    x_dot     = Re/(Re + h)*v*cos(gamma);           % ...Equation 11.8(1)
    h_dot     = v*sin(gamma);                       % ...Equation 11.8(2)
    vG_dot    = -g*sin(gamma);                      % ...Gravity loss rate
else
    v_dot     = T/m - D - g*sin(gamma);
    gamma_dot = -1/v*(g - v^2/(Re + h))*cos(gamma); % ...Equation 11.7
    x_dot     = Re/(Re + h)*v*cos(gamma);           % ...Equation 11.8(1)
    h_dot     = v*sin(gamma);                       % ...Equation 11.8(2)
    vG_dot    = -g*sin(gamma);                      % ...Gravity loss rate
end

vD_dot = -D;  % ...Drag loss rate

%...Load the first derivatives of f(t) into the vector dfdt:
dydt(1,1) = v_dot;
dydt(2,1) = gamma_dot;
dydt(3,1) = x_dot;
dydt(4,1) = h_dot;
dydt(5,1) = vD_dot;
dydt(6,1) = vG_dot;
dydt(7,1) = m_dot;
end

%~~~~~~~~~~~~~~
function output 
%~~~~~~~~~~~~~~
% fprintf('\n\n -----------------------------------\n')
% fprintf('\n Initial flight path angle = %10g deg ',gamma0/deg)
% fprintf('\n Pitchover altitude = %10g m ',hturn)
% fprintf('\n Burn time = %10g s ',tburn)
% fprintf('\n Final speed = %10g km/s',v(end))
% fprintf('\n Final flight path angle = %10g deg ',gamma(end))
% fprintf('\n Altitude = %10g km ',h(end))
% fprintf('\n Downrange distance = %10g km ',x(end))
% fprintf('\n Drag loss = %10g km/s',vD(end))
% fprintf('\n Gravity loss = %10g km/s',vG(end))
% fprintf('\n\n -----------------------------------\n')

figure(4)
plot(x, h)
axis equal
xlabel('Downrange Distance (km)')
ylabel('Altitude (km)')
% axis([-inf, inf, -inf, inf])
grid
% 
% figure(5)
% subplot(2,1,1)
% plot(h, v)
% xlabel('Altitude (km)')
% ylabel('Speed (km/s)')
% axis([-inf, inf, -inf, inf])
% grid
% 
% subplot(2,1,2)
% plot(t, gamma)
% xlabel('Time (s)')
% ylabel('Flight path angle (deg)')
% axis([-inf, inf, -inf, inf])
% grid
% 
% figure(6)
% plot(h, q)
% xlabel('Altitude (km)')
% ylabel('Dynamic pressure (N/m^2)')
% axis([-inf, inf, -inf, inf])
% grid

end %output

end %Example_11_03
% ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
