%% States
import casadi.*

% States 
x = MX.sym('x',10); 

% lateral displacement ( m ) 
y = x(1);
% heading deviation ( rad ) 
psi = x(2);
% velocity ( m/s ) 
v = x(3);
% slip angle ( rad ) 
beta = x(4);
% yaw rate ( rad/s ) 
yaw = x(5);
% acceleration ( m/s^2 ) 
a = x(6);
% front steering angle ( rad ) 
delta_f = x(7);
% rear steering angle ( rad ) 
delta_r = x(8);
% front steering angle rate( rad/s )
ddelta_f = x(9);
% rear steering angle rate ( rad/s )
ddelta_r = x(10);

%% Differential State Variables 

% Differntial States 
dx = MX.sym('dx',10); 

% lateral displacement velocity ( m/s ) 
y_dot = dx(1);
% heading deviation rate ( rad/s ) 
psi_dot = dx(2);
% acceleration ( m/s^2 ) 
v_dot = dx(3);
% slip angle rate ( rad/s ) 
beta_dot = dx(4);
% yaw angle acceleration ( rad/s^2 ) 
yaw_dot = dx(5);
% jerk ( m/s^3 ) 
a_dot = dx(6);
% front steering angle rate ( rad/s ) 
delta_f_dot = dx(7);
% rear steering angle rate ( rad/s ) 
delta_r_dot = dx(8);
% front steering angle acceleration ( rad/s^2 )
ddelta_f_dot = dx(9);
% rear steering angle rate ( rad/s^2 )
ddelta_r_dot = dx(10);

%% Control inputs 

% Control inputs 
u = MX.sym('u',3); 

% Desired acceleration ( m/s^2 )
a_ref = u(1);
% Desired front steering angle rate ( rad/s )
ddelta_f_ref = u(2);
% Desired rear steering angle rate ( rad/s ) 
ddelta_r_ref = u(3);

%% Disturbance 
p = MX.sym('p',1); 