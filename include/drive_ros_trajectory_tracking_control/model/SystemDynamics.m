%% Set-up nonlinear state-space model

 % front lateral tire force ( N ) 
 F_lat_f = stiffness * (delta_f - beta - lf * yaw/ v);
 % rear lateral tire force ( N ) 
 F_lat_r = stiffness * (delta_r - beta + lr * yaw / v);
 % front and rear longitudinal force ( N )
 F_lon = m * a / (cos(delta_f) + cos(delta_r));


%% Explicit Nonlinear State-Space Model
 fe = [ ... 
% lateral displacement velocity ( m/s ) 
 v * sin(beta + psi);
% heading deviation rate ( rad/s ) 
 yaw;
% acceleration ( m/s^2 )
 1 / m * (F_lon * cos(delta_f - beta) + F_lon * cos(delta_r - beta)...
 - F_lat_f * sin(delta_f - beta) - F_lat_r * sin(delta_r - beta));
% slip angle rate ( rad/s ) 
 - yaw + 1 / (m * v) * (F_lon * sin(delta_f - beta) + F_lon * sin(delta_r - beta)...
 + F_lat_f * cos(delta_f - beta) + F_lat_r * cos(delta_r - beta));
% yaw angle acceleration ( rad/s^2 ) 
 1 / J_z * (lf * F_lon * sin(delta_f) - lr * F_lon * sin(delta_r)...
 + lf * F_lat_f * cos(delta_f) - lr * F_lat_r * cos(delta_r));
% jerk ( m/s^3 ) 
 1 / T_ax * (a_ref - a);
% front steering angle rate ( rad/s )
 ddelta_f;
% rear steering angle rate ( rad/s )
 ddelta_r;
% front steering angle acceleration ( rad/s^2 ) 
 1 / T_steer * (ddelta_f_ref - ddelta_f);
% rear steering angle acceleration ( rad/s^2 ) 
 1 / T_steer * (ddelta_r_ref - ddelta_r);
 ];



%% Implicit Nonlinear State-Space Model
fi = dx - fe;
