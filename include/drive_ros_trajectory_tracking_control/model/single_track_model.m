
clear all;
close all;
clc

%% Init

import casadi.*


% casadi opts for code generation
if CasadiMeta.version()=='3.4.0'
	% casadi 3.4
	opts = struct('mex', false, 'casadi_int', 'int', 'casadi_real', 'double','with_header',true,'cpp',true);
else
	% old casadi versions
	error('Please download and install Casadi 3.4.0')
end
model_name_prefix = 'single_track_model_';

%% define the symbolic variables
SymbolicVariables;

%% load parameters
Parameters;

%% define ode in explicit form
SystemDynamics;

%% generate casadi C functions
nx = 10;
nu = 3;


% impl_ode_fun

impl_ode_fun = Function([model_name_prefix,'impl_ode_fun'], {x, dx, u, p}, {fi});
impl_ode_fun.generate([model_name_prefix,'impl_ode_fun'], opts);


% impl_ode_fun_jac_x_xdot

impl_ode_fun_jac_x_xdot = Function([model_name_prefix,'impl_ode_fun_jac_x_xdot'], {x, dx, u, p}, {fi, jacobian(fi, x), jacobian(fi, dx)});
impl_ode_fun_jac_x_xdot.generate([model_name_prefix,'impl_ode_fun_jac_x_xdot'], opts);


% impl_ode_jac_x_xdot_u

impl_ode_jac_x_xdot_u = Function([model_name_prefix,'impl_ode_jac_x_xdot_u'], {x, dx, u, p}, {jacobian(fi, x), jacobian(fi, dx), jacobian(fi, u)});
impl_ode_jac_x_xdot_u.generate([model_name_prefix,'impl_ode_jac_x_xdot_u'], opts);


% impl_fun_ode_jac_x_xdot_u

impl_ode_fun_jac_x_xdot_u = Function([model_name_prefix,'impl_ode_fun_jac_x_xdot_u'], {x, dx, u, p}, {fi, jacobian(fi, x), jacobian(fi, dx), jacobian(fi, u)});
impl_ode_fun_jac_x_xdot_u.generate([model_name_prefix,'impl_ode_fun_jac_x_xdot_u'], opts);


