function [ u1, u2 ] = controller(~, state, des_state, params)
%CONTROLLER  Controller for the planar quadrotor
%
%   state: The current state of the robot with the following fields:
%   state.pos = [y; z], state.vel = [y_dot; z_dot], state.rot = [phi],
%   state.omega = [phi_dot]
%
%   des_state: The desired states are:
%   des_state.pos = [y; z], des_state.vel = [y_dot; z_dot], des_state.acc =
%   [y_ddot; z_ddot]
%
%   params: robot parameters

%   Using these current and desired states, you have to compute the desired
%   controls

u1 = 0;
u2 = 0;

% FILL IN YOUR CODE HERE
k_p_y =16;
k_v_y = 6;
k_p_z = 50;
k_v_z = 7;
k_p_phi = 300;
k_v_phi = 30;

phi_c_dot = 0;
phi_c_ddot = 0;

phi_c = -1 / params.gravity * (des_state.acc(1) + k_v_y * (des_state.vel(1) - state.vel(1)) + k_p_y * (des_state.pos(1) - state.pos(1)));
u1 = params.mass * (params.gravity + des_state.acc(2) + k_v_z * (des_state.vel(2) - state.vel(2)) + k_p_z * (des_state.pos(2) - state.pos(2)));
u2 = params.Ixx * (phi_c_ddot + k_v_phi * (phi_c_dot - state.omega) + k_p_phi * (phi_c - state.rot));
end

