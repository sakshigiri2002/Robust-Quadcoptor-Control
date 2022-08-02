function [ u ] = pd_controller(~, s, s_des, params)
%PD_CONTROLLER  PD controller for the height
%
%   s: 2x1 vector containing the current state [z; v_z]
%   s_des: 2x1 vector containing desired state [z; v_z]
%   params: robot parameters

e = s_des - s;
m = params.mass;
g = params.gravity;
u_min = params.u_min
u_max = params.u_max
% FILL IN YOUR CODE HERE
kp = 150
kv = 20
u = kp*e(1) + kv*e(2) + m*g ;
if (u>u_max)
    u = u_max ;
end
if (u<u_min)
    u = u_min ;
end



end

