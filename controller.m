function [F, M] = controller(t, state, des_state, params)
%CONTROLLER  Controller for the quadrotor
%
%   Inputs:
%   t          : Current time (unused in this implementation)
%   state      : Current state of the robot with fields:
%                pos   = [x; y; z]          (position)
%                vel   = [x_dot; y_dot; z_dot] (velocity)
%                rot   = [phi; theta; psi]  (orientation)
%                omega = [p; q; r]          (angular velocities)
%   des_state  : Desired state with fields:
%                pos   = [x; y; z]          (desired position)
%                vel   = [x_dot; y_dot; z_dot] (desired velocity)
%                acc   = [x_ddot; y_ddot; z_ddot] (desired acceleration)
%                yaw   : desired yaw angle
%                yawdot: desired yaw rate
%   params     : Robot parameters, including mass and gravity

% Extract desired position, velocity, and acceleration
xd = des_state.pos(1);
yd = des_state.pos(2);
zd = des_state.pos(3);

xd_dot = des_state.vel(1);
yd_dot = des_state.vel(2);
zd_dot = des_state.vel(3);

xd_ddot = des_state.acc(1);
yd_ddot = des_state.acc(2);
zd_ddot = des_state.acc(3);

% Extract desired yaw and yaw rate
yawd = des_state.yaw;
yawd_dot = des_state.yawdot;

% Extract current position, velocity, and orientation
x = state.pos(1);
y = state.pos(2);
z = state.pos(3);

x_dot = state.vel(1);
y_dot = state.vel(2);
z_dot = state.vel(3);

phi = state.rot(1);
theta = state.rot(2);
psi = state.rot(3);

% Extract current angular velocities
p = state.omega(1);
q = state.omega(2);
r = state.omega(3);

% Controller gains
kpx = 200; kdx = 10;   % Proportional and derivative gains for x
kpy = 200; kdy = 10;   % Proportional and derivative gains for y
kpz = 400; kdz = 100;  % Proportional and derivative gains for z

kpphi = 200; kdphi = 5;        % Proportional and derivative gains for roll (phi)
kptheta = 200; kdtheta = 5;    % Proportional and derivative gains for pitch (theta)
kppsi = 1; kdpsi = 0.01;       % Proportional and derivative gains for yaw (psi)

% Calculate desired roll (phi_com) and pitch (theta_com) commands
phi_com = (1/params.gravity) * ...
          (sin(yawd) * (xd_ddot + kpx*(xd - x) + kdx*(xd_dot - x_dot)) - ...
           cos(yawd) * (yd_ddot + kpy*(yd - y) + kdy*(yd_dot - y_dot)));

theta_com = (1/params.gravity) * ...
            (cos(yawd) * (xd_ddot + kpx*(xd - x) + kdx*(xd_dot - x_dot)) + ...
             sin(yawd) * (yd_ddot + kpy*(yd - y) + kdy*(yd_dot - y_dot)));

% Calculate thrust (F) required to achieve desired z position
F = params.mass * (params.gravity + zd_ddot + kpz*(zd - z) + kdz*(zd_dot - z_dot));

% Calculate moments (M) required to achieve desired orientation
M = [kpphi * (phi_com - phi) + kdphi * (0 - p);       % Roll control (phi)
     kptheta * (theta_com - theta) + kdtheta * (0 - q); % Pitch control (theta)
     kppsi * (yawd - psi) + kdpsi * (yawd_dot - r)];  % Yaw control (psi)

end
