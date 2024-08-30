function [desired_state] = traj_generator(t, state, waypoints)
    % TRAJ_GENERATOR Generates the trajectory for the quadrotor.
    % This function computes the desired position, velocity, and acceleration 
    % of the quadrotor based on polynomial coefficients for each segment.
    %
    % INPUTS:
    % t         - Current time
    % state     - Current state (not used here but included for consistency)
    % waypoints - A 3xN matrix of waypoints where each column represents [x; y; z]
    %
    % OUTPUT:
    % desired_state - Struct containing position, velocity, acceleration, yaw, and yawdot

    % Persistent variables to store information across function calls
    persistent total_time time_segment coeffx coeffy coeffz num_segments initialized waypoints_persistent

    if nargin == 3 && ~isempty(waypoints)
        % Initialization phase: Calculate polynomial coefficients for each segment
        
        % Store waypoints and calculate the number of segments
        waypoints_persistent = waypoints;
        num_segments = size(waypoints, 2) - 1;
        
        % Preallocate coefficient matrices for x, y, z dimensions
        coeffx = zeros(6, num_segments);
        coeffy = zeros(6, num_segments);
        coeffz = zeros(6, num_segments);
        
        % Define the total time for the trajectory
        total_time = 8; 
        
        % Calculate distances between consecutive waypoints
        distances = sqrt(sum(diff(waypoints, 1, 2).^2, 1));
        total_distance = sum(distances);
        
        % Allocate time for each segment proportionally to the distance
        time_segment = total_time * (distances / total_distance);

        % Initialize velocity and acceleration for each segment
        v = zeros(3, num_segments + 1);  
        a = zeros(3, num_segments + 1);  
        
        % Calculate velocities and accelerations for each segment
        for i = 1:num_segments
            v(:, i+1) = (waypoints(:, i + 1) - waypoints(:, i)) / time_segment(i);
            a(:, i+1) = v(:, i+1) / time_segment(i);
        end
        
        % Ensure the quadrotor comes to a smooth stop at the final waypoint
        v(:, end) = [0; 0; 0];
        a(:, end) = [0; 0; 0];
        
        % Calculate polynomial coefficients for each segment
        for i = 1:num_segments
            t1 = 0;
            t2 = time_segment(i);

            A = [
                1  t1  t1^2  t1^3  t1^4  t1^5;
                0  1   2*t1  3*t1^2  4*t1^3  5*t1^4;
                0  0   2     6*t1   12*t1^2  20*t1^3;
                1  t2  t2^2  t2^3  t2^4  t2^5;
                0  1   2*t2  3*t2^2  4*t2^3  5*t2^4;
                0  0   2     6*t2   12*t2^2  20*t2^3;
            ];
   
            for j = 1:3
                B = [waypoints(j, i); v(j,i); a(j,i); waypoints(j, i+1); v(j, i+1); 0];
                
                if j == 1
                    coeffx(:, i) = A\B;
                elseif j == 2
                    coeffy(:, i) = A\B;
                else
                    coeffz(:, i) = A\B;
                end
            end
        end
        
        % Update total time considering the final segment to reach the last waypoint
        t_f = time_segment(end);
        final_segment_time = max(abs(coeffx(1,end) + t_f*coeffx(2,end) + t_f^2*coeffx(3,end) + t_f^3*coeffx(4,end) + t_f^4*coeffx(5,end) + coeffx(6,end)* t_f^5 - waypoints(1,end)));
        total_time = total_time + final_segment_time;
        
        % Mark initialization as complete
        initialized = true;
        desired_state = [];
        
    elseif nargin == 2 && initialized
        % Calculation phase: Determine the desired state at time t
        
        % Ensure the time does not exceed the total trajectory time
        if t >= total_time
            t = total_time;
        end
        
        % Identify the current segment based on time t
        for i = 1:num_segments
            if t >= sum(time_segment(1:i-1)) && t < sum(time_segment(1:i))
                t_seg = t - sum(time_segment(1:i-1));
                
                A_mold = [
                    1  t_seg  t_seg^2  t_seg^3  t_seg^4  t_seg^5;
                    0  1  2*t_seg  3*t_seg^2  4*t_seg^3  5*t_seg^4;
                    0  0  2     6*t_seg   12*t_seg^2  20*t_seg^3;
                ];
                
                % Compute desired position, velocity, and acceleration
                desired_state.pos = [A_mold(1,:)*coeffx(:,i); A_mold(1,:)*coeffy(:,i); A_mold(1,:)*coeffz(:,i)];
                desired_state.vel = [A_mold(2,:)*coeffx(:,i); A_mold(2,:)*coeffy(:,i); A_mold(2,:)*coeffz(:,i)];
                desired_state.acc = [A_mold(3,:)*coeffx(:,i); A_mold(3,:)*coeffy(:,i); A_mold(3,:)*coeffz(:,i)];
                desired_state.yaw = 0;
                desired_state.yawdot = 0;
                return;
            end
        end
        
        % Ensure quadrotor is at rest at the end of the trajectory
        desired_state.pos = waypoints_persistent(:, end);
        desired_state.vel = zeros(3, 1);
        desired_state.acc = zeros(3, 1);
        desired_state.yaw = 0;
        desired_state.yawdot = 0;
    
    end
end
