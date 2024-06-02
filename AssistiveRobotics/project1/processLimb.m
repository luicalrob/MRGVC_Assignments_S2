function [q_interpolation, velocities, T_final, cartesian_traj] = processLimb(limb, q_initial, q_final, q_interp, vel, cart, n_interp, sample_time)
    % Joint trajectory interpolation
    new_q = jtraj(q_initial, q_final, n_interp);

    % Compute the velocity between each interpolated joint position
    % First, calculate differences between consecutive positions
    position_differences = diff(new_q, 1, 1);  % Computes the difference between consecutive rows
    
    q_interpolation = [q_interp; new_q(2:end, :)];

    % Then, divide by sample_time to get velocity
    velocities = [vel; position_differences / sample_time];

    % Forward kinematics
    T_initial = fkine(limb, q_initial);
    T_final = fkine(limb, q_final);
    
    % Cartesian trajectory interpolation
    new_cart = ctraj(T_initial, T_final, n_interp);
    cartesian_traj = [cart, new_cart(2:end)];
end
