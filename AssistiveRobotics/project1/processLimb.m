function [q_interpolation, velocities, T_final, cartesian_traj] = processLimb(limb, q_initial, q_final, q_interp, vel, cart, n_interp, sample_time)
    % Joint trajectory interpolation
    q_interpolation = [q_interp; jtraj(q_initial, q_final, n_interp)];
    
    % Velocity calculation
    velocities = [vel; repmat((q_initial - q_final) / (n_interp * sample_time), n_interp, 1)];
    
    % Forward kinematics
    T_initial = fkine(limb, q_initial);
    T_final = fkine(limb, q_final);
    
    % Cartesian trajectory interpolation
    cartesian_traj = [cart, ctraj(T_initial, T_final, n_interp)];
end
