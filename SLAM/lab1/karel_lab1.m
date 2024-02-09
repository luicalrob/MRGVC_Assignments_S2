%-------------------------------------------------------------------------
% University of Zaragoza
%
% Author:  J. Neira
%-------------------------------------------------------------------------
% SLAM for Karel the robot in 1D
%-------------------------------------------------------------------------

clear all; % varialbes
close all; % figures
randn('state', 1); % always use same random number sequence
rand('state', 1); % always use same random number sequence
format long

%-------------------------------------------------------------------------
% configuration parameters

global config;

% display correlation matrix and pause at every step
config.step_by_step = 0;

%number of robot motions for each local map
config.steps_per_map = 125;

% figure counter (to always plot a new figure)
config.fig = 0;

config.total_maps = 8;
%-------------------------------------------------------------------------

%-------------------------------------------------------------------------
% 1D world characteristics

global world;

% In the absolute W reference, these will not change during program execution
world.true_point_locations = [0.5:1:100000]';

% In the absolute W reference, this will change as the robot moves
world.true_robot_location = 0;

%-------------------------------------------------------------------------

%-------------------------------------------------------------------------
% robot characteristics

global robot;

%    factor_x: fraction of the motion that constitutes odometry error
%     true_uk: ground true robot motion per step

robot.factor_x = 0.1; % ten percent
robot.true_uk = 1; % each true motion is 1m
%-------------------------------------------------------------------------

%-------------------------------------------------------------------------
% sensor characteristics

global sensor;

%     factor_z: measurement error is this fraction of the distance
%    range_min: minimum sensor range
%    range_max: maximum sensor range

sensor.factor_z = 0.01; % one percent
sensor.range_min = 0;
sensor.range_max = 2;
%-------------------------------------------------------------------------

%-------------------------------------------------------------------------
% all map details

global map;

%            R0: absolute location of base reference for map
%         hat_x: estimated robot and feature locations
%         hat_P: robot and feature covariance matrix
%             n: number of features in map
%        true_x: true robot and feature location (with respect to R0)
%      true_ids: true label of features in map (according to world)
%  stats.true_x: true robotlocation wrt R0 for the whole trajectory
% stats.error_x: robotlocation error at every step of the trajectory
% stats.sigma_x: robot location uncertainty at every step of the trajectory
%  stats.cost_t: computational cost (elapsed time) for each step
%-------------------------------------------------------------------------

%-------------------------------------------------------------------------
% measurements (not global, but this is the structure)

%            z_k: measured distance to all visible features
%            R_k: covariance of z_k
%          ids_f: true ids of the ones that are already in the map
%          ids_n: true ids of the ones that new to the map
%        z_pos_f: positions in z_r of ids_f
%        z_pos_n: positions in z_r of ids_n
%        x_pos_f: positions in hat_x of ids_f
%            z_f: z_k(ids_f), measurements to features already in the map
%            R_f: R_k(ids_f), meas. cov. to features already in the map
%            z_n: z_k(ids_n), measurements to features new to the map
%            R_n: R_k(ids_n), Meas. cov. to features new to the map
%-------------------------------------------------------------------------

global global_map;

%-------------------------------------------------------------------------
% BEGIN
%-------------------------------------------------------------------------
for k = 1:config.total_maps
    fprintf("Computing map %d... \n", k);
    [map] = Kalman_filter_slam (map, config.steps_per_map);
    if (k > 1)
        tjoin = tic;
        [global_map] = join_maps(global_map, map);
        % matching and fusion (exercise 11)
        [global_map] = match_and_fuse(global_map);
        global_map.stats.cost_t(end) = global_map.stats.cost_t(end) + toc(tjoin);
    else
        global_map = map;
    end
end

display_map_results (global_map);

%-------------------------------------------------------------------------
% END
%-------------------------------------------------------------------------

%-------------------------------------------------------------------------
% Kalman_filter_slam
%-------------------------------------------------------------------------

function[map] = Kalman_filter_slam (map, steps)

    global world;
    
    % initial ground truth of hat_x 
    map.true_x = zeros(steps);
    map.true_x = [0];
    
    % initial number of features
    map.n = 0;
    
    % initial map state and covariance
    map.R0 = world.true_robot_location;
    
    map.hat_x = zeros(steps);
    map.hat_x = [0];
    
    map.hat_P = zeros(steps,steps);
    map.hat_P = [0];
    
    % feature ids, for robot = 0
    map.true_ids = [0];
    
    % useful statistics
    map.stats.true_x = [0];
    map.stats.error_x = [];
    map.stats.sigma_x = [];
    map.stats.cost_t = [];
    
    for k = 0:steps
        
        tstart = tic;
        
        if k > 0
            [map] = compute_motion (map);
        end
        
        % get measurements
        [measurements] = get_measurements_and_data_association(map);
        
        %disp(map.n);
        % seen features already in the map? KF update!
        if not(isempty(measurements.z_f))
            [map] = update_map (map, measurements);
        end
        
        % some new features?
        if not(isempty(measurements.z_n))
            [map] = add_new_features (map, measurements);
        end
        
        % record statistics
        map.stats.error_x = [map.stats.error_x; (map.stats.true_x(end) - map.hat_x(1))];
        map.stats.sigma_x = [map.stats.sigma_x; sqrt(map.hat_P(1,1))];
        map.stats.cost_t = [map.stats.cost_t; toc(tstart)];
        
    end
end

%-------------------------------------------------------------------------
% compute_motion
%-------------------------------------------------------------------------

function [map] = compute_motion (map)

    global config;
    global robot;
    global world;
    
    % move robot and update ground truth robot location
    world.true_robot_location = world.true_robot_location + robot.true_uk;
    
    % odometry error
    sigma_xk = robot.factor_x * robot.true_uk;
    error_k = randn(1)*sigma_xk;
    
    % estimated motion
    map.hat_x(1) = map.hat_x(1) + robot.true_uk + error_k;
    map.hat_P(1,1) = map.hat_P(1,1) + sigma_xk^2;
    
    % update ground truth for the current map
    map.true_x(1) = map.true_x(1) + robot.true_uk;
    map.stats.true_x = [map.stats.true_x; map.stats.true_x(end) + robot.true_uk];
    
    if config.step_by_step
        fprintf('Move to %d...\n',map.true_x(1));
        plot_correlation(map.hat_P);
        pause
    end
end

%-------------------------------------------------------------------------
% get_measurements
%-------------------------------------------------------------------------

function [measurements] = get_measurements_and_data_association(map)

    global world;
    global sensor;
    
    distances = world.true_point_locations - world.true_robot_location;
    visible_ids = find((distances >= sensor.range_min) & (distances <= sensor.range_max));
    visible_d = distances(visible_ids);
    n_visible = length(visible_ids);
    
    % sensor error
    sigma_z = sensor.factor_z * visible_d;
    error_z = randn(n_visible, 1) .* sigma_z;
    
    measurements.z_k = visible_d + error_z;
    measurements.R_k = diag(sigma_z.^2);
    
    % data association
    measurements.ids_f = intersect(map.true_ids, visible_ids);
    measurements.ids_n = setdiff(visible_ids, map.true_ids);
    measurements.z_pos_f = find(ismember(visible_ids, measurements.ids_f));
    measurements.z_pos_n = find(ismember(visible_ids, measurements.ids_n));
    measurements.x_pos_f = find(ismember(map.true_ids, visible_ids));
    
    %features already in the map
    measurements.z_f = measurements.z_k(measurements.z_pos_f);
    measurements.R_f = measurements.R_k(measurements.z_pos_f,measurements.z_pos_f);
    
    %new features
    measurements.z_n = measurements.z_k(measurements.z_pos_n);
    measurements.R_n = measurements.R_k(measurements.z_pos_n,measurements.z_pos_n);

end

%-------------------------------------------------------------------------
% update_map
%-------------------------------------------------------------------------

function [map] = update_map (map, measurements)

    global config;
    
    % DO SOMETHING HERE!
    % You need to compute H_k, y_k, S_k, K_k and update map.hat_x and map.hat_P
    
    H_k = sparse(length(measurements.z_f), length(map.hat_x));
    H_k(:,1) = -1;
    H_k(measurements.z_pos_f, measurements.x_pos_f) = 1;
    
    y_k = measurements.z_f - H_k * map.hat_x;
    S_k = H_k * map.hat_P * H_k' + measurements.R_f;
    K_k = map.hat_P * H_k' / S_k;
    
    map.hat_x = map.hat_x + K_k * y_k;
    map.hat_P = (eye(length(map.hat_x)) - K_k * H_k) * map.hat_P;
    
    if config.step_by_step
        fprintf('Updape map for last %d features...\n',length(measurements.ids_f));
        plot_correlation(map.hat_P);
        pause
    end
end

%-------------------------------------------------------------------------
% add_new_features
%-------------------------------------------------------------------------

function [map] = add_new_features (map, measurements)

    global config;
    global world;
    
    % DO SOMETHING HERE!
    % update map.hat_x, map.hat_P, map.true_ids, map.true_x, map.n
    
    m = map.n;
    n = length(measurements.z_n);
    A = sparse(1 + m + n, 1 + m); 
    B = sparse(1 + m + n, n);
    
    A(1:1+m , 1:1+m) = eye(1+m);
    A(2+m:1+m+n , 1) = 1;
    
    B(2+m : 1+m+n , 1:n) = eye(n);
    
    % candidatas a ser las matrices sparse, porque tienen muchos ceros
    map.hat_x = A * map.hat_x + B * measurements.z_n;
    map.hat_P = A * map.hat_P * A' + B * measurements.R_n * B';
    
    map.true_ids = [map.true_ids; measurements.ids_n];
    map.n = m + n;
    map.true_x = [world.true_robot_location; world.true_point_locations(1:map.n)];
    
    if config.step_by_step
        fprintf('Add %d new features...\n',length(measurements.ids_n));
        plot_correlation(map.hat_P);
        pause
    end
end

function [global_map] = join_maps(global_map, map)
    global world;
    global sensor;

    m = global_map.n + 1;
    n = map.n + 1;
    A = sparse(m + n - 1, m); 
    B = sparse(m + n - 1, n);
    
    A(1:m , 1:m) = eye(m);
    A(m+1:m+n-1 , 1) = 1;
        
    B(1, 1) = 1;
    B(m+1:m+n-1 , 2:n) = eye(n-1);

    global_map.hat_x = A * global_map.hat_x + B * map.hat_x;
    global_map.hat_P = A * global_map.hat_P * A' + B * map.hat_P * B';
    global_map.n = global_map.n + map.n;
    
    global_map.true_ids = [global_map.true_ids; map.true_ids(2:end)];
    global_map.true_x = [world.true_robot_location; global_map.true_x(2:end); global_map.true_x(end) + 0.5 - sensor.range_max + map.true_x(2:end)];

    % record statistics
    global_map.stats.error_x = [global_map.stats.error_x; global_map.stats.error_x(end) + map.stats.error_x];
    global_map.stats.sigma_x = [global_map.stats.sigma_x; sqrt(global_map.stats.sigma_x(end)^2 + map.stats.sigma_x.^2 )];
    global_map.stats.cost_t = [global_map.stats.cost_t; map.stats.cost_t];
    global_map.stats.true_x = [global_map.stats.true_x; global_map.stats.true_x(end) + 1 + map.stats.true_x];
    
end

function [global_map] = match_and_fuse(global_map)

    % Find unique values and their counts
    [unique_vals, ~, idx] = unique(global_map.true_ids);
    counts = histcounts(idx, numel(unique_vals));

    % Find indices of duplicated values
    duplicated_elements = find(counts > 1);

    % Initialize array to store pairs of indices
    positions = zeros(length(duplicated_elements), 2);
    
    % Iterate over duplicated values to find their indices
    for i = 1:length(duplicated_elements)
        id = unique_vals(duplicated_elements(i));
        indices = find(global_map.true_ids == id);
        positions(i, :) = indices(1:2);
    end

    H_k = sparse(1:numel(duplicated_elements), positions(:, 1), 1, numel(duplicated_elements), numel(global_map.true_ids));
    H_k = H_k - sparse(1:numel(duplicated_elements), positions(:, 2), 1, numel(duplicated_elements), numel(global_map.true_ids));

    y_k = - H_k * global_map.hat_x;
    S_k = H_k * global_map.hat_P * H_k';
    K_k = global_map.hat_P * H_k' / S_k;
    
    global_map.hat_x = global_map.hat_x - K_k * y_k;
    global_map.hat_P = (eye(length(global_map.hat_x)) - K_k * H_k) * global_map.hat_P;
    global_map.n = global_map.n - length(duplicated_elements);

    for i = 1:length(duplicated_elements)    
        % remove
        global_map.hat_x(positions(i,2)-(i-1), :) = [];
        global_map.hat_P(positions(i,2)-(i-1), :) = [];
        global_map.hat_P(:, positions(i,2)-(i-1)) = [];

        global_map.true_ids(positions(i,2)-(i-1), :) = [];
        global_map.true_x(positions(i,2)-(i-1), :) = [];
           
        % global_map.hat_x(positions(i,1)-(i-1), :) = [];
        % global_map.hat_P(positions(i,1)-(i-1), :) = [];
        % global_map.hat_P(:, positions(i,1)-(i-1)) = [];
        % 
        % global_map.true_ids(positions(i,1)-(i-1), :) = [];
        % global_map.true_x(positions(i,1)-(i-1), :) = [];
    
    end

end

%-------------------------------------------------------------------------
% display_results
%-------------------------------------------------------------------------

function  display_map_results (map)

    global config;
    
    config.fig = config.fig + 1;
    figure(config.fig);
    axis([0 length(map.hat_x) -2*max(sqrt(diag(map.hat_P))) 2*max(sqrt(diag(map.hat_P)))]);
    grid on;
    hold on;
    plot(map.true_ids, map.hat_x-map.true_x, 'ro','Linewidth', 2);
    plot(map.true_ids, 2*sqrt(diag(map.hat_P)), 'b+','Linewidth', 2);
    plot(map.true_ids, -2*sqrt(diag(map.hat_P)), 'b+','Linewidth', 2);
    xlabel('Feature number (robot = 0)');
    ylabel('meters (m)');
    title('Map estimation error + 2sigma bounds');
    
    config.fig = config.fig + 1;
    figure(config.fig);
    plot_correlation(map.hat_P);
    title(sprintf('Correlation matrix of size %d', size(map.hat_P,1)));
    
    config.fig = config.fig + 1;
    figure(config.fig);
    grid on;
    hold on;
    plot(map.stats.true_x, map.stats.cost_t,'r-','Linewidth',2);
    xlabel('step');
    ylabel('seconds');
    title('Cost per step');
    
    config.fig = config.fig + 1;
    figure(config.fig);
    grid on;
    hold on;
    plot(map.stats.true_x,cumsum(map.stats.cost_t),'r-','Linewidth',2);
    xlabel('step');
    ylabel('seconds');
    title('Cumulative cost');
    
    config.fig = config.fig + 1;
    figure(config.fig);
    axis([0 map.stats.true_x(end) -2*max(map.stats.sigma_x) 2*max(map.stats.sigma_x)]);
    grid on;
    hold on;
    plot(map.stats.true_x, map.stats.error_x,'r-','Linewidth',2);
    plot(map.stats.true_x, 2*map.stats.sigma_x,'b-','Linewidth',2);
    plot(map.stats.true_x,-2*map.stats.sigma_x,'b-','Linewidth',2);
    xlabel('meters (m)');
    ylabel('meters (m)');
    title('Robot estimation error + 2sigma bounds');

end

%-------------------------------------------------------------------------
% plot_correlation
%-------------------------------------------------------------------------

function plot_correlation(P)

    ncol=256 ;
    
    % hot cold colormap
    cmap=hsv2rgb([linspace(2/3, 0, ncol)' 0.9*ones(ncol,1) ones(ncol,1)]) ;
    cmap(1,:)=[0 0 0] ;
    colormap(cmap) ;
    
    corr = correlation(P);
    imagesc(abs(corr), [0 1]) ;  % no sign
    
    axis image;
    colorbar ;

end

%-------------------------------------------------------------------------
% correlation
%-------------------------------------------------------------------------

function Corr=correlation(Cov)

    sigmas = sqrt(diag(Cov));
    Corr = diag(1./sigmas)*Cov*diag(1./sigmas);

end
