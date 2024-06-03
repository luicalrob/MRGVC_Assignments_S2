clear all;
close all;

l_l1 = 0.6;
l_l2 = 0.3;

l_a1 = 0.5;
l_a2 = 0.25;

l_hip = 0.25;
l_shoulder = 0.37;

l_hip_to_shoulder = 0.8;

%% LEFT LEG

%       [theta,     D,      A,      alpha,  sigma,  offset]

% HIP
% DH parameters for 0T1 (Circumduction)
L1 = Link([0,       0,      0,      pi/2,   0,      0]); 
L1.qlim = [-pi/2, pi/2];
%  DH parameters for 1T2 (Abduction-Adduction)
L2 = Link([0,       0,      0,      pi/2,   0,      pi/2]);
L2.qlim = [-pi/2, pi];
%  DH parameters for 2T3 (Flexion-Extension)
L3 = Link([0,       0,      l_l1,      0,      0,      -pi]);
L3.qlim = [-pi, pi];

% KNEE
% DH parameters for 3T4 (Flexion-Extension)
L4 = Link([0,       0,      l_l1,      0,      0,      0]);  
L4.qlim = [pi, 2*pi];

% ANKLE
%  DH parameters for 4T5 (Plantar-dorsal flexion)
L5 = Link([0,       0,      0,      -pi/2,  0,      pi/2]);
L5.qlim = [-pi/2, 30*pi/180];
%  DH parameters for 5T6 (Abduction-Adduction)
L6 = Link([0,       0,      l_l2,      0,      0,      0]);
L6.qlim = [-pi/2, pi/2];

% Construction of an object belonging to the class robot
left_leg=SerialLink([L1,L2,L3,L4,L5,L6],'name', 'left_leg');
% Location of the base reference frame
left_leg.base=transl(0,l_hip,0)*trotz(-pi/2);
% Visualization of the robot in the joint position given by qr
left_leg_qr=[0, 0, 0, 2*pi, 0, 0];
left_leg.plot(left_leg_qr)
% Visualization of the robot and manual guidance of the different joints, starting from
% the initial joint position qr
hold on;
teach(left_leg, left_leg_qr)


%% RIGHT LEG

%       [theta,     D,      A,      alpha,  sigma,  offset]

% HIP
% DH parameters for 0T1 (Circumduction)
L1 = Link([0,       0,      0,      pi/2,   0,      0]); 
L1.qlim = [-pi/2, pi/2];
%  DH parameters for 1T2 (Abduction-Adduction)
L2 = Link([0,       0,      0,      pi/2,   0,      pi/2]);
L2.qlim = [-pi/2, pi];
%  DH parameters for 2T3 (Flexion-Extension)
L3 = Link([0,       0,      l_l1,      0,      0,      -pi]);
L3.qlim = [-pi, pi];

% KNEE
% DH parameters for 3T4 (Flexion-Extension)
L4 = Link([0,       0,      l_l1,      0,      0,      0]);  
L4.qlim = [pi, 2*pi];

% ANKLE
%  DH parameters for 4T5 (Plantar-dorsal flexion)
L5 = Link([0,       0,      0,      -pi/2,  0,      pi/2]);
L5.qlim = [-pi/2, 30*pi/180];
%  DH parameters for 5T6 (Abduction-Adduction)
L6 = Link([0,       0,      l_l2,      0,      0,      0]);
L6.qlim = [-pi/2, pi/2];

% Construction of an object belonging to the class robot
right_leg=SerialLink([L1,L2,L3,L4,L5,L6],'name', 'right_leg');
% Location of the base reference frame
right_leg.base=transl(0,-l_hip,0)*trotz(-pi/2);
% Visualization of the robot in the joint position given by qr
right_leg_qr=[0, 0, 0, 2*pi, 0, 0];
right_leg.plot(right_leg_qr)
% Visualization of the robot and manual guidance of the different joints, starting from
% the initial joint position qr
teach(right_leg, right_leg_qr)


%% LEFT ARM

%       [theta,     D,      A,      alpha,  sigma,  offset]

% SHOULDER
% DH parameters for 0T1 (Circumduction)
L1 = Link([0,       0,      0,      pi/2,   0,      pi/2]); 
L1.qlim = [-pi/2, pi];
%  DH parameters for 1T2 (Abduction-Adduction)
L2 = Link([0,       0,      0,      pi/2,   0,      pi/2]);
L2.qlim = [-pi/2, pi];
%  DH parameters for 2T3 (Flexion-Extension)
L3 = Link([0,       0,      l_a1,      pi/2,   0,      pi]);
L3.qlim = [-pi, pi/2];

% ELBOW
% DH parameters for 3T4 (Flexion-Extension)
L4 = Link([0,       0,      0,      pi/2,   0,      pi]);  
L4.qlim = [0, pi];
% DH parameters for 4T5 (Pronation-Supination)
L5 = Link([0,       0,      l_a1,      pi/2,   0,      pi]);  
L5.qlim = [-pi/2, pi/2];

% WRIST
%  DH parameters for 5T6 (Plantar-dorsal flexion)
L6 = Link([0,       0,      0,      pi/2,   0,      pi/2]);
L6.qlim = [-pi/2, pi/2];
%  DH parameters for 6T7 (Abduction-Adduction)
L7 = Link([0,       0,      l_a2,      0,      0,      0]);
L7.qlim = [-pi/2, pi/2];

% Construction of an object belonging to the class robot
left_arm=SerialLink([L1,L2,L3,L4,L5,L6,L7],'name', 'left_arm');
% Location of the base reference frame
left_arm.base=transl(0,l_shoulder,l_hip_to_shoulder)*trotz(-pi/2);
% Visualization of the robot in the joint position given by qr
left_arm_qr=[0, 0, 0, 0, 0, 0, 0];
left_arm.plot(left_arm_qr, 'nobase')
% Visualization of the robot and manual guidance of the different joints, starting from
% the initial joint position qr
teach(left_arm, left_arm_qr, 'nobase')


%% RIGHT ARM

%       [theta,     D,      A,      alpha,  sigma,  offset]

% SHOULDER
% DH parameters for 0T1 (Flexion-Extension)
L1 = Link([0,       0,      0,      pi/2,   0,      pi/2]); 
L1.qlim = [-pi/2, pi];
%  DH parameters for 1T2 (Abduction-Adduction)
L2 = Link([0,       0,      0,      pi/2,   0,      pi/2]);
L2.qlim = [-pi/2, pi];
%  DH parameters for 2T3 (Circumduction)
L3 = Link([0,       0,      l_a1,      pi/2,   0,      pi]);
L3.qlim = [-pi, pi/2];

% ELBOW
% DH parameters for 3T4 (Flexion-Extension)
L4 = Link([0,       0,      0,      pi/2,   0,      pi]);  
L4.qlim = [0, pi];
% DH parameters for 4T5 (Pronation-Supination)
L5 = Link([0,       0,      l_a1,      pi/2,   0,      pi]);  
L5.qlim = [-pi/2, pi/2];

% WRIST
%  DH parameters for 5T6 (Plantar-dorsal flexion)
L6 = Link([0,       0,      0,      pi/2,   0,      pi/2]);
L6.qlim = [-pi/2, pi/2];
%  DH parameters for 6T7 (Abduction-Adduction)
L7 = Link([0,       0,      l_a2,      0,      0,      0]);
L7.qlim = [-pi/2, pi/2];

% Construction of an object belonging to the class robot
right_arm=SerialLink([L1,L2,L3,L4,L5,L6,L7],'name', 'right_arm');
% Location of the base reference frame
right_arm.base=transl(0,0,l_hip_to_shoulder)*transl(0,-l_shoulder,0)*trotz(-pi/2);
% Visualization of the robot in the joint position given by qr
right_arm_qr=[0, 0, 0, 0, 0, 0, 0];

right_arm.plot(right_arm_qr, 'nobase');
% Visualization of the robot and manual guidance of the different joints, starting from
% the initial joint position qr

teach(right_arm, right_arm_qr, 'nobase')


% Plot constant trunk, hip, shoulders and neck
plot3([0 0], [0 0], [0 l_hip_to_shoulder], 'k-', 'LineWidth', 2);
plot3([0 0], [l_hip -l_hip], [0 0], 'k-', 'LineWidth', 2);
plot3([0 0], [l_shoulder -l_shoulder], [l_hip_to_shoulder l_hip_to_shoulder], 'k-', 'LineWidth', 2);

hold off;

%% Movements definition
sample_time = 0.4;

vel_left_leg = [];
vel_right_leg = [];
vel_left_arm = [];
vel_right_arm = [];

cartesians_left_leg = [];
cartesians_right_leg = [];
cartesians_left_arm = [];
cartesians_right_arm = [];

%% 1
n_interp = 10; 

% Left leg
q_left_leg_initial = left_leg_qr;
q_left_leg_final = [5.4*pi/180, -8.1*pi/180, 25.2*pi/180, 304*pi/180, 24*pi/180, 7.2*pi/180];
q_left_leg_interpolation = jtraj(q_left_leg_initial, q_left_leg_final, n_interp);  

position_differences = diff(q_left_leg_interpolation, 1, 1);  % Computes the difference between consecutive rows
    
vel_left_leg = [zeros(1,6); position_differences / sample_time];

T_left_leg_initial = fkine(left_leg, q_left_leg_initial);
T_left_leg_final = fkine(left_leg, q_left_leg_final);

cartesians_left_leg = [cartesians_left_leg, ctraj(T_left_leg_initial, T_left_leg_final, n_interp)];
T_left_leg = [T_left_leg_initial; T_left_leg_final];


% Right leg
q_right_leg_initial = right_leg_qr;
q_right_leg_final = [5.4*pi/180, 8.1*pi/180, 25.2*pi/180, 304*pi/180, 24*pi/180, 7.2*pi/180];
q_right_leg_interpolation = jtraj(q_right_leg_initial, q_right_leg_final, n_interp);  

position_differences = diff(q_right_leg_interpolation, 1, 1);  % Computes the difference between consecutive rows
    
vel_right_leg = [zeros(1,6); position_differences / sample_time];

T_right_leg_initial = fkine(right_leg, q_right_leg_initial);
T_right_leg_final = fkine(right_leg, q_right_leg_final);

cartesians_right_leg = [cartesians_right_leg, ctraj(T_right_leg_initial, T_right_leg_final, n_interp)];
T_right_leg = [T_right_leg_initial; T_right_leg_final];


% Left arm
q_left_arm_initial = left_arm_qr;
q_left_arm_final = [-21.6*pi/180, 23.4*pi/180, -5.4*pi/180, 25.2*pi/180, -10.8*pi/180, -43.2*pi/180, -25.2*pi/180];
q_left_arm_interpolation = jtraj(q_left_arm_initial, q_left_arm_final, n_interp);  

position_differences = diff(q_left_arm_interpolation, 1, 1);  % Computes the difference between consecutive rows
    
vel_left_arm = [zeros(1,7); position_differences / sample_time];

T_left_arm_initial = fkine(left_arm, q_left_arm_initial);
T_left_arm_final = fkine(left_arm, q_left_arm_final);

cartesians_left_arm = [cartesians_left_arm, ctraj(T_left_arm_initial, T_left_arm_final, n_interp)];
T_left_arm = [T_left_arm_initial; T_left_arm_final];


% Right arm
q_right_arm_initial = right_arm_qr;
q_right_arm_final = [21.6*pi/180, 23.4*pi/180, 5.4*pi/180, 25.2*pi/180, 10.8*pi/180, -43.2*pi/180, 25.2*pi/180];
q_right_arm_interpolation = jtraj(q_right_arm_initial, q_right_arm_final, n_interp);  

position_differences = diff(q_right_arm_interpolation, 1, 1);  % Computes the difference between consecutive rows
    
vel_right_arm = [zeros(1,7); position_differences / sample_time];

T_right_arm_initial = fkine(right_arm, q_right_arm_initial);
T_right_arm_final = fkine(right_arm, q_right_arm_final);

cartesians_right_arm = [cartesians_right_arm, ctraj(T_right_arm_initial, T_right_arm_final, n_interp)];
T_right_arm = [T_right_arm_initial; T_right_arm_final];


%% 2
n_interp = 12; 

% Left leg
q_left_leg_initial = q_left_leg_final;
q_left_leg_final = [-9.0*pi/180, -5.4*pi/180, 18.0*pi/180, 317.0*pi/180, 0.0*pi/180, 10.8*pi/180];
[q_left_leg_interpolation, vel_left_leg, T_left_leg_final, cartesians_left_leg] = processLimb(left_leg, q_left_leg_initial, q_left_leg_final, q_left_leg_interpolation, vel_left_leg, cartesians_left_leg, n_interp, sample_time);
T_left_leg = [T_left_leg; T_left_leg_final];


% Right leg
q_right_leg_initial = q_right_leg_final;
q_right_leg_final = [-10.8*pi/180, 8.1*pi/180, 7.2*pi/180, 347.0*pi/180, 0*pi/180, 10.8*pi/180];
[q_right_leg_interpolation, vel_right_leg, T_right_leg_final, cartesians_right_leg] = processLimb(right_leg, q_right_leg_initial, q_right_leg_final, q_right_leg_interpolation, vel_right_leg, cartesians_right_leg, n_interp, sample_time);
T_right_leg = [T_right_leg; T_right_leg_final];


% Left arm
q_left_arm_initial = q_left_arm_final;
q_left_arm_final = [-90.0*pi/180, 59.4*pi/180, 21.6*pi/180, 10.8*pi/180, 0.0*pi/180, -7.2*pi/180, -23.4*pi/180];
[q_left_arm_interpolation, vel_left_arm, T_left_arm_final, cartesians_left_arm] = processLimb(left_arm, q_left_arm_initial, q_left_arm_final, q_left_arm_interpolation, vel_left_arm, cartesians_left_arm, n_interp, sample_time);
T_left_arm = [T_left_arm; T_left_arm_final];


% Right arm
q_right_arm_initial = q_right_arm_final;
q_right_arm_final = [-35.1*pi/180, -11.7*pi/180, -2.7*pi/180, 113.0*pi/180, 12.6*pi/180, -32.4*pi/180, 3.6*pi/180];
[q_right_arm_interpolation, vel_right_arm, T_right_arm_final, cartesians_right_arm] = processLimb(right_arm, q_right_arm_initial, q_right_arm_final, q_right_arm_interpolation, vel_right_arm, cartesians_right_arm, n_interp, sample_time);
T_right_arm = [T_right_arm; T_right_arm_final];


%% 3
n_interp = 6; 

% Left leg
q_left_leg_initial = q_left_leg_final;
q_left_leg_final = [-3.6*pi/180, -2.7*pi/180, 14.4*pi/180, 324.0*pi/180, 0.0*pi/180, 5.4*pi/180];
[q_left_leg_interpolation, vel_left_leg, T_left_leg_final, cartesians_left_leg] = processLimb(left_leg, q_left_leg_initial, q_left_leg_final, q_left_leg_interpolation, vel_left_leg, cartesians_left_leg, n_interp, sample_time);
T_left_leg = [T_left_leg; T_left_leg_final];


% Right leg
q_right_leg_initial = q_right_leg_final;
q_right_leg_final = [1.8*pi/180, 10.8*pi/180, 7.2*pi/180, 349.0*pi/180, -2.4*pi/180, 7.2*pi/180];
[q_right_leg_interpolation, vel_right_leg, T_right_leg_final, cartesians_right_leg] = processLimb(right_leg, q_right_leg_initial, q_right_leg_final, q_right_leg_interpolation, vel_right_leg, cartesians_right_leg, n_interp, sample_time);
T_right_leg = [T_right_leg; T_right_leg_final];


% Left arm
q_left_arm_initial = q_left_arm_final;
q_left_arm_final = [-90.0*pi/180, 19.8*pi/180, 37.8*pi/180, 5.4*pi/180, -5.4*pi/180, -14.4*pi/180, -25.2*pi/180];
[q_left_arm_interpolation, vel_left_arm, T_left_arm_final, cartesians_left_arm] = processLimb(left_arm, q_left_arm_initial, q_left_arm_final, q_left_arm_interpolation, vel_left_arm, cartesians_left_arm, n_interp, sample_time);
T_left_arm = [T_left_arm; T_left_arm_final];


% Right arm
q_right_arm_initial = q_right_arm_final;
q_right_arm_final = [2.7*pi/180, 10.8*pi/180, 0.0*pi/180, 46.8*pi/180, 16.2*pi/180, -27.0*pi/180, -19.8*pi/180];
[q_right_arm_interpolation, vel_right_arm, T_right_arm_final, cartesians_right_arm] = processLimb(right_arm, q_right_arm_initial, q_right_arm_final, q_right_arm_interpolation, vel_right_arm, cartesians_right_arm, n_interp, sample_time);
T_right_arm = [T_right_arm; T_right_arm_final];


%% 4
n_interp = 6; 

% Left leg
q_left_leg_initial = q_left_leg_final;
q_left_leg_final = [3.6*pi/180, -2.7*pi/180, 10.8*pi/180, 336.6*pi/180, 0.0*pi/180, 1.8*pi/180];
[q_left_leg_interpolation, vel_left_leg, T_left_leg_final, cartesians_left_leg] = processLimb(left_leg, q_left_leg_initial, q_left_leg_final, q_left_leg_interpolation, vel_left_leg, cartesians_left_leg, n_interp, sample_time);
T_left_leg = [T_left_leg; T_left_leg_final];


% Right leg
q_right_leg_initial = q_right_leg_final;
q_right_leg_final = [16.2*pi/180, 10.8*pi/180, 3.6*pi/180, 349.2*pi/180, -4.8*pi/180, 3.6*pi/180];
[q_right_leg_interpolation, vel_right_leg, T_right_leg_final, cartesians_right_leg] = processLimb(right_leg, q_right_leg_initial, q_right_leg_final, q_right_leg_interpolation, vel_right_leg, cartesians_right_leg, n_interp, sample_time);
T_right_leg = [T_right_leg; T_right_leg_final];


% Left arm
q_left_arm_initial = q_left_arm_final;
q_left_arm_final = [-90.0*pi/180, 7.2*pi/180, 35.1*pi/180, 1.8*pi/180, -10.8*pi/180, -14.4*pi/180, -45.0*pi/180];
[q_left_arm_interpolation, vel_left_arm, T_left_arm_final, cartesians_left_arm] = processLimb(left_arm, q_left_arm_initial, q_left_arm_final, q_left_arm_interpolation, vel_left_arm, cartesians_left_arm, n_interp, sample_time);
T_left_arm = [T_left_arm; T_left_arm_final];


% Right arm
q_right_arm_initial = q_right_arm_final;
q_right_arm_final = [29.7*pi/180, 32.4*pi/180, 2.7*pi/180, 5.4*pi/180, 5.4*pi/180, -32.4*pi/180, -0.0*pi/180];
[q_right_arm_interpolation, vel_right_arm, T_right_arm_final, cartesians_right_arm] = processLimb(right_arm, q_right_arm_initial, q_right_arm_final, q_right_arm_interpolation, vel_right_arm, cartesians_right_arm, n_interp, sample_time);
T_right_arm = [T_right_arm; T_right_arm_final];


%% 5
n_interp = 8; 

% Left leg
q_left_leg_initial = q_left_leg_final;
q_left_leg_final = [5.4*pi/180, -5.4*pi/180, 21.6*pi/180, 329.4*pi/180, -1.2*pi/180, -3.6*pi/180];
[q_left_leg_interpolation, vel_left_leg, T_left_leg_final, cartesians_left_leg] = processLimb(left_leg, q_left_leg_initial, q_left_leg_final, q_left_leg_interpolation, vel_left_leg, cartesians_left_leg, n_interp, sample_time);
T_left_leg = [T_left_leg; T_left_leg_final];


% Right leg
q_right_leg_initial = q_right_leg_final;
q_right_leg_final = [57.6*pi/180, 8.1*pi/180, 18.0*pi/180, 334.8*pi/180, -40.8*pi/180, -16.2*pi/180];
[q_right_leg_interpolation, vel_right_leg, T_right_leg_final, cartesians_right_leg] = processLimb(right_leg, q_right_leg_initial, q_right_leg_final, q_right_leg_interpolation, vel_right_leg, cartesians_right_leg, n_interp, sample_time);
T_right_leg = [T_right_leg; T_right_leg_final];


% Left arm
q_left_arm_initial = q_left_arm_final;
q_left_arm_final = [26.1*pi/180, -29.7*pi/180, 21.6*pi/180, 113.0*pi/180, -1.8*pi/180, -50.4*pi/180, -7.2*pi/180];
[q_left_arm_interpolation, vel_left_arm, T_left_arm_final, cartesians_left_arm] = processLimb(left_arm, q_left_arm_initial, q_left_arm_final, q_left_arm_interpolation, vel_left_arm, cartesians_left_arm, n_interp, sample_time);
T_left_arm = [T_left_arm; T_left_arm_final];


% Right arm
q_right_arm_initial = q_right_arm_final;
q_right_arm_final = [48.6*pi/180, 54.9*pi/180, 21.6*pi/180, 9.0*pi/180, -1.8*pi/180, -28.8*pi/180, 12.6*pi/180];
[q_right_arm_interpolation, vel_right_arm, T_right_arm_final, cartesians_right_arm] = processLimb(right_arm, q_right_arm_initial, q_right_arm_final, q_right_arm_interpolation, vel_right_arm, cartesians_right_arm, n_interp, sample_time);
T_right_arm = [T_right_arm; T_right_arm_final];


%%  Animation plot
len_leg = size(q_left_leg_interpolation);
len_arm = size(q_left_arm_interpolation);
anim1 = Animate('golf.mp4');

tic;
for i = 1:max(len_leg(1),len_arm(1))
    if i <= len_leg(1)
        left_leg.animate(q_left_leg_interpolation(i,:));
        right_leg.animate(q_right_leg_interpolation(i,:));
    end
    if i <= len_arm(1)
        left_arm.animate(q_left_arm_interpolation(i,:));
        right_arm.animate(q_right_arm_interpolation(i,:));
    end
    anim1.add()
end
anim1.close()

elapsedTime = toc;
fprintf('Total execution time: %.2f seconds\n', elapsedTime);

%% Cartesian positions FW model

% arms
id_fig = 2;

% Determine the number of elements
n = length(cartesians_left_arm);
timeVector = linspace(0, n*sample_time, n);
% 
% right_curve = getCurveInterpolation('left');
% left_curve = getCurveInterpolation('left');
% disp(length(right_curve))
% for i = 1:length(right_curve)
%     cartesians_right_arm(i+20) = right_curve(i);
%     cartesians_left_arm(i+20) = left_curve(i);
% end

% Preallocate the arrays
pos_left_arm = zeros(n, length(cartesians_left_arm(1).t));
pos_right_arm = zeros(n, length(cartesians_right_arm(1).t));

% Fill the preallocated arrays
for i = 1:n
    pos_left_arm(i, :) = cartesians_left_arm(i).t';
    pos_right_arm(i, :) = cartesians_right_arm(i).t';
end


figure(id_fig);
id_fig = id_fig + 1;
plot(timeVector, pos_right_arm, '-');
grid;
title("Position Right Arm");
legend('X', 'Y', 'Z');
xlabel('Time (s)');
ylabel('Position (m)');

figure(id_fig);
id_fig = id_fig + 1;
plot(timeVector, pos_left_arm, '-');
grid;
title("Position Left Arm");
legend('X', 'Y', 'Z');
xlabel('Time (s)');
ylabel('Position (m)');

%Legs

% Determine the number of elements
n = length(cartesians_left_leg);

% Preallocate the arrays
pos_left_leg = zeros(n, length(cartesians_left_leg(1).t));
pos_right_leg = zeros(n, length(cartesians_right_leg(1).t));

% Fill the preallocated arrays
for i = 1:n
    pos_left_leg(i, :) = cartesians_left_leg(i).t';
    pos_right_leg(i, :) = cartesians_right_leg(i).t';
end

figure(id_fig);grid;
id_fig = id_fig +1;
plot(timeVector, pos_right_leg,'-');
grid;
title("Position Right Leg")
legend('X','Y','Z');
xlabel('Time (s)');
ylabel('Position (m)');

figure(id_fig);grid;
id_fig = id_fig +1;
plot(timeVector, pos_left_leg,'-');
grid;
title("Position Left Leg")
legend('X','Y','Z');
xlabel('Time (s)');
ylabel('Position (m)');

figure(id_fig);
id_fig = id_fig +1;
plot3(pos_left_leg(:,1), pos_left_leg(:,2), pos_left_leg(:,3), 'g', ...
    pos_right_leg(:,1), pos_right_leg(:,2), pos_right_leg(:,3), 'k', ...
    pos_right_arm(:,1), pos_right_arm(:,2), pos_right_arm(:,3), 'r', ...
    pos_left_arm(:,1), pos_left_arm(:,2), pos_left_arm(:,3)), 'b';
grid;
title("Limb trajectories 3D")
legend('LL', 'RL', 'RA', 'LA');
xlabel('X (m)');
ylabel('Y (m)');
zlabel('Z (m)');

%% Velocities

% Left leg
figure(id_fig);
id_fig = id_fig +1;
plot(timeVector, vel_left_leg);
axis([0 inf -1.5 1.5]);
grid;
title('Radian/s vel left leg joints');
legend('q1','q2','q3','q4','q5','q6');
xlabel('Time (s)');
ylabel('Velocity (rad/s)');


% Right leg
figure(id_fig);
id_fig = id_fig +1;
plot(timeVector, vel_right_leg);
axis([0 inf -1.5 1.5]);
grid;
title('Radian/s vel right leg joints');
legend('q1','q2','q3','q4','q5','q6');
xlabel('Time (s)');
ylabel('Velocity (rad/s)');


% Left arm
figure(id_fig);
id_fig = id_fig +1;
plot(timeVector, vel_left_arm);
axis([0 inf -1.5 1.5]);
grid;
title('Radian/s vel left arm joints');
legend('q1','q2','q3','q4','q5','q6','q7');
xlabel('Time (s)');
ylabel('Velocity (rad/s)');


% Right arm
figure(id_fig);
id_fig = id_fig +1;
plot(timeVector, vel_right_arm);
axis([0 inf -1.5 1.5]);
grid;
title('Radian/s vel right arm joints');
legend('q1','q2','q3','q4','q5','q6','q7');
xlabel('Time (s)');
ylabel('Velocity (rad/s)');

%% Inverse kinematics
% Cartesian

len_leg = size(cartesians_left_leg);
len_arm = size(cartesians_left_arm);
anim = Animate('golf_c.mp4');

% q_left_leg_ini = q_left_leg_interpolation;
% q_right_leg_ini = q_right_leg_interpolation;
% q_left_arm_ini = q_left_arm_interpolation;
% q_right_arm_ini = q_right_arm_interpolation;

q_left_leg = [];
q_right_leg = [];
q_left_arm = [];
q_right_arm = [];

for i = 1:max(len_leg(2),len_arm(2))
    if i <= len_leg(2)
        
        if i == 1
            q_left_leg_ini = left_leg_qr; % Initial guess for the first pose
            q_right_leg_ini = right_leg_qr;
        else
            q_left_leg_ini = q_left_leg_inverse; % Use the previous solution as the initial guess
            q_right_leg_ini = q_right_leg_inverse;
        end

        fprintf("Left leg step %d\n", i);

        %q_left_leg_inverse = inverseKinematics(left_leg, cartesians_left_leg(i), q_left_leg_ini, q_left_leg_interpolation(i,:), left_leg_qr);
        q_left_leg_inverse = inverseKinematics2(left_leg, cartesians_left_leg(i), q_left_leg_ini, q_left_leg_interpolation(i,:));

        if ~isempty(q_left_leg_inverse)
            left_leg.animate(q_left_leg_inverse);
        else     
            q_left_leg_inverse = left_leg_qr;
        end
        
        q_left_leg = [q_left_leg; q_left_leg_inverse];

        fprintf("Right leg step %d\n", i);

        % q_right_leg_inverse = inverseKinematics(right_leg, cartesians_right_leg(i), q_right_leg_ini, q_right_leg_interpolation(i,:), right_leg_qr);
        q_right_leg_inverse = inverseKinematics2(right_leg, cartesians_right_leg(i), q_right_leg_ini, q_right_leg_interpolation(i,:));

        if ~isempty(q_right_leg_inverse)
            right_leg.animate(q_right_leg_inverse);
        else      
            q_right_leg_inverse = right_leg_qr;
        end
        
        q_right_leg = [q_right_leg; q_right_leg_inverse];

    end
    if i <= len_arm(2)

        if i == 1
            q_left_arm_ini = left_arm_qr; % Initial guess for the first pose
            q_right_arm_ini = right_arm_qr;
        else
            q_left_arm_ini = q_left_arm_inverse; % Use the previous solution as the initial guess
            q_right_arm_ini = q_right_arm_inverse;
        end

        fprintf("Left arm step %d\n", i);
       
        % q_left_arm_inverse = inverseKinematics(left_arm, cartesians_left_arm(i), q_left_arm_ini, q_left_arm_interpolation(i,:),left_arm_qr);
        q_left_arm_inverse = inverseKinematics2(left_arm, cartesians_left_arm(i), q_left_arm_ini, q_left_arm_interpolation(i,:));

        if ~isempty(q_left_arm_inverse)
            left_arm.animate(q_left_arm_inverse);
        else
            q_left_arm_inverse = left_arm_qr;
        end
        
        q_left_arm = [q_left_arm; q_left_arm_inverse];

        fprintf("Right arm step %d\n", i);
       
        % q_right_arm_inverse = inverseKinematics(right_arm, cartesians_right_arm(i), q_right_arm_ini, q_right_arm_interpolation(i,:), right_arm_qr);
        q_right_arm_inverse = inverseKinematics2(right_arm, cartesians_right_arm(i), q_right_arm_ini, q_right_arm_interpolation(i,:));

        if ~isempty(q_right_arm_inverse)
            right_arm.animate(q_right_arm_inverse);
        else
            q_right_arm_inverse = right_arm_qr;
        end

        q_right_arm = [q_right_arm; q_right_arm_inverse];

    end
    anim.add()
end
anim.close()

% Left leg
figure(id_fig);
id_fig = id_fig +1;
plot(timeVector, q_left_leg);
grid;
title('Inverse joint coordinates left leg joints');
legend('q1','q2','q3','q4','q5','q6');
xlabel('Time (s)');
ylabel('Angle (rad)');

figure(id_fig);
id_fig = id_fig +1;
plot(timeVector, q_left_leg_interpolation);
grid;
title('Interpolated joint coordinates left leg joints');
legend('q1','q2','q3','q4','q5','q6');
xlabel('Time (s)');
ylabel('Angle (rad)');


% Left leg
figure(id_fig);
id_fig = id_fig +1;
plot(timeVector, q_right_leg);
grid;
title('Inverse joint coordinates right leg joints');
legend('q1','q2','q3','q4','q5','q6');
xlabel('Time (s)');
ylabel('Angle (rad)');

figure(id_fig);
id_fig = id_fig +1;
plot(timeVector, q_right_leg_interpolation);
grid;
title('Interpolated joint coordinates right leg joints');
legend('q1','q2','q3','q4','q5','q6');
xlabel('Time (s)');
ylabel('Angle (rad)');


% Left arm
figure(id_fig);
id_fig = id_fig +1;
plot(timeVector, q_left_arm);
grid;
title('Inverse joint coordinates left arm joints');
legend('q1','q2','q3','q4','q5','q6','q7');
xlabel('Time (s)');
ylabel('Angle (rad)');

figure(id_fig);
id_fig = id_fig +1;
plot(timeVector, q_left_arm_interpolation);
grid;
title('Interpolated joint coordinates left arm joints');
legend('q1','q2','q3','q4','q5','q6','q7');
xlabel('Time (s)');
ylabel('Angle (rad)');


% Right arm
figure(id_fig);
id_fig = id_fig +1;
plot(timeVector, q_right_arm);
grid;
title('Inverse joint coordinates right arm joints');
legend('q1','q2','q3','q4','q5','q6','q7');
xlabel('Time (s)');
ylabel('Angle (rad)');

figure(id_fig);
id_fig = id_fig +1;
plot(timeVector, q_right_arm_interpolation);
grid;
title('Interpolated joint coordinates right arm joints');
legend('q1','q2','q3','q4','q5','q6','q7');
xlabel('Time (s)');
ylabel('Angle (rad)');
