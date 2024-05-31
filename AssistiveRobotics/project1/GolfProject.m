clear;
close all;

%%  Structure with 2 joints exercise
l_1 = 5;
l_2 = 5;

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
L3 = Link([0,       0,      2,      0,      0,      -pi]);
L3.qlim = [-pi, pi];

% KNEE
% DH parameters for 3T4 (Flexion-Extension)
L4 = Link([0,       0,      2,      0,      0,      0]);  
L4.qlim = [pi, 2*pi];

% ANKLE
%  DH parameters for 4T5 (Plantar-dorsal flexion)
L5 = Link([0,       0,      0,      -pi/2,  0,      pi/2]);
L5.qlim = [-pi/2, 30*pi/180];
%  DH parameters for 5T6 (Abduction-Adduction)
L6 = Link([0,       0,      1,      0,      0,      0]);
L6.qlim = [-pi/2, pi/2];

% Construction of an object belonging to the class robot
left_leg=SerialLink([L1,L2,L3,L4,L5,L6],'name', 'left_leg');
% Location of the base reference frame
left_leg.base=transl(0,1,0)*trotz(-pi/2);
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
L3 = Link([0,       0,      2,      0,      0,      -pi]);
L3.qlim = [-pi, pi];

% KNEE
% DH parameters for 3T4 (Flexion-Extension)
L4 = Link([0,       0,      2,      0,      0,      0]);  
L4.qlim = [pi, 2*pi];

% ANKLE
%  DH parameters for 4T5 (Plantar-dorsal flexion)
L5 = Link([0,       0,      0,      -pi/2,  0,      pi/2]);
L5.qlim = [-pi/2, 30*pi/180];
%  DH parameters for 5T6 (Abduction-Adduction)
L6 = Link([0,       0,      1,      0,      0,      0]);
L6.qlim = [-pi/2, pi/2];

% Construction of an object belonging to the class robot
right_leg=SerialLink([L1,L2,L3,L4,L5,L6],'name', 'right_leg');
% Location of the base reference frame
right_leg.base=transl(0,-1,0)*trotz(-pi/2);
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
L3 = Link([0,       0,      2,      pi/2,   0,      pi]);
L3.qlim = [-pi, pi/2];

% ELBOW
% DH parameters for 3T4 (Flexion-Extension)
L4 = Link([0,       0,      0,      pi/2,   0,      pi]);  
L4.qlim = [0, pi];
% DH parameters for 4T5 (Pronation-Supination)
L5 = Link([0,       0,      2,      pi/2,   0,      pi]);  
L5.qlim = [-pi/2, pi/2];

% WRIST
%  DH parameters for 5T6 (Plantar-dorsal flexion)
L6 = Link([0,       0,      0,      pi/2,   0,      pi/2]);
L6.qlim = [-pi/2, pi/2];
%  DH parameters for 6T7 (Abduction-Adduction)
L7 = Link([0,       0,      1,      0,      0,      0]);
L7.qlim = [-pi/2, pi/2];

% Construction of an object belonging to the class robot
left_arm=SerialLink([L1,L2,L3,L4,L5,L6,L7],'name', 'left_arm');
% Location of the base reference frame
left_arm.base=transl(0,1.5,5)*trotz(-pi/2);
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
L3 = Link([0,       0,      2,      pi/2,   0,      pi]);
L3.qlim = [-pi, pi/2];

% ELBOW
% DH parameters for 3T4 (Flexion-Extension)
L4 = Link([0,       0,      0,      pi/2,   0,      pi]);  
L4.qlim = [0, pi];
% DH parameters for 4T5 (Pronation-Supination)
L5 = Link([0,       0,      2,      pi/2,   0,      pi]);  
L5.qlim = [-pi/2, pi/2];

% WRIST
%  DH parameters for 5T6 (Plantar-dorsal flexion)
L6 = Link([0,       0,      0,      pi/2,   0,      pi/2]);
L6.qlim = [-pi/2, pi/2];
%  DH parameters for 6T7 (Abduction-Adduction)
L7 = Link([0,       0,      1,      0,      0,      0]);
L7.qlim = [-pi/2, pi/2];

% Construction of an object belonging to the class robot
right_arm=SerialLink([L1,L2,L3,L4,L5,L6,L7],'name', 'right_arm');
% Location of the base reference frame
right_arm.base=transl(0,0,5)*transl(0,-1.5,0)*trotz(-pi/2);
% Visualization of the robot in the joint position given by qr
right_arm_qr=[0, 0, 0, 0, 0, 0, 0];

right_arm.plot(right_arm_qr, 'nobase');
% Visualization of the robot and manual guidance of the different joints, starting from
% the initial joint position qr

teach(right_arm, right_arm_qr, 'nobase')


% Plot constant trunk, hip, shoulders and neck
plot3([0 0], [0 0], [0 5], 'k-', 'LineWidth', 2);
plot3([0 0], [1 -1], [0 0], 'k-', 'LineWidth', 2);
plot3([0 0], [1.5 -1.5], [5 5], 'k-', 'LineWidth', 2);

hold off;

%% Movements definition
sample_time = 5;

vel_left_leg = [];
vel_right_leg = [];
vel_left_arm = [];
vel_right_arm = [];

cartesians_left_leg = [];
cartesians_right_leg = [];
cartesians_left_arm = [];
cartesians_right_arm = [];

%% 1
t = 15; 

% Left leg
q_left_leg_initial = left_leg_qr;
q_left_leg_final = [5.4*pi/180, -8.1*pi/180, 25.2*pi/180, 304*pi/180, 24*pi/180, 7.2*pi/180];
q_left_leg_interpolation = jtraj(q_left_leg_initial, q_left_leg_final, t);  

vel_left_leg = [vel_left_leg; repmat((q_left_leg_initial - q_left_leg_final) / t * sample_time, t, 1)];

T_left_leg_initial = fkine(left_leg, q_left_leg_initial);
T_left_leg_final = fkine(left_leg, q_left_leg_final);

cartesians_left_leg = [cartesians_left_leg, ctraj(T_left_leg_initial, T_left_leg_final, t)];
T_left_leg = [T_left_leg_initial; T_left_leg_final];


% Right leg
q_right_leg_initial = right_leg_qr;
q_right_leg_final = [5.4*pi/180, 8.1*pi/180, 25.2*pi/180, 304*pi/180, 24*pi/180, 7.2*pi/180];
q_right_leg_interpolation = jtraj(q_right_leg_initial, q_right_leg_final, t);  

vel_right_leg = [vel_right_leg; repmat((q_right_leg_initial - q_right_leg_final) / t * sample_time, t, 1)];

T_right_leg_initial = fkine(right_leg, q_right_leg_initial);
T_right_leg_final = fkine(right_leg, q_right_leg_final);

cartesians_right_leg = [cartesians_right_leg, ctraj(T_right_leg_initial, T_right_leg_final, t)];
T_right_leg = [T_right_leg_initial; T_right_leg_final];


% Left arm
q_left_arm_initial = left_arm_qr;
q_left_arm_final = [-21.6*pi/180, 23.4*pi/180, -5.4*pi/180, 25.2*pi/180, -10.8*pi/180, -43.2*pi/180, -25.2*pi/180];
q_left_arm_interpolation = jtraj(q_left_arm_initial, q_left_arm_final, t);  

vel_left_arm = [vel_left_arm; repmat((q_left_arm_initial - q_left_arm_final) / t * sample_time, t, 1)];

T_left_arm_initial = fkine(left_arm, q_left_arm_initial);
T_left_arm_final = fkine(left_arm, q_left_arm_final);

cartesians_left_arm = [cartesians_left_arm, ctraj(T_left_arm_initial, T_left_arm_final, t)];
T_left_arm = [T_left_arm_initial; T_left_arm_final];


% Right arm
q_right_arm_initial = right_arm_qr;
q_right_arm_final = [21.6*pi/180, 23.4*pi/180, 5.4*pi/180, 25.2*pi/180, 10.8*pi/180, -43.2*pi/180, 25.2*pi/180];
q_right_arm_interpolation = jtraj(q_right_arm_initial, q_right_arm_final, t);  

vel_right_arm = [vel_right_arm; repmat((q_right_arm_initial - q_right_arm_final) / t * sample_time, t, 1)];

T_right_arm_initial = fkine(right_arm, q_right_arm_initial);
T_right_arm_final = fkine(right_arm, q_right_arm_final);

cartesians_right_arm = [cartesians_right_arm, ctraj(T_right_arm_initial, T_right_arm_final, t)];
T_right_arm = [T_right_arm_initial; T_right_arm_final];


%% 2
t = 8; 

% Left leg
q_left_leg_initial = q_left_leg_final;
q_left_leg_final = [-9.0*pi/180, -5.4*pi/180, 18.0*pi/180, 317.0*pi/180, 0.0*pi/180, 10.8*pi/180];
q_left_leg_interpolation = [q_left_leg_interpolation; jtraj(q_left_leg_initial, q_left_leg_final, t)];  

vel_left_leg = [vel_left_leg; repmat((q_left_leg_initial - q_left_leg_final) / t * sample_time, t, 1)];

T_left_leg_initial = T_left_leg_final;
T_left_leg_final = fkine(left_leg, q_left_leg_final);

cartesians_left_leg = [cartesians_left_leg, ctraj(T_left_leg_initial, T_left_leg_final, t)];
T_left_leg = [T_left_leg; T_left_leg_final];


% Right leg
q_right_leg_initial = q_right_leg_final;
q_right_leg_final = [-10.8*pi/180, 8.1*pi/180, 7.2*pi/180, 347.0*pi/180, 0*pi/180, 10.8*pi/180];
q_right_leg_interpolation = [q_right_leg_interpolation; jtraj(q_right_leg_initial, q_right_leg_final, t)];  

vel_right_leg = [vel_right_leg; repmat((q_right_leg_initial - q_right_leg_final) / t * sample_time, t, 1)];

T_right_leg_initial = T_right_leg_final;
T_right_leg_final = fkine(right_leg, q_right_leg_final);

cartesians_right_leg = [cartesians_right_leg, ctraj(T_right_leg_initial, T_right_leg_final, t)];
T_right_leg = [T_right_leg; T_right_leg_final];


% Left arm
q_left_arm_initial = q_left_arm_final;
q_left_arm_final = [-90.0*pi/180, 59.4*pi/180, 21.6*pi/180, 10.8*pi/180, 0.0*pi/180, -7.2*pi/180, -23.4*pi/180];
q_left_arm_interpolation = [q_left_arm_interpolation; jtraj(q_left_arm_initial, q_left_arm_final, t)];  

vel_left_arm = [vel_left_arm; repmat((q_left_arm_initial - q_left_arm_final) / t * sample_time, t, 1)];

T_left_arm_initial = T_left_arm_final;
T_left_arm_final = fkine(left_arm, q_left_arm_final);

cartesians_left_arm = [cartesians_left_arm, ctraj(T_left_arm_initial, T_left_arm_final, t)];
T_left_arm = [T_left_arm; T_left_arm_final];


% Right arm
q_right_arm_initial = q_right_arm_final;
q_right_arm_final = [-35.1*pi/180, -11.7*pi/180, -2.7*pi/180, 113.0*pi/180, 12.6*pi/180, -32.4*pi/180, 3.6*pi/180];
q_right_arm_interpolation = [q_right_arm_interpolation; jtraj(q_right_arm_initial, q_right_arm_final, t)];  

vel_right_arm = [vel_right_arm; repmat((q_right_arm_initial - q_right_arm_final) / t * sample_time, t, 1)];

T_right_arm_initial = T_right_arm_final;
T_right_arm_final = fkine(right_arm, q_right_arm_final);

cartesians_right_arm = [cartesians_right_arm, ctraj(T_right_arm_initial, T_right_arm_final, t)];
T_right_arm = [T_right_arm; T_right_arm_final];


%% 3
t = 5; 

% Left leg
q_left_leg_initial = q_left_leg_final;
q_left_leg_final = [-3.6*pi/180, -2.7*pi/180, 14.4*pi/180, 324.0*pi/180, 0.0*pi/180, 5.4*pi/180];
q_left_leg_interpolation = [q_left_leg_interpolation; jtraj(q_left_leg_initial, q_left_leg_final, t)];  

vel_left_leg = [vel_left_leg; repmat((q_left_leg_initial - q_left_leg_final) / t * sample_time, t, 1)];

T_left_leg_initial = T_left_leg_final;
T_left_leg_final = fkine(left_leg, q_left_leg_final);

cartesians_left_leg = [cartesians_left_leg, ctraj(T_left_leg_initial, T_left_leg_final, t)];
T_left_leg = [T_left_leg; T_left_leg_final];


% Right leg
q_right_leg_initial = q_right_leg_final;
q_right_leg_final = [1.8*pi/180, 10.8*pi/180, 7.2*pi/180, 349.0*pi/180, -2.4*pi/180, 7.2*pi/180];
q_right_leg_interpolation = [q_right_leg_interpolation; jtraj(q_right_leg_initial, q_right_leg_final, t)];  

vel_right_leg = [vel_right_leg; repmat((q_right_leg_initial - q_right_leg_final) / t * sample_time, t, 1)];

T_right_leg_initial = T_right_leg_final;
T_right_leg_final = fkine(right_leg, q_right_leg_final);

cartesians_right_leg = [cartesians_right_leg, ctraj(T_right_leg_initial, T_right_leg_final, t)];
T_right_leg = [T_right_leg; T_right_leg_final];


% Left arm
q_left_arm_initial = q_left_arm_final;
q_left_arm_final = [-90.0*pi/180, 19.8*pi/180, 37.8*pi/180, 5.4*pi/180, -5.4*pi/180, -14.4*pi/180, -25.2*pi/180];
q_left_arm_interpolation = [q_left_arm_interpolation; jtraj(q_left_arm_initial, q_left_arm_final, t)];  

vel_left_arm = [vel_left_arm; repmat((q_left_arm_initial - q_left_arm_final) / t * sample_time, t, 1)];

T_left_arm_initial = T_left_arm_final;
T_left_arm_final = fkine(left_arm, q_left_arm_final);

cartesians_left_arm = [cartesians_left_arm, ctraj(T_left_arm_initial, T_left_arm_final, t)];
T_left_arm = [T_left_arm; T_left_arm_final];


% Right arm
q_right_arm_initial = q_right_arm_final;
q_right_arm_final = [2.7*pi/180, 10.8*pi/180, 0.0*pi/180, 46.8*pi/180, 16.2*pi/180, -27.0*pi/180, -19.8*pi/180];
q_right_arm_interpolation = [q_right_arm_interpolation; jtraj(q_right_arm_initial, q_right_arm_final, t)];  

vel_right_arm = [vel_right_arm; repmat((q_right_arm_initial - q_right_arm_final) / t * sample_time, t, 1)];

T_right_arm_initial = T_right_arm_final;
T_right_arm_final = fkine(right_arm, q_right_arm_final);

cartesians_right_arm = [cartesians_right_arm, ctraj(T_right_arm_initial, T_right_arm_final, t)];
T_right_arm = [T_right_arm; T_right_arm_final];


%% 4
t = 3; 

% Left leg
q_left_leg_initial = q_left_leg_final;
q_left_leg_final = [3.6*pi/180, -2.7*pi/180, 10.8*pi/180, 336.6*pi/180, 0.0*pi/180, 1.8*pi/180];
q_left_leg_interpolation = [q_left_leg_interpolation; jtraj(q_left_leg_initial, q_left_leg_final, t)];  

vel_left_leg = [vel_left_leg; repmat((q_left_leg_initial - q_left_leg_final) / t * sample_time, t, 1)];

T_left_leg_initial = T_left_leg_final;
T_left_leg_final = fkine(left_leg, q_left_leg_final);

cartesians_left_leg = [cartesians_left_leg, ctraj(T_left_leg_initial, T_left_leg_final, t)];
T_left_leg = [T_left_leg; T_left_leg_final];


% Right leg
q_right_leg_initial = q_right_leg_final;
q_right_leg_final = [16.2*pi/180, 10.8*pi/180, 3.6*pi/180, 349.2*pi/180, -4.8*pi/180, 3.6*pi/180];
q_right_leg_interpolation = [q_right_leg_interpolation; jtraj(q_right_leg_initial, q_right_leg_final, t)];  

vel_right_leg = [vel_right_leg; repmat((q_right_leg_initial - q_right_leg_final) / t * sample_time, t, 1)];

T_right_leg_initial = T_right_leg_final;
T_right_leg_final = fkine(right_leg, q_right_leg_final);

cartesians_right_leg = [cartesians_right_leg, ctraj(T_right_leg_initial, T_right_leg_final, t)];
T_right_leg = [T_right_leg; T_right_leg_final];


% Left arm
q_left_arm_initial = q_left_arm_final;
q_left_arm_final = [-90.0*pi/180, 7.2*pi/180, 35.1*pi/180, 1.8*pi/180, -10.8*pi/180, -14.4*pi/180, -45.0*pi/180];
q_left_arm_interpolation = [q_left_arm_interpolation; jtraj(q_left_arm_initial, q_left_arm_final, t)];  

vel_left_arm = [vel_left_arm; repmat((q_left_arm_initial - q_left_arm_final) / t * sample_time, t, 1)];

T_left_arm_initial = T_left_arm_final;
T_left_arm_final = fkine(left_arm, q_left_arm_final);

cartesians_left_arm = [cartesians_left_arm, ctraj(T_left_arm_initial, T_left_arm_final, t)];
T_left_arm = [T_left_arm; T_left_arm_final];


% Right arm
q_right_arm_initial = q_right_arm_final;
q_right_arm_final = [29.7*pi/180, 32.4*pi/180, 2.7*pi/180, 5.4*pi/180, 5.4*pi/180, -32.4*pi/180, -0.0*pi/180];
q_right_arm_interpolation = [q_right_arm_interpolation; jtraj(q_right_arm_initial, q_right_arm_final, t)];  

vel_right_arm = [vel_right_arm; repmat((q_right_arm_initial - q_right_arm_final) / t * sample_time, t, 1)];

T_right_arm_initial = T_right_arm_final;
T_right_arm_final = fkine(right_arm, q_right_arm_final);

cartesians_right_arm = [cartesians_right_arm, ctraj(T_right_arm_initial, T_right_arm_final, t)];
T_right_arm = [T_right_arm; T_right_arm_final];


%% 5
t = 5; 

% Left leg
q_left_leg_initial = q_left_leg_final;
q_left_leg_final = [5.4*pi/180, -5.4*pi/180, 21.6*pi/180, 329.4*pi/180, -1.2*pi/180, -3.6*pi/180];
q_left_leg_interpolation = [q_left_leg_interpolation; jtraj(q_left_leg_initial, q_left_leg_final, t)];  

vel_left_leg = [vel_left_leg; repmat((q_left_leg_initial - q_left_leg_final) / t * sample_time, t, 1)];

T_left_leg_initial = T_left_leg_final;
T_left_leg_final = fkine(left_leg, q_left_leg_final);

cartesians_left_leg = [cartesians_left_leg, ctraj(T_left_leg_initial, T_left_leg_final, t)];
T_left_leg = [T_left_leg; T_left_leg_final];


% Right leg
q_right_leg_initial = q_right_leg_final;
q_right_leg_final = [57.6*pi/180, 8.1*pi/180, 18.0*pi/180, 334.8*pi/180, -40.8*pi/180, -16.2*pi/180];
q_right_leg_interpolation = [q_right_leg_interpolation; jtraj(q_right_leg_initial, q_right_leg_final, t)];  

vel_right_leg = [vel_right_leg; repmat((q_right_leg_initial - q_right_leg_final) / t * sample_time, t, 1)];

T_right_leg_initial = T_right_leg_final;
T_right_leg_final = fkine(right_leg, q_right_leg_final);

cartesians_right_leg = [cartesians_right_leg, ctraj(T_right_leg_initial, T_right_leg_final, t)];
T_right_leg = [T_right_leg; T_right_leg_final];


% Left arm
q_left_arm_initial = q_left_arm_final;
q_left_arm_final = [26.1*pi/180, -29.7*pi/180, 21.6*pi/180, 113.0*pi/180, -1.8*pi/180, -50.4*pi/180, -7.2*pi/180];
q_left_arm_interpolation = [q_left_arm_interpolation; jtraj(q_left_arm_initial, q_left_arm_final, t)];  

vel_left_arm = [vel_left_arm; repmat((q_left_arm_initial - q_left_arm_final) / t * sample_time, t, 1)];

T_left_arm_initial = T_left_arm_final;
T_left_arm_final = fkine(left_arm, q_left_arm_final);

cartesians_left_arm = [cartesians_left_arm, ctraj(T_left_arm_initial, T_left_arm_final, t)];
T_left_arm = [T_left_arm; T_left_arm_final];


% Right arm
q_right_arm_initial = q_right_arm_final;
q_right_arm_final = [48.6*pi/180, 54.9*pi/180, 21.6*pi/180, 9.0*pi/180, -1.8*pi/180, -28.8*pi/180, 12.6*pi/180];
q_right_arm_interpolation = [q_right_arm_interpolation; jtraj(q_right_arm_initial, q_right_arm_final, t)];  

vel_right_arm = [vel_right_arm; repmat((q_right_arm_initial - q_right_arm_final) / t * sample_time, t, 1)];

T_right_arm_initial = T_right_arm_final;
T_right_arm_final = fkine(right_arm, q_right_arm_final);

cartesians_right_arm = [cartesians_right_arm, ctraj(T_right_arm_initial, T_right_arm_final, t)];
T_right_arm = [T_right_arm; T_right_arm_final];

vel_left_leg = [vel_left_leg; [0,0,0,0,0,0]];
vel_right_leg = [vel_right_leg; [0,0,0,0,0,0]];
vel_left_arm = [vel_left_arm; [0,0,0,0,0,0,0]];
vel_right_arm = [vel_right_arm; [0,0,0,0,0,0,0]];


% Animation plot
len_leg = size(q_left_leg_interpolation);
len_arm = size(q_left_arm_interpolation);
anim1 = Animate('golf.mp4');
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


%% Cartesian positions FW model

% arms
id_fig = 2;

% Determine the number of elements
n = length(cartesians_left_arm);

% Preallocate the arrays
pos_left_arm = zeros(n, length(cartesians_left_arm(1).t));
pos_right_arm = zeros(n, length(cartesians_right_arm(1).t));

% Fill the preallocated arrays
for i = 1:n
    pos_left_arm(i, :) = cartesians_left_arm(i).t';
    pos_right_arm(i, :) = cartesians_right_arm(i).t';
end


figure(id_fig);
id_fig = id_fig +1;
plot(pos_right_arm,'-');
grid;
title("Position Right Arm")
legend('X','Y','Z');

figure(id_fig);grid;
id_fig = id_fig +1;
plot(pos_left_arm,'-');
grid;
title("Position Left Arm")
legend('X','Y','Z');

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
plot(pos_right_leg,'-');
grid;
title("Position Right Leg")
legend('X','Y','Z');

figure(id_fig);grid;
id_fig = id_fig +1;
plot(pos_left_leg,'-');
grid;
title("Position Left Leg")
legend('X','Y','Z');


%% Velocities

% Left leg
figure(id_fig);
id_fig = id_fig +1;
plot(vel_left_leg);
grid;
title('Radian/s vel left leg joints');
legend('q1','q2','q3','q4','q5','q6');

% Right leg
figure(id_fig);
id_fig = id_fig +1;
plot(vel_right_leg);
grid;
title('Radian/s vel right leg joints');
legend('q1','q2','q3','q4','q5','q6');

% Left arm
figure(id_fig);
id_fig = id_fig +1;
plot(vel_left_arm);
grid;
title('Radian/s vel left arm joints');
legend('q1','q2','q3','q4','q5','q6','q7');

% Right arm
figure(id_fig);
id_fig = id_fig +1;
plot(vel_right_arm);
grid;
title('Radian/s vel right arm joints');
legend('q1','q2','q3','q4','q5','q6','q7');

