clear;
close all;

%%  Structure with 2 joints exercise
l_1 = 5;
l_2 = 5;

%% LEFT LEG

%       [theta,     D,      A,      alpha,  sigma,  offset]

% HIP
% DH parameters for 0T1 (Flexion-Extension)
L1 = Link([0,       0,      0,      pi/2,   0,      0]); 
L1.qlim = [0, pi/2];
%  DH parameters for 1T2 (Abduction-Adduction)
L2 = Link([0,       0,      0,      pi/2,   0,      pi/2]);
L2.qlim = [-pi/2, pi];
%  DH parameters for 2T3 (Circumduction)
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
left_leg_qr=[0, 0, 0, 0, 0, 0];
left_leg.plot(left_leg_qr)
% Visualization of the robot and manual guidance of the different joints, starting from
% the initial joint position qr
teach(left_leg, left_leg_qr)
hold on;

%% RIGHT LEG

%       [theta,     D,      A,      alpha,  sigma,  offset]

% HIP
% DH parameters for 0T1 (Flexion-Extension)
L1 = Link([0,       0,      0,      pi/2,   0,      0]); 
L1.qlim = [-pi/2, 0];
%  DH parameters for 1T2 (Abduction-Adduction)
L2 = Link([0,       0,      0,      pi/2,   0,      pi/2]);
L2.qlim = [-pi/2, pi];
%  DH parameters for 2T3 (Circumduction)
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
right_leg_qr=[0, 0, 0, 0, 0, 0];
right_leg.plot(right_leg_qr)
% Visualization of the robot and manual guidance of the different joints, starting from
% the initial joint position qr
teach(right_leg, right_leg_qr)


%% LEFT ARM

%       [theta,     D,      A,      alpha,  sigma,  offset]

% SHOULDER
% DH parameters for 0T1 (Flexion-Extension)
L1 = Link([0,       0,      0,      pi/2,   0,      pi/2]); 
L1.qlim = [0, pi];
%  DH parameters for 1T2 (Abduction-Adduction)
L2 = Link([0,       0,      0,      pi/2,   0,      pi/2]);
L2.qlim = [0, pi];
%  DH parameters for 2T3 (Circumduction)
L3 = Link([0,       0,      2,      pi/2,   0,      pi]);
L3.qlim = [-pi, pi/2];

% ELBOW
% DH parameters for 3T4 (Flexion-Extension)
L4 = Link([0,       0,      0,      pi/2,   0,      pi]);  
L4.qlim = [-pi/2, pi/2];
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
left_arm=SerialLink([L1,L2,L3,L4,L5,L6],'name', 'left_arm');
% Location of the base reference frame
left_arm.base=transl(0,1.5,5)*trotz(-pi/2);
% Visualization of the robot in the joint position given by qr
left_arm_qr=[0, 0, 0, 0, 0, 0];
left_arm.plot(left_arm_qr, 'nobase')
% Visualization of the robot and manual guidance of the different joints, starting from
% the initial joint position qr
teach(left_arm, left_arm_qr, 'nobase')


%% RIGHT ARM

%       [theta,     D,      A,      alpha,  sigma,  offset]

% SHOULDER
% DH parameters for 0T1 (Flexion-Extension)
L1 = Link([0,       0,      0,      pi/2,   0,      pi/2]); 
L1.qlim = [0, pi];
%  DH parameters for 1T2 (Abduction-Adduction)
L2 = Link([0,       0,      0,      pi/2,   0,      pi/2]);
L2.qlim = [0, pi];
%  DH parameters for 2T3 (Circumduction)
L3 = Link([0,       0,      2,      pi/2,   0,      pi]);
L3.qlim = [-pi, pi/2];

% ELBOW
% DH parameters for 3T4 (Flexion-Extension)
L4 = Link([0,       0,      0,      pi/2,   0,      pi]);  
L4.qlim = [-pi/2, pi/2];
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
right_arm=SerialLink([L1,L2,L3,L4,L5,L6],'name', 'right_arm');
% Location of the base reference frame
right_arm.base=transl(0,0,5)*transl(0,-1.5,0)*trotz(-pi/2);
% Visualization of the robot in the joint position given by qr
right_arm_qr=[0, 0, 0, 0, 0, 0];

right_arm.plot(right_arm_qr, 'nobase');
% Visualization of the robot and manual guidance of the different joints, starting from
% the initial joint position qr

teach(right_arm, right_arm_qr, 'nobase')


% Plot constant trunk, hip, shoulders and neck
plot3([0 0], [0 0], [0 5], 'k-', 'LineWidth', 2);
plot3([0 0], [1 -1], [0 0], 'k-', 'LineWidth', 2);
plot3([0 0], [1.5 -1.5], [5 5], 'k-', 'LineWidth', 2);




