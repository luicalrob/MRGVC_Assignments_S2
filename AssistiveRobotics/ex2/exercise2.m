%%  Structure with 2 joints exercise
l_1 = 5;
l_2 = 5;
%% A

% Elements that define a link: DH parameters, sigma (==0 rotat., == 1
% transl.), offset: if we want to include an initial offset for the joint 
%[theta,D,A,alpha,sigma,offset]
L1 = Link([0,0,0, -pi/2,0,30*pi/180]); % DH parameters for 0T1
%L1 = Link([0,0,0, -pi/2,0,-60*pi/180]); % DH parameters for 0T1
L2 = Link([0,l_1+l_2,0,0,1,0]); %  DH parameters for 1T2
% Set limits for the joint
L2.qlim = [0 15];
% Construction of an object belonging to the class robot
my_arm=SerialLink([L1,L2],'name', 'robot1');
% Location of the base reference frame
my_arm.base=transl(0,0,0)*trotx(pi/2);
% Visualization of the robot in the joint position given by qr
qr=[0, 2];
plot(my_arm,qr)
% Visualization of the robot and manual guidance of the different joints, starting from
% the initial joint position qr
teach(my_arm,qr)

%% B

% Elements that define a link: DH parameters, sigma (==0 rotat., == 1
% transl.), offset: if we want to include an initial offset for the joint 
%[theta,D,A,alpha,sigma,offset]
L1 = Link([0,0,l_1, 0,0,20*pi/180]); % DH parameters for 0T1
L2 = Link([0,0,l_2,0,0,40*pi/180]); %  DH parameters for 1T2
% Set limits for the joint
L2.qlim = [0 15];
% Construction of an object belonging to the class robot
my_arm=SerialLink([L1,L2],'name', 'robot1');
% Location of the base reference frame
my_arm.base=transl(0,0,0)*trotx(pi/2);
% Visualization of the robot in the joint position given by qr
qr=[0, 0];
plot(my_arm,qr)
% Visualization of the robot and manual guidance of the different joints, starting from
% the initial joint position qr
teach(my_arm,qr)


%% C

% Elements that define a link: DH parameters, sigma (==0 rotat., == 1
% transl.), offset: if we want to include an initial offset for the joint 
%[theta,D,A,alpha,sigma,offset]
L1 = Link([0,l_1,0, pi/2,0,0]); % DH parameters for 0T1
L2 = Link([0,0,l_2,0,0,45*pi/180]); %  DH parameters for 1T2
% Set limits for the joint
L2.qlim = [0 15];
% Construction of an object belonging to the class robot
my_arm=SerialLink([L1,L2],'name', 'robot1');
% Location of the base reference frame
my_arm.base=transl(0,0,0)*trotx(0);
% Visualization of the robot in the joint position given by qr
qr=[0, 0];
plot(my_arm,qr)
% Visualization of the robot and manual guidance of the different joints, starting from
% the initial joint position qr
teach(my_arm,qr)
