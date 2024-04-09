% To define robot links using the associated D-H parameters 
% [theta,d,a,alpha,sigma]  (sigma=0, rotational joint; sigma=1, translational)
B(1) = Link([0 0 0 -pi/2 0]);

% Matrix with DH parameters. Rows are arranged as in B
dh = [theta1, d1, a1, alpha1, sigma1;
      theta2, d2, a2, alpha2, sigma2;
      ...
      thetan, dn, an, alphan, sigman];

% Establishing joint limits  
B(1).qlim = [-pi/2 pi/2];

% Construction of an object belonging to the class robot
my_arm = SerialLink(B, 'name', 'right arm');

% Location of the base reference frame
my_arm.base = transl(0, 2, 0) * trotx(pi);

% Visualization of the robot in the joint position given by qr
plot(my_arm, qr)

% Visualization of the robot and manual guidance of the different joints, starting from
% the initial joint position qr
teach(my_arm, qr)

% Joint trajectory from qi to qf, using 100 intermediary steps
trayart = jtraj(qi, qf, 100);

% It does the same, but using the timesteps given by time vector t
% (useful to fix the speed)
trayart = jtraj(qi, qf, t);

% Straight Cartesian trajectory from T0 to Tf, computing 100 intermediary positions
traycar = ctraj(Ti, Tf, 100);

% It does the same, but it uses the time vector t instead
traycar = ctraj(Ti, Tf, t);

% Computation of the Cartesian location T based on the DH robot parameters and the
% joint position vector q (Forward kinematics model)
T = fkine(my_arm, q);

% Computation of the joint position q based on the DH robot parameters and the
% Cartesian location T (Inverse kinematics model).
% q0 is the initial value (default: q0=0).
q = ikine(my_arm, T, q0);

% More functions, scripts, and demos: Toolbox Robotics in moodle2
