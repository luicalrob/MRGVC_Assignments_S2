%controlVisualH.m
%Position based visual servoing by means of calibrated homographies
%Gonzalo Lopez-Nicolas. 2023
%
%Uses auxiliary files available in the 'funciones_auxiliares' folder.
%(Includes functions from Peter Corke's Robotics Toolbox)
%And the file to be created funcionCalcularH.m

clear all;
close all;
addpath('./funciones_auxiliares');


%Camera calibration
k=[640     0     0;
     0   480     0;
     0     0     1];

%Target position of the camera (Do not modify):
ct= [-716 -630 -230 0 0 0]; %[x y z rx ry rz] (units en mm and degrees)

%Initial position of the camera:
%Posición inicial de la cámara:
c= [-1250 -100 -50 -20 10 -40];
%c= [500 -600 200 -20 0 0];


%Create a set of points as a scenario
img = imread('./cara.jpg');
[Puntos3D,n] = generaEscenaCara(img);

%TARGET IMAGE
%Generate the image from the target position
[Puntos2Dt, Puntos2Dtk, d] = generaImagen(k, ct, Puntos3D);
%d: Approximate estimate of the distance from the plane to the camera position


Tciclo= 1;
Tmax= 100; %Maximum time
v_tiempo=[0:Tciclo:Tmax];
tamTiempo=length(v_tiempo);


%Vectors to save simulation data
v_posiciones=zeros(tamTiempo,6); %Save the camera motion
v_correccionesvw=zeros(tamTiempo,6); %Save control velocities
v_homs=zeros(tamTiempo,9); %Save homographies entries
v_pts=zeros(tamTiempo,2*n); %Saves trajectories of image points


%Rotation and translation for initial solution selection from H
Rt= Rgiros( ct(4:6) ); 
R_est=Rt'*Rgiros( c(4:6) ); 
c_est=Rt'*( c(1:3)-ct(1:3) )';


%MAIN LOOP
for it=1:tamTiempo,
    
    v_posiciones(it,:)=c; %save simulation data for plots
    
    %CURRENT IMAGE
    %Generate the image from the current position
    Puntos2D = generaImagen(k, c, Puntos3D);
    v_pts(it,:)= reshape(Puntos2D',1, n*2); %save simulation data for plots
    
       
    %COMPUTE HOMOGRAPHY
    %Compute homography from corresponding points [Puntos2Dt Puntos2D]
    %Define a new function in a file 'funcionCalcularH.m' that returns the computed homography: 
    %
    % FILL IN
    %
    H=funcionCalcularH(Puntos2Dt, Puntos2D);
    %H=eye(3); %Remove this when you have the function implemented


    %Normalize H
    H=H/H(3,3);
    v_homs(it,:)=[H(1,:) H(2,:) H(3,:)]; %save simulation data for plots
    
    %Compute rotation and translation (t1, t2, R1, R2) from H:
    %
    % FILL IN
    [R1, R2, t1, t2] = decomposeH(k, H, d);
    
    %--------------------------------------------------------------------
    
    % %Just to put something:
    % R1=eye(3);
    % R2=R1;
    % t1=[0 0 0]';
    % t2=t1;
    %--------------------------------------------------------------------
    
    %The correct motion solution is chosen:
	[R_est, c_est]=seleccionaSolucionH(R1,t1,R2,t2,R_est,c_est);


    %CONTROL LAW
    %
    % FILL IN
    %
    %From R_est and c_est compute the linear velocities v=(vx, vy, vz)
    %and angular velocities w=(wx, wy, wz)
    % v= [0 0 0];%At the moment we are not moving; Just to put something
    % w= [0 0 0];%At the moment we are not moving; Just to put something
    
    r11 = R_est(1,1);
    r21 = R_est(2,1);
    r31 = R_est(3,1);
    r32 = R_est(3,2);
    r33 = R_est(3,3);

    % Calculate yaw (heading)
    yaw = atan2(r21, r11);
    
    % Calculate pitch (attitude)
    pitch = asin(-r31);
    
    % Calculate roll (bank)
    roll = atan2(r32, r33);

    K_v = 0.05;
    K_w = 5.0;
    w = -K_w * [roll pitch yaw];
    v = -K_v * c_est;
    v = v';

    v_correccionesvw(it,:)=[v w]; %save simulation data for plots
    
    %Perform motion
    %We act directly on the modeled robot with integrators
    c(1:3) = c(1:3) + v*Tciclo;
    c(4:6) = c(4:6) + w*Tciclo;
    
end

%%PLOTS
%Draw plots with the results obtained
generaGraficas(Puntos3D,ct,Puntos2Dt,v_tiempo,v_posiciones,v_correccionesvw,v_pts,v_homs);

%TO DISPLAY STEP BY STEP (It is a little slow, use from time to time)
%tpausa=0.1;
% generaAnimacion(Puntos2Dt,v_tiempo,v_pts,tpausa);

%Visualizes robot motion
generaAnimacionRobot(Puntos3D,v_tiempo,v_posiciones);

