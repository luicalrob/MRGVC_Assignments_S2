%controlVisualF.m
%Position based visual servoing by means of calibrated Fundamental matrix
%Gonzalo Lopez-Nicolas. 2023
%
%Uses auxiliary files available in the 'funciones_auxiliares' folder.
%(Includes functions from Peter Corke's Robotics Toolbox)
%And the file to be created funcionCalcularF.m

clear all;
close all;
addpath('./funciones_auxiliares');


%Camera calibration
k=[640     0     0
     0   480     0
     0     0     1];

%Target position of the camera (Do not modify):
ct= [500 -600 0 0 0 0]; %[x y z rx ry rz] (units en mm and degrees)

%Initial position of the camera:
c= [500 600 -100 -10 15 20]; %[x y z rx ry rz] (units en mm and degrees)

%Create a set of 3D points as a scenario
[Puntos3D,n] = generaEscena3D;

% FILL IN: Optionally, use a planar scene
% [Puntos3D,n] = generaEscenaPlana;


%TARGET IMAGE
%Generate the image from the target position
[Puntos2Dt, Puntos2Dtk, d] = generaImagen(k, ct, Puntos3D);
%d: Approximate estimate of the distance from the plane to the camera position

Tciclo= 1;
Tmax= 300; %Maximum time
v_tiempo=[0:Tciclo:Tmax];
tamTiempo=length(v_tiempo);


%Vectors to save simulation data
v_posiciones=zeros(tamTiempo,6); %Save the camera motion
v_correccionesvw=zeros(tamTiempo,6); %Save control velocities
v_efes=zeros(tamTiempo,9); %Save homographies entries
v_pts=zeros(tamTiempo,2*n); %Saves trajectories of image points


%Rotation and translation for initial solution selection from F
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
    
   
    %FILL IN. Optionally, introduce image noise:
%     ruido=0.1;
%     Puntos2Dt= Puntos2Dt + randn(n,2)*ruido;
%     Puntos2D= Puntos2D + randn(n,2)*ruido;
    

    %COMPUTE FUNDAMENTAL MATRIX
    %Compute F from corresponding points [Puntos2Dt Puntos2D]
    %Reutilizando el código de prácticas anteriores, crea la función 
    %Define a new function in a file 'funcionCalcularF.m' that returns the fundamental matrix: 
    %
    % FILL IN
    %
    F=funcionCalcularF(Puntos2Dt, Puntos2D);
    % F=diag([0 1 1]); %Remove this when you have the function implemented


    F=F/F(3,3);
    v_efes(it,:)=[F(1,:) F(2,:) F(3,:)]; %save simulation data for plots
    
        
    %Compute rotation and translation (t1, t2, R1, R2) from F:
    %
    % FILL IN
    %--------------------------------------------------------------------
    
    [R1, R2, t1, e0, e1] = decomposeF(k, F);

    % %Just to put something:
    % R1=eye(3);
    % R2=R1;
    % t1=[0 0 0]';
    %--------------------------------------------------------------------
 
    %The correct motion solution is chosen:
    [R_est, c_est]=seleccionaSolucionF(R1,R2,t1,R_est,c_est);   
    
    %CONTROL LAW
    %
    % FILL IN
    %
    %From Ri and ti compute the linear velocities v=(vx, vy, vz)
    %and angular velocities w=(wx, wy, wz)
       
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

    K_v = 0.15;
    K_w = 5.0;
    w = -K_w * [roll pitch yaw];

    % Compute epipole coordinates in target camera
    % epipole in current frame: optical centre of target camera in current
    % image
    dist_epipoles = norm(e1-e0);
    % epipole in target frame: optical centre of current camera in target
    % image
    % Apply stop condition if necessary
    if dist_epipoles < 0.2
        % Stop condition reached, exit loop  
        break;
    else
        v = K_v * e1;
        v = v';
    end

    %v= [0 0 0];%At the moment we are not moving; Just to put something
    % w= [0 0 0];%At the moment we are not moving; Just to put something

    v_correccionesvw(it,:)=[v w]; %save simulation data for plots
    
    %Perform motion
    %We act directly on the modeled robot with integrators
    c(1:3) = c(1:3) + v*Tciclo;
    c(4:6) = c(4:6) + w*Tciclo;
    
end


%%PLOTS
%Draw plots with the results obtained.
generaGraficas(Puntos3D,ct,Puntos2Dt,v_tiempo,v_posiciones,v_correccionesvw,v_pts,v_efes);

%TO DISPLAY STEP BY STEP (It is a little slow, use from time to time)
%tpausa=0.1;
% generaAnimacion(Puntos2Dt,v_tiempo,v_pts,tpausa);

%Visualizes robot motion
% generaAnimacionRobot(Puntos3D,v_tiempo,v_posiciones);

