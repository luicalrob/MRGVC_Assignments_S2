%controlVisualimage.m
%Image based visual servoing by means of the interaction matrix
%Gonzalo Lopez-Nicolas. 2023
%
%Uses auxiliary files available in the 'funciones_auxiliares' folder.
%(Includes functions from Peter Corke's Robotics Toolbox)

clear all;
close all;
addpath('./funciones_auxiliares');

%Camera calibration
k=[640     0     320
     0   480     240
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
[Puntos2Dt, Puntos2Dkt, dt] = generaImagen(k, ct, Puntos3D);
%d: Approximate estimate of the distance from the plane to the camera position
figure;
plot(Puntos2Dt(:,1),Puntos2Dt(:,2),'*r');

% %Create a set of points as a scenario
% [Puntos3D,n] = generaEscenaPlana;
% 
% %TARGET IMAGE
% %Generate the image from the target position
% [Puntos2Dt, Puntos2Dkt, dt] = generaImagen(k, ct, Puntos3D);
% %d: Approximate estimate of the distance from the plane to the camera position
% figure;
% plot(Puntos2Dt(:,1),Puntos2Dt(:,2),'*-r');

z_deseado=dt*ones(n,1);
z_cte=-300*ones(n,1); %We estimate the same depth for all the points with an approximate value

Tciclo= 0.4;
Tmax= 100; %Maximum time
v_tiempo=[0:Tciclo:Tmax];
tamTiempo=length(v_tiempo);


%Vectors to save simulation data
v_posiciones=zeros(tamTiempo,6); %Save the camera motion
v_correccionesvw=zeros(tamTiempo,6); %Save control velocities
v_pts=zeros(tamTiempo,2*n); %Saves trajectories of image points

%MAIN LOOP
for it=1:tamTiempo,

    v_posiciones(it,:)=c; %save simulation data for plots


        R= Rgiros( c(4:6) ); 

    %CURRENT IMAGE
    %Generate the image from the current position
    [Puntos2D, Puntos2Dk, d] = generaImagen(k, c, Puntos3D);
    R= Rgiros( c(4:6) );
    z_real=d*ones(n,1);

    v_pts(it,:)= reshape(Puntos2D',1, n*2); %save simulation data for plots



    %INTERACTION MATRIX - JACOBIAN
    %Build the interaction matrix J that relates the motion of the
    % points [Puntos2Dk] in the image with the movement of the 3D 
    % camera. You have available the real depth of the points in
    % z_real, and the real depth of the reference points in z_deseado
    %
    % FILL IN
    %--------------------------------------------------------------------

    %J = eye(6); %Just to put something:

    % Number of points
    n = size(Puntos2Dk, 1);

    % Construct the matrix A
    J = zeros(2 * n, 6);
    for i = 1:n
        x = Puntos2Dk(i, 1);
        y = Puntos2Dk(i, 2);
        z = z_real(i);
        J(2*i-1, :) = [-1./z, 0, x/z, x*y, -(1+x.^2), y];
        J(2*i, :) = [0, -1./z, y/z, (1+y.^2), -x*y, -x];
    end

    %--------------------------------------------------------------------

    %CONTROL LAW
    %
    %Compute the pseudoinverse Ji of the interaction matrix J.
    %Define the control law, to calculate linear velocities
    %v=(vx, vy, vz) and the angular velocities w=(wx, wy, wz),
    %from Ji and the points in the image in calibrated coordinates.
    %
    % FILL IN

    Ji = pinv(J);

    lambda = 0.1;

    e = Puntos2Dk - Puntos2Dkt;
    e = reshape(e', [2*n 1]);

    % e = zeros(2 * n, 1);
    % for i = 1:n
    %     e(2*i-1) = error(i,1);
    %     e(2*i) = error(i,2);
    % end

    vel = - lambda * Ji * e;

    v= [vel(1);vel(2);vel(3)];%At the moment we are not moving; Just to put something
    w= [vel(4);vel(5);vel(6)];%At the moment we are not moving; Just to put something

    %--------------------------------------------------------------------
    v=[R*v]';
    w=[R*w]'*180/pi;

    v_correccionesvw(it,:)=[v w]; %save simulation data for plots

    %Perform motion
    %We act directly on the modeled robot with integrators
    c(1:3) = c(1:3) + v*Tciclo;
    c(4:6) = c(4:6) + w*Tciclo;

end

%%PLOTS
%Draw plots with the results obtained
generaGraficas(Puntos3D,ct,Puntos2Dt,v_tiempo,v_posiciones,v_correccionesvw,v_pts);

%TO DISPLAY STEP BY STEP (It is a little slow, use from time to time)
% tpausa=0.1;
% generaAnimacion(Puntos2Dt,v_tiempo,v_pts,tpausa);

%Visualizes robot motion
generaAnimacionRobot(Puntos3D,v_tiempo,v_posiciones);
