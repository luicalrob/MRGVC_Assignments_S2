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
ct= [500 -600 200 0 0 0]; %[x y z rx ry rz] (units en mm and degrees)

%Initial position of the camera:
%Posici�n inicial de la c�mara:
c= [500 -600 200 0 0 170]; %1
%c= [600 0 -300 0 0 170]; %2

%Create a set of points as a scenario
[Puntos3D,n] = generaEscenaPlana;

%TARGET IMAGE
%Generate the image from the target position
[Puntos2Dt, Puntos2Dkt, dt] = generaImagen(k, ct, Puntos3D);
%d: Approximate estimate of the distance from the plane to the camera position

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

n = size(Puntos2Dkt, 1);
Puntos2Dkt_cyl = zeros(2*n);
for i=1:n
    xt = Puntos2Dkt(i, 1);
    yt = Puntos2Dkt(i, 2);
    rhokt = sqrt(xt.^2 + yt.^2);
    thetakt = atan2(yt,xt);
    Puntos2Dkt_cyl(2*i-1) = rhokt;
    Puntos2Dkt_cyl(2*i) = thetakt;
end

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
    
    Puntos2Dk_cyl = zeros(2*n);

    for i = 1:n
        x = Puntos2Dk(i, 1);
        y = Puntos2Dk(i, 2);
        z = z_real(i);
        rhok = sqrt(x.^2 + y.^2);
        thetak = atan2(y,x);
        J(2*i-1, :) = [-cos(thetak)/z, -sin(thetak)/z, rhok/z, (1+rhok.^2)*sin(thetak), -(1+rhok.^2)*cos(thetak), 0.];
        J(2*i, :) = [sin(thetak)/(rhok*z), -cos(thetak)/(rhok*z), 0., cos(thetak)/rhok, sin(thetak)/rhok, -1.];
        
        Puntos2Dk_cyl(2*i-1) = rhok;
        Puntos2Dk_cyl(2*i) = thetak;

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

    e = Puntos2Dk_cyl - Puntos2Dkt_cyl;

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
