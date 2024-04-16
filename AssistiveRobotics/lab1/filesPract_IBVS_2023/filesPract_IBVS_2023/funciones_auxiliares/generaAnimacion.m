function generaAnimacion(Puntos2Dt,v_tiempo,v_pts,tpausa)
% function generaGraficas(Puntos3D,ct,Puntos2Dt,v_tiempo,v_posiciones,v_correccionesvw,v_pts,v_homs)
%A partir de la simulaci�n genera animaci�n de la c�mara en movimiento, 
%cada iteraci�n con una duraci�n de 'paso' segundos
%Gonzalo Lopez Nicolas Agosto 2022

if nargin<4
    tpausa=0.2;
end
n=size(Puntos2Dt,1);

%PARA VISUALIZAR PASO A PASO (Es un poco lento, usar de vez en cuando)
figure;
for i=2:length(v_tiempo),
    
    plot(Puntos2Dt(:,1),Puntos2Dt(:,2),'.r');
    hold
    plot(v_pts(1,1:2:2*n), v_pts(1,2:2:2*n),'.-b'); 
    
    plot(v_pts(1:i,1:2:2*n),v_pts(1:i,2:2:2*n),':b');
    plot(v_pts(i,1:2:2*n),v_pts(i,2:2:2*n),'.-b');
    axis equal;
    title(['Plano imagen. Tiempo: ',num2str(i), ' / ',num2str(length(v_tiempo))]);    
    pause(tpausa)
    clf
end;
