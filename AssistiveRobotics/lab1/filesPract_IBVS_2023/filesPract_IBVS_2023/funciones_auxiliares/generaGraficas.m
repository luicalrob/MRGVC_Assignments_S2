function generaGraficas(Puntos3D,ct,Puntos2Dt,v_tiempo,v_posiciones,v_correccionesvw,v_pts,v_HF)
% function generaGraficas(Puntos3D,ct,Puntos2Dt,v_tiempo,v_posiciones,v_correccionesvw,v_pts,v_HF)
%A partir de la simulación genera todo gráficas con los resultados
%Gonzalo Lopez Nicolas Agosto 2022

grosor=3;
tamarca=18;
n=size(Puntos3D,1);

%%

figure;
subplot(2,3,1);
plot(v_tiempo, v_posiciones(:,1:3));  
legend('x','y','z');
title('Evolución de posiciones (x y z)');

subplot(2,3,4);
plot(v_tiempo,v_posiciones(:,4:6));  
legend('Rx','Ry','Rz');
title('Evolución de orientaciones (Rx Ry Rz)');


subplot(2,3,2);
plot(v_tiempo, v_correccionesvw(:,1:3));
legend('Vx','Vy','Vz');
title('Velocidades (Vx Vy Vz)');

subplot(2,3,5);
plot(v_tiempo, v_correccionesvw(:,4:6)); 
legend('wx','wy','wz');
title('Velocidades de rotación (wx wy wz)');


unos=ones(length(v_tiempo),1);
subplot(2,3,3);
plot(v_tiempo, v_posiciones(:,1:3)-[ct(1)*unos,ct(2)*unos,ct(3)*unos]);
legend('Ex','Ey','Ez');
title('Error de posiciones (x y z)');

subplot(2,3,6);
plot(v_tiempo, v_posiciones(:,4:6)-[ct(4)*unos,ct(5)*unos,ct(6)*unos]);
legend('ERx','ERy','ERz');
title('Error de orientaciones (Rx Ry Rz)');

%%
if nargin>7    
    %H o F, comprobar rango
    rango=3;
    for ii=1:length(v_tiempo)
        HF=reshape(v_HF(ii,:),3,3);
        if rank(HF)<3
            rango=rank(HF);
            break
        end
    end
    if rango==3
        %H
        figure
        subplot(3,3,1); plot(v_tiempo, v_HF(:,1)); title('H_{11}');title('Evolución homografías');
        subplot(3,3,2); plot(v_tiempo, v_HF(:,2)); title('H_{12}');
        subplot(3,3,3); plot(v_tiempo, v_HF(:,3)); title('H_{13}');
        subplot(3,3,4); plot(v_tiempo, v_HF(:,4)); title('H_{21}');
        subplot(3,3,5); plot(v_tiempo, v_HF(:,5)); title('H_{22}');
        subplot(3,3,6); plot(v_tiempo, v_HF(:,6)); title('H_{23}');
        subplot(3,3,7); plot(v_tiempo, v_HF(:,7)); title('H_{31}');
        subplot(3,3,8); plot(v_tiempo, v_HF(:,8)); title('H_{32}');
        subplot(3,3,9); plot(v_tiempo, v_HF(:,9)); title('H_{33}');
    else
        %F
        figure
        subplot(3,3,1); plot(v_HF(:,1)); title('F_{11}');title('Evolución F');
        subplot(3,3,2); plot(v_HF(:,2)); title('F_{12}');
        subplot(3,3,3); plot(v_HF(:,3)); title('F_{13}');
        subplot(3,3,4); plot(v_HF(:,4)); title('F_{21}');
        subplot(3,3,5); plot(v_HF(:,5)); title('F_{22}');
        subplot(3,3,6); plot(v_HF(:,6)); title('F_{23}');
        subplot(3,3,7); plot(v_HF(:,7)); title('F_{31}');
        subplot(3,3,8); plot(v_HF(:,8)); title('F_{32}');
        subplot(3,3,9); plot(v_HF(:,9)); title('F_{33}');
    end
end
%%
figure;
plot3(v_posiciones(:,1), v_posiciones(:,2), v_posiciones(:,3), '.');
hold;
plot3(Puntos3D(:,1), Puntos3D(:,2), Puntos3D(:,3), '.','LineWidth',grosor,'Markersize',tamarca); 

for i=1:length(v_tiempo),
  dibCamara3D(v_posiciones(i,:),'k-', 50);  
end;
axis equal

dibCamara3D(ct,'r-',50,'r');  
dibCamara3D(v_posiciones(1,:),'g-',50,'g');  
title('trayectoria 3D');
%%

figure;
plot(Puntos2Dt(:,1),Puntos2Dt(:,2),'*r');
hold

paso=2;
plot(v_pts(1,1:2:2*n),v_pts(1,2:2:2*n),'*b');
plot(v_pts(:,1:paso:2*n),v_pts(:,2:paso:2*n),':b');
axis equal;
title('Plano imagen. *rojo=deseado, *azul=trayectoria puntos');
