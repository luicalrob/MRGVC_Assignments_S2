function generaAnimacionRobot(Puntos3D,v_tiempo,v_posiciones)
% function generaAnimacionRobot(Puntos3D,ct,Puntos2Dt,v_tiempo,v_posiciones,v_correccionesvw,v_pts,v_homs)
%A partir de la simulación genera animación del robot en movimiento, 
%Gonzalo Lopez Nicolas Agosto 2022


grosor=3;
tamarca=18;
% n=size(Puntos3D,1);

hold off
%Visualiza movimiento del robot
%Carga de datos geométricos y dinámicos del robot puma560
mdl_puma560_3D;

Rcam=Rgiros([-180 0 0]);%Posición de la cámara en la muñeca del robot

v_qi=zeros(length(v_tiempo),6);
for i=1:length(v_tiempo),   
    Ti=[Rcam*Rgiros( v_posiciones(i,4:6) ) , v_posiciones(i,1:3)' ; 0 0 0 1];
    qi=p560_3D.ikine6s(Ti);
%     v_qi=[v_qi; qi];
v_qi(i,:)=qi;
end;
puma3d(v_qi)

plot3(Puntos3D(:,1), Puntos3D(:,2), Puntos3D(:,3), '.r','LineWidth',grosor,'Markersize',tamarca);
title('');

figure;
plot(v_tiempo, v_qi);  
grid;
legend('q1','q2','q3','q4','q5','q6');
title('Evolution of posiciones (radians)');
xlabel('t (s)');
ylabel('rad');

sample_time = 0.2;

position_differences = diff(v_qi, 1, 1);  % Computes the difference between consecutive rows
for i = 1:length(position_differences(:, 1))
    for j = 1:length(position_differences(1, :))
        if (position_differences(i,j) >= 2*pi*0.98)
            position_differences(i,j) = position_differences(i,j)-2*pi;
        end
        if (position_differences(i,j) <= -2*pi*0.98)
            position_differences(i,j) = position_differences(i,j)+2*pi;
        end
    end
end
% Then, divide by sample_time to get velocity
v_vel = [zeros(1,6); position_differences / sample_time];

figure;
plot(v_tiempo, v_vel); 
grid;
legend('q1','q2','q3','q4','q5','q6');
title('Evolution of velocities (radian/s)');
xlabel('t (s)');
ylabel('rad/s');

