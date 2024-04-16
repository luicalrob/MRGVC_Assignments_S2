function [Puntos2D, Puntos2Dk, d] = generaImagen(k, c, Puntos3D)
%function function [Puntos2Dt, d] = generaImagen(k, ct)
%Gonzalo Lopez Nicolas Agosto 2022
%A partir de matriz de calibracci�n k de la c�mara y de la posici�n de la
%c�mara en el espacio c (x,y,z,rx,ry,rz), genera una imagen virtual de la
%escena Puntos3D . Devuelve puntos en la imagen en pixels (Puntos2Dk) y en 
% unidades m�tricas (Puntos2Dk). Tambi�n devuelve la distancia aproximada 
%de la c�mara al plano de la escena (asumiendo escena plana y plano 
%aproximadamente paralelo al plano imagen)

n=size(Puntos3D,1);

%Generar la imagen en la posici�n c
%Matriz de proyecci�n
R= Rgiros( c(4:6) ); 
P= k*[R', -R'*c(1:3)'];
%Proyecci�n de los puntos 3D a la imagen
Puntos2D= (P*[Puntos3D, ones(n,1) ]')'; %en pixeles


%Estimaci�n aproximada de la distancia del plano a la posici�n 
%objetivo conocida una distancia en el espacio 3D entre dos puntos:
%Aproximaci�n: Asumo que el plano es paralelo al plano imagen
Puntos2Dk=[inv(k)*Puntos2D']';%en unidades m�tricas
Puntos2Dk=[Puntos2Dk(:,1)./Puntos2Dk(:,3) , Puntos2Dk(:,2)./Puntos2Dk(:,3)];
dm=sqrt((Puntos3D(1,:)-Puntos3D(2,:))*(Puntos3D(1,:)-Puntos3D(2,:))');
dp=sqrt((Puntos2Dk(1,:)-Puntos2Dk(2,:))*(Puntos2Dk(1,:)-Puntos2Dk(2,:))');
d=-dm/dp;

%Transformar puntos de la imagen a coordenadas homog�neas
Puntos2D=[Puntos2D(:,1)./Puntos2D(:,3) , Puntos2D(:,2)./Puntos2D(:,3)];
