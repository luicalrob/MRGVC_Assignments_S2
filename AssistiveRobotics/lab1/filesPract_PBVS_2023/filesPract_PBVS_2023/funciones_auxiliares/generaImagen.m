function [Puntos2D, Puntos2Dk, d] = generaImagen(k, c, Puntos3D)
%function function [Puntos2Dt, d] = generaImagen(k, ct)
%Gonzalo Lopez Nicolas Agosto 2022
%A partir de matriz de calibracción k de la cámara y de la posición de la
%cámara en el espacio c (x,y,z,rx,ry,rz), genera una imagen virtual de la
%escena Puntos3D . Devuelve puntos en la imagen en pixels (Puntos2Dk) y en 
% unidades métricas (Puntos2Dk). También devuelve la distancia aproximada 
%de la cámara al plano de la escena (asumiendo escena plana y plano 
%aproximadamente paralelo al plano imagen)

n=size(Puntos3D,1);

%Generar la imagen en la posición c
%Matriz de proyección
R= Rgiros( c(4:6) ); 
P= k*[R', -R'*c(1:3)'];
%Proyección de los puntos 3D a la imagen
Puntos2D= (P*[Puntos3D, ones(n,1) ]')'; %en pixeles


%Estimación aproximada de la distancia del plano a la posición 
%objetivo conocida una distancia en el espacio 3D entre dos puntos:
%Aproximación: Asumo que el plano es paralelo al plano imagen
Puntos2Dk=[inv(k)*Puntos2D']';%en unidades métricas
Puntos2Dk=[Puntos2Dk(:,1)./Puntos2Dk(:,3) , Puntos2Dk(:,2)./Puntos2Dk(:,3)];
dm=sqrt((Puntos3D(1,:)-Puntos3D(2,:))*(Puntos3D(1,:)-Puntos3D(2,:))');
dp=sqrt((Puntos2Dk(1,:)-Puntos2Dk(2,:))*(Puntos2Dk(1,:)-Puntos2Dk(2,:))');
d=-dm/dp;

%Transformar puntos de la imagen a coordenadas homogéneas
Puntos2D=[Puntos2D(:,1)./Puntos2D(:,3) , Puntos2D(:,2)./Puntos2D(:,3)];
