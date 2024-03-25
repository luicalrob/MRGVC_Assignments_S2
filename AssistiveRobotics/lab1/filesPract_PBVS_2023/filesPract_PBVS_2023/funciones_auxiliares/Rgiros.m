function oRotGir=Rgiros(iRotGir)
%Gonzalo Lopez 2005
%
%function Rotation=Rgiros(angulosGrados)
%pasa los giros [chi, tita, fi](en grados) a matriz de rotacion R
%
%function angulosGrados=Rgiros(Rotation)
%pasa la matriz de rotacion R(3x3) a giros [chi, tita, fi](en grados)
%
%Sistema de referencia
%         |Z(fi)
%         |
%         |
%         |------- Y(tita)
%        / 
%       /
%      / X(chi)
%
%Propiedades de matriz de rotación
%   transpose(R)==inv(R) => transpose(R)*R==eye(3)
%   det(R)= +1

if size(iRotGir)==[3 3],
    R=iRotGir;
    
    Rz= atan2(R(2,1), R(1,1));
    Ry= atan2(-R(3,1),R(1,1)*cos(Rz)+R(2,1)*sin(Rz));
     Rx= atan2(R(1,3)*sin(Rz)-R(2,3)*cos(Rz), -R(1,2)*sin(Rz)+R(2,2)*cos(Rz)); 
     angulosGrados=[Rx Ry Rz]*180/pi;
    
    oRotGir=angulosGrados;    
else
    angulosGrados=iRotGir;
    

    chi=  angulosGrados(1)*pi/180; %rad
    tita= angulosGrados(2)*pi/180; %rad
    fi=   angulosGrados(3)*pi/180; %rad
        
    Rotation=[cos(fi)*cos(tita)  cos(fi)*sin(tita)*sin(chi)-sin(fi)*cos(chi)  cos(fi)*sin(tita)*cos(chi)+sin(fi)*sin(chi);
              sin(fi)*cos(tita)  sin(fi)*sin(tita)*sin(chi)+cos(fi)*cos(chi)  sin(fi)*sin(tita)*cos(chi)-cos(fi)*sin(chi);
              -sin(tita)         cos(tita)*sin(chi)                           cos(tita)*cos(chi)                        ];
    
    oRotGir=Rotation;
end   