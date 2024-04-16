function [Puntos3D,n]=generaEscena3D(paso)
%function [Puntos3D,n]=generaEscenaPlana(paso)
%Gonzalo Lopez Nicolas Agosto 2022
%Crea un objeto plano (en el plano x-y) de n puntos x-y-z en Puntos3D

if nargin<1
    paso=50;
end
%Centrados en cero
lado=500;
vlado=[-lado/2:paso:lado/2]';
unos=ones(length(vlado),1);
zdepth=-500;
Dzdepth=100;
cuadrado=[vlado,        unos*lado/2, zdepth*unos
          unos*lado/2,  vlado,       zdepth*unos
          -vlado,      -unos*lado/2, zdepth*unos
          -unos*lado/2 -vlado,       zdepth*unos       ];
unosc=ones(size(cuadrado,1),1);

%rombo
R=[cosd(45), -sind(45); sind(45), cosd(45) ];
rombo=[ [R*cuadrado(:,1:2)' *sqrt(2)/2]', cuadrado(:,3)+Dzdepth];

%circulo
pasoR=[-pi:1/length(vlado):pi];
unosR=ones(size(pasoR,2),1);
circulo=[cos(pasoR)'*lado/2 sin(pasoR)'*lado/2 , zdepth*unosR+Dzdepth];

%equis
equis=[vlado,  vlado, zdepth*unos+Dzdepth
       vlado, -vlado, zdepth*unos+Dzdepth];
unosx=ones(size(equis,1),1);

Puntos3D=[cuadrado + [ unosc*lado/2,  unosc*lado/2, unosc*0]
          rombo + [ unosc*lado/2,  unosc*lado/2, unosc*0]
          cuadrado + [ unosc*lado/2, -unosc*lado/2, unosc*0]
          -lado -lado zdepth
          circulo + [-unosR*lado/2, -unosR*lado/2, unosR*0]
          equis + [-unosx*lado/2, unosx*lado/2, unosx*0]
          cuadrado + [-unosc*lado/2,  unosc*lado/2, unosc*0]
          cuadrado + [-unosc*lado/2, -unosc*lado/2, unosc*0]
          ];

Puntos3D(:,1)=Puntos3D(:,1)+500;
n=size(Puntos3D,1);