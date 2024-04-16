function dibCamara3D(location, attributes, lado, color)
%Gonzalo Lopez 2010 Junio
%Pinta la camara 3D con una piramide en la posición que se le indique
%function dibCamara3D(location, attributes, lado, color)
%location: [x y z giro_x giro_y giro_z (grados)]
%attributes: Ej attibutes: 'r-'   'b.'
%lado: (optativo) tamaño de la camara
%color: (optativo) Ej color: [0.1 1 0.5]

if (nargin < 3),
	lado=0.5;
end

%Modelo camara (piramide vista superior): 
%           |x
%           |      
%        b-----c
%        | \  /|
%        |  \/ |
%  y ----|  e  |
%        | / \ |
%        |/   \|
%        a-----d

modelo= [-1  1  -2 1 %1%a %parte inferior
          1  1  -2 1 %2%b
          1 -1  -2 1 %3%c
         -1 -1  -2 1 %4%d         
          0  0   0 1 %5%e %parte superior
          ];
      
%Aplicar escala:      
modelo(:,1:3) = lado*modelo(:,1:3);
   
t=[location(1:3)]';
R=Rgiros(location(4:6));

camara = [ [R , t ; 0 0 0 1] * modelo']';

camara=camara(:,1:3);
cara1= camara([1 2 5],:);
cara2= camara([2 3 5],:);
cara3= camara([3 4 5],:);
cara4= camara([1 4 5],:);
cara5= camara([1 2 3 4],:);

if (nargin < 4),
    patch(cara1(:,1),cara1(:,2),cara1(:,3),'r');
    patch(cara2(:,1),cara2(:,2),cara2(:,3),'g');
    patch(cara3(:,1),cara3(:,2),cara3(:,3),'b');
    patch(cara4(:,1),cara4(:,2),cara4(:,3),'y');
    patch(cara5(:,1),cara5(:,2),cara5(:,3),[0.6 0.6 0.6]);
else
    patch(cara1(:,1),cara1(:,2),cara1(:,3),color,'EdgeColor',color );
    patch(cara2(:,1),cara2(:,2),cara2(:,3),color,'EdgeColor',color );
    patch(cara3(:,1),cara3(:,2),cara3(:,3),color,'EdgeColor',color );
    patch(cara4(:,1),cara4(:,2),cara4(:,3),color,'EdgeColor',color );
    patch(cara5(:,1),cara5(:,2),cara5(:,3),color,'EdgeColor',color );
end

