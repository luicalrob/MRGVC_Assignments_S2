function [Puntos3D,n]=generaEscenaPlanaIMG(paso)
%function [Puntos3D,n]=generaEscenaPlana(paso)
%Gonzalo Lopez Nicolas Agosto 2022
%Crea un objeto plano (en el plano x-y) de n puntos x-y-z en Puntos3D

    if nargin < 1
        paso = 50;
    end

    zdepth=-500;
    
    img = imread('cara.jpg');
    imshow(img);
    title('Seleccione puntos en la imagen, presione Enter cuando termine');
    
    hold on;
    title('Seleccione puntos en la imagen, presione Enter cuando termine');
    
    % Inicializar matriz para los puntos seleccionados
    Puntos3D = [];
    
    % Selección de puntos con visualización en tiempo real
    while true
        [x, y, button] = ginput(1);
        if isempty(button) || button == 13 % Enter key
            break;
        end
        plot(x, y, 'r+', 'MarkerSize', 10, 'LineWidth', 2);
        Puntos3D = [Puntos3D; x, y, zdepth];
    end
    hold off;
    
    % Número total de puntos
    n = size(Puntos3D, 1);
end