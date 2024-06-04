function [Puntos3D, n] = generaEscenaCara(img)
% Function [Puntos3D, n] = generaCaraPlana(paso)
% Gonzalo Lopez Nicolas, Agosto 2022
% Crea una representación plana de una cara en el plano x-y de n puntos x-y-z en Puntos3D

% Definir dimensiones de la cara
ancho = 1000; % ancho total de la cara en unidades
alto = 1000;  % alto total de la cara en unidades
profundidad = -500;  % Profundidad constante para todos los puntos
n = 68;  % Número total de puntos según el diseño estándar

grayImg = rgb2gray(img);

% Apply a Laplacian of Gaussian filter
logImg = imfilter(grayImg, fspecial('log', [5 5], 0.5), 'replicate');
scaledLogImg = rescale(logImg);

% figure();
% imshow(scaledLogImg);

% Convert the filtered image to a binary image by thresholding
threshold = 0.25; % More threshold? More features
binaryImg = abs(scaledLogImg) > threshold;

[puntosY, puntosX] = find(binaryImg); % Find the coordinates of the points

z = profundidad * ones(size(puntosX));  % Todos los puntos están a la misma profundidad

% Crear los Puntos3D
Puntos3D = [puntosX - ancho + 200 / 2 , puntosY - alto + 200 / 2, z];  % Centering points in the same step
n=size(Puntos3D,1);
end
