%% 2.1 READING THE IMAGE %%

clear all;
close all;

img = imread('IMG_0691.tiff');

info = imfinfo('IMG_0691.tiff');
bits_per_pixel = info.BitDepth;
width = info.Width;
height = info.Height;

fprintf("bits per pixel: %d \n",bits_per_pixel);
fprintf("width: %d \n",width);
fprintf("height: %d \n",height);

img_double = double(img);
img_double_01 = im2double(img);
% Display original image
%figure(1);
%imshow(img_double_01);
%title('Original Image');

% Display intermediate image (brightened for better visibility)
%figure(2);
%imshow(min(1, img_double_01 * 5));
%title('Intermediate Image (Brightened)');


%%  2.2. LINEARIZATION %%

offset = 1023;
scale = 1 / (15600 - 1023);
linear_img = (img_double - offset) * scale;
linear_img = max(0, min(1, linear_img));

% figure(3);
% imshow(linear_img);
% title('Linearized Image');

%% 2.3 DEMOSAIC %%
pattern = 'rggb';
demosaiced_img = demosaic(img, pattern);

% figure(4);
% imshow(demosaiced_img);
% title('Demosaiced Image');

% create masks
bayer_red = repmat([1 0; 0 0], ceil(height/2),ceil(width/2));
bayer_green = repmat([0 1; 1 0], ceil(height/2),ceil(width/2));
bayer_blue = repmat([0 0; 0 1], ceil(height/2),ceil(width/2));

% Extracting the red, green, and blue components of the image using the mask
red_channel = linear_img .* bayer_red;
blue_channel = linear_img .* bayer_blue;
green_channel = linear_img .* bayer_green;

bilinear_img_red = interpolate_bilinear(red_channel);
bilinear_img_blue = interpolate_bilinear(blue_channel);
bilinear_img_green = interpolate_bilinear(green_channel);
bilinear_img = cat(3, bilinear_img_red, bilinear_img_green, bilinear_img_blue);

I_demosaic(:,:,1) = bilinear_img_red;
I_demosaic(:,:,2) = bilinear_img_green;
I_demosaic(:,:,3) = bilinear_img_blue;

figure(5);
imshow(I_demosaic);
title('Bilinear Interpolation (Manual)');