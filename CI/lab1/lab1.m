%%%% READING THE IMAGE %%%%

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


%%%% LINEARIZATION %%%%

offset = 1023;
scale = 1 / (15600 - 1023);
linear_img = (img_double - offset) * scale;


linear_img(linear_img < 0) = 0;
linear_img(linear_img > 1) = 1;

%figure(3);
%imshow(linear_img);
%title('Linearized Image');


%%%% DEMOSAIC %%%%
pattern = 'rggb';
demosaiced_img = demosaic(img, pattern);

% figure(4);
% imshow(demosaiced_img);
% title('Demosaiced Image');

% Separate into color channels
red_channel = img_double(1:2:end, 1:2:end);
green_channel_1 = img_double(1:2:end, 2:2:end);
green_channel_2 = img_double(2:2:end, 1:2:end);
green_channel = (green_channel_1 + green_channel_2) / 2; % Average of two green pixels
blue_channel = img_double(2:2:end, 2:2:end);

expanded_red_channel = zeros(size(img_double));
expanded_green_channel = zeros(size(img_double));
expanded_blue_channel = zeros(size(img_double));

expanded_red_channel(1:2:end, 1:2:end) = red_channel;
expanded_green_channel(1:2:end, 2:2:end) = green_channel;
expanded_green_channel(2:2:end, 1:2:end) = green_channel;
expanded_blue_channel(2:2:end, 2:2:end) = blue_channel;

bilinear_img_red = interpolate_bilinear(expanded_red_channel);
bilinear_img_blue = interpolate_bilinear(expanded_blue_channel);
bilinear_img_green = interpolate_bilinear(expanded_green_channel);
bilinear_img = cat(3, bilinear_img_red, bilinear_img_green, bilinear_img_blue);

nn_linear_img = (bilinear_img - offset) * scale;

nn_linear_img(nn_linear_img < 0) = 0;
nn_linear_img(nn_linear_img > 1) = 1;

figure(5);
imshow(nn_linear_img);
title('Bilinear Interpolation (Manual)');