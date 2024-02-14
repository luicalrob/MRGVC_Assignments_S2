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

% NEAREST NEIGHBOUR
nn_img_red = interpolate_nn(red_channel, 'red');
nn_img_blue = interpolate_nn(blue_channel, 'blue');
nn_img_green = interpolate_nn(green_channel, 'green');

I_demosaic_nn(:,:,1) = nn_img_red;
I_demosaic_nn(:,:,2) = nn_img_green;
I_demosaic_nn(:,:,3) = nn_img_blue;

% BILINEAR
bilinear_img_red = interpolate_bilinear(red_channel);
bilinear_img_blue = interpolate_bilinear(blue_channel);
bilinear_img_green = interpolate_bilinear(green_channel);

I_demosaic_b(:,:,1) = bilinear_img_red;
I_demosaic_b(:,:,2) = bilinear_img_green;
I_demosaic_b(:,:,3) = bilinear_img_blue;

figure(5);
imshow(I_demosaic_nn);
title('Nearest neighbour Interpolation (Manual)');

figure(6);
imshow(I_demosaic_b);
title('Bilinear Interpolation (Manual)');

I_demosaic = I_demosaic_b; % choose the demosaic method
%% 2.4 White balancing %%

% Grey world assumption

% Compute per-channel average

R_avg = mean(I_demosaic(:,:,1), [1 2]);
G_avg = mean(I_demosaic(:,:,2), [1 2]);
B_avg = mean(I_demosaic(:,:,3), [1 2]);

% Normalize each channel by its average and by green channel average
balanced_red = G_avg./R_avg * I_demosaic(:,:,1);
balanced_green = I_demosaic(:,:,2);
balanced_blue = G_avg./B_avg * I_demosaic(:,:,3);

grey_world_img(:,:,1) = balanced_red;
grey_world_img(:,:,2) = balanced_green;
grey_world_img(:,:,3) = balanced_blue;

figure(7);
imshow(grey_world_img);
title('White balancing: Grey World Assumption');

% White world assumption

% Compute per-channel max

R_max = max(I_demosaic(:,:,1), [], 'all');
G_max = max(I_demosaic(:,:,2), [], 'all');
B_max = max(I_demosaic(:,:,3), [], 'all');

% Normalize each channel by its max and by green channel max
balanced_red = G_max./R_max * I_demosaic(:,:,1);
balanced_green = I_demosaic(:,:,2);
balanced_blue = G_max./B_max * I_demosaic(:,:,3);

white_world_img(:,:,1) = balanced_red;
white_world_img(:,:,2) = balanced_green;
white_world_img(:,:,3) = balanced_blue;

figure(8);
imshow(white_world_img);
title('White balancing: White World Assumption');

% Manual white balancing

% Point that looks grey in the raw image (2917, 769)
% Point that looks white in the raw image (5501, 302)

RGB_pixel = I_demosaic(3446,2350,:);

k_r = 0.5 ./ RGB_pixel(:,:,1);
k_g = 0.5 ./ RGB_pixel(:,:,2);
k_b = 0.5 ./ RGB_pixel(:,:,3);

balanced_red = k_r * I_demosaic(:,:,1);
balanced_green = k_g * I_demosaic(:,:,2);
balanced_blue = k_b * I_demosaic(:,:,3);

manual_balancing_img(:,:,1) = balanced_red;
manual_balancing_img(:,:,2) = balanced_green;
manual_balancing_img(:,:,3) = balanced_blue;

figure(9);
imshow(manual_balancing_img);
title('White balancing: Manual Balancing');