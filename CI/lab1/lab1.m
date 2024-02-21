%% 2.1 READING THE IMAGE %%

clear all;
close all;

img = imread('IMG_0691.tiff');
real_image = imread('IMG_0691.CR2');
real_image_double = double(real_image);

info = imfinfo('IMG_0691.tiff');
bits_per_pixel = info.BitDepth;
width = info.Width;
height = info.Height;

fprintf("bits per pixel: %d \n",bits_per_pixel);
fprintf("width: %d \n",width);
fprintf("height: %d \n",height);

img_double = double(img);
% Display original image
% figure(1);
% imshow(img);
% title('Original Image');

% Display intermediate image (brightened for better visibility)
% figure(2);
% imshow(min(1, im2double(img) * 5));
% title('Intermediate Image (Brightened)');

%%  2.2. LINEARIZATION %%

offset = 1023;
scale = 1 / (15600 - 1023);
linear_img = (img_double - offset) * scale;
linear_img = max(0, min(1, linear_img));

% figure(3);
% imshow(linear_img);
% title('Linearized Image');

%% 3. DEMOSAIC %%
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

% figure(5);
% imshow(I_demosaic_nn);
% title('Nearest neighbour Interpolation (Manual)');
% 
% figure(6);
% imshow(I_demosaic_b);
% title('Bilinear Interpolation (Manual)');

I_demosaic = I_demosaic_b; % choose the demosaic method
%% 4. White balancing %%

% Grey world assumption

% Compute per-channel average

R_avg = mean(I_demosaic(:,:,1), [1 2]);
G_avg = mean(I_demosaic(:,:,2), [1 2]);
B_avg = mean(I_demosaic(:,:,3), [1 2]);

% Normalize each channel by its average and by green channel average
balanced_red = G_avg./R_avg * I_demosaic(:,:,1);
balanced_green = I_demosaic(:,:,2);
balanced_blue = G_avg./B_avg * I_demosaic(:,:,3);

grey_world_img = zeros(size(I_demosaic));
grey_world_img(:,:,1) = balanced_red;
grey_world_img(:,:,2) = balanced_green;
grey_world_img(:,:,3) = balanced_blue;

% figure(7);
% imshow(grey_world_img);
% title('White balancing: Grey World Assumption');

% White world assumption

% Compute per-channel max

R_max = max(I_demosaic(:,:,1), [], 'all');
G_max = max(I_demosaic(:,:,2), [], 'all');
B_max = max(I_demosaic(:,:,3), [], 'all');

% Normalize each channel by its max and by green channel max
balanced_red = G_max./R_max * I_demosaic(:,:,1);
balanced_green = I_demosaic(:,:,2);
balanced_blue = G_max./B_max * I_demosaic(:,:,3);

white_world_img = zeros(size(I_demosaic));
white_world_img(:,:,1) = balanced_red;
white_world_img(:,:,2) = balanced_green;
white_world_img(:,:,3) = balanced_blue;

% figure(8);
% imshow(white_world_img);
% title('White balancing: White World Assumption');

% Manual white balancing
done = 0;
while(~done)
    figure(9);
    imshow(real_image);
    title('Manual Balancing: Click on the image to select points. Press Enter when done.');
    % Allow user to click points on the image
    [x, y] = ginput;
    x = round(x);
    y = round(y);
    % Perform computation (example: display selected points)
    disp('Selected pixel coordinates:');
    disp([x, y]);


    RGB_pixel = I_demosaic(y,x,:);
    disp('Selected pixel values:');
    disp([I_demosaic(y,x,1), I_demosaic(y,x,2), I_demosaic(y,x,3)]);

    %RGB_pixel = I_demosaic(3446,2350,:);
    %RGB_pixel = I_demosaic(3746,2002,:);
    
    mean_value = mean(RGB_pixel);
    k_r = mean_value ./ RGB_pixel(:,:,1);
    k_g = mean_value ./ RGB_pixel(:,:,2);
    k_b = mean_value ./ RGB_pixel(:,:,3);
    
    balanced_red = k_r * I_demosaic(:,:,1);
    balanced_green = k_g * I_demosaic(:,:,2);
    balanced_blue = k_b * I_demosaic(:,:,3);
    
    manual_balancing_img = zeros(size(I_demosaic));
    manual_balancing_img(:,:,1) = balanced_red;
    manual_balancing_img(:,:,2) = balanced_green;
    manual_balancing_img(:,:,3) = balanced_blue;
    
    figure(9);
    imshow(manual_balancing_img);
    title('White balancing: Manual Balancing');
    done = input("Write 1 to continue with selected settings. 0 to try again\n");
end
% Choose the best for rest of pipeline
white_balanced_img = manual_balancing_img;

%% 5. Denoising %%

% % Mean filter
%
filter_size = 3;
mean_filter_img = zeros(size(white_balanced_img));
for i=1:3
    mean_filter_img(:,:,i) = conv2(white_balanced_img(:,:,i), ones(filter_size)/(filter_size^2), 'same');
end
% figure(10);
% imshow(mean_filter_img);
% title('Mean filter:');

% % Median filter
% 
medianFilterKernel = ones(windowSize) / windowSize^2;
median_filter_img = zeros(size(white_balanced_img));
for i=1:3
    median_filter_img(:,:,i) = conv2(white_balanced_img(:,:,i), ones(filter_size)/(filter_size^2), 'same');
end
median_filter_img = medfilt3(white_balanced_img, [filter_size filter_size filter_size]);
% figure(11);
% imshow(median_filter_img);
% title('Median filter:');

% % Gaussian filter
%
gaussian_filter_img = zeros(size(white_balanced_img));
sigma = 0.5;
for i=1:3
    gaussian_filter_img(:,:,i) = conv2(white_balanced_img(:,:,i), ones(filter_size)/(filter_size^2), 'same');
end
gaussian_filter_img = imgaussfilt3(white_balanced_img, sigma);
% figure(12);
% imshow(gaussian_filter_img);
% title('Gaussian filter:');

denoised_img = median_filter_img;

%% 6. Color balance
figure(12);
imshow(denoised_img);
title('denoised_img:');

HSV_img = rgb2hsv(denoised_img);
HSV_img(:,:,2) = min(1, HSV_img(:,:,2) * 1.5);
color_balanced_img = hsv2rgb(HSV_img);
figure(13);
imshow(color_balanced_img);
title('Color balanced image:');

%% 7. Tone reproduction

% brighten the image if necessary
scale_factor = 1.5 * max(max(rgb2gray(color_balanced_img)));
brightened_img = color_balanced_img * scale_factor;
figure(14);
imshow(brightened_img);
title('Brightened image:');

gamma_corrected_img = zeros(size(brightened_img));
% gamma correction
gamma = 0.8;
threshold = 0.0031308;
% Linear transformation for values below threshold
linear_transform = 12.92 * brightened_img;
% Non-linear transformation for values above threshold
non_linear_transform = (1.0 + 0.055) * (brightened_img .^ (1./gamma)) - 0.055;
% Apply threshold condition using logical indexing
below_threshold = brightened_img <= threshold;
gamma_corrected_img(below_threshold) = linear_transform(below_threshold);
gamma_corrected_img(~below_threshold) = non_linear_transform(~below_threshold);

figure(15);
imshow(gamma_corrected_img);
title('Gamma corrected image:');

%% 8. Compression
imwrite(gamma_corrected_img, 'IMG_0691.png');
imwrite(gamma_corrected_img, 'IMG_0691.jpg', "Quality", 95);
