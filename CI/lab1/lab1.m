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
figure(1);
imshow(img);
title('Original Image');

% Display intermediate image (brightened for better visibility)
figure(2);
imshow(min(1, im2double(img) * 5));
title('Intermediate Image (Brightened)');

%%  2.2. LINEARIZATION %%

offset = 1023;
scale = 1 / (15600 - 1023);
linear_img = (img_double - offset) * scale;
linear_img = max(0, min(1, linear_img));

figure(3);
imshow(linear_img);
title('Linearized Image');

%% 3. DEMOSAIC %%
pattern = 'rggb';
demosaiced_img = demosaic(img, pattern);

figure(4);
imshow(demosaiced_img);
title('Demosaiced Image');

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

white_world_img = zeros(size(I_demosaic));
white_world_img(:,:,1) = balanced_red;
white_world_img(:,:,2) = balanced_green;
white_world_img(:,:,3) = balanced_blue;

figure(8);
imshow(white_world_img);
title('White balancing: White World Assumption');

% Manual white balancing
auto = input('Manual balancing: Use preset? (Y/N): ', 's');
done = 'N';
while(strcmpi(done, 'N'))
    if(strcmpi(auto, 'Y'))
        x = 5776;
        y = 3920;
        RGB_pixel = I_demosaic(y,x,:);
        done = 1;
    else
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
    end
    disp('Selected pixel values:');
    disp([I_demosaic(y,x,1), I_demosaic(y,x,2), I_demosaic(y,x,3)]);
    
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
    % imshow(min(1,manual_balancing_img*5)); %brightened intermediate result
    imshow(manual_balancing_img);
    title('White balancing: Manual Balancing');
    if(strcmpi(auto, 'N'))
        done = input('Proceed? N to try again (Y/N): ', 's');
    end
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
figure(10);
% imshow(min(1,mean_filter_img*5)); brightened intermediate result
imshow(mean_filter_img);
title('Mean filter:');

% % Median filter 

median_filter_img = medfilt3(white_balanced_img, [filter_size filter_size filter_size]);
figure(11);
%imshow(min(1,median_filter_img*5)); brightened intermediate result
imshow(median_filter_img);
title('Median filter:');

% % Gaussian filter
%
% gaussian_filter_img = zeros(size(white_balanced_img));
sigma = 0.5;
gaussian_filter_img = imgaussfilt3(white_balanced_img, sigma);
figure(12);
%imshow(min(1,gaussian_filter_img*5)); brightened intermediate result
imshow(gaussian_filter_img);
title('Gaussian filter:');

denoised_img = median_filter_img;

%% 6. Color balance

HSV_img = rgb2hsv(denoised_img);
HSV_img(:,:,2) = min(1, HSV_img(:,:,2) * 1.3);
global color_balanced_img;
color_balanced_img = hsv2rgb(HSV_img);
figure(13);
imshow(color_balanced_img);
title('Color balanced image:');

%% 7. Tone reproduction
global brightnessValue gammaValue;
% Initialize variables to store slider values
done = 0;
brightnessValue = 1.8;
gammaValue = 0.8;
exposureValue = 1;

figure(14);
imshow(color_balanced_img);
title('Tone reproduction: select values for brightness and gamma:');

% Create text labels for sliders and initialize value labels
brightnessLabel = uicontrol('Style', 'text', 'String', ['Brightness: ', num2str(brightnessValue)], ...
    'Position', [240, 20, 100, 20], 'HorizontalAlignment', 'left');
gammaLabel = uicontrol('Style', 'text', 'String', ['Gamma: ', num2str(gammaValue)], ...
    'Position', [240, 50, 100, 20], 'HorizontalAlignment', 'left');

% Callback function for brightness slider
brightnessCallback = @(hObject, ~) updateBrightness(hObject, brightnessLabel);
brightnessSlider = uicontrol('Style', 'slider', 'Min', 0.0, 'Max', 4.0, 'Value', 1.8, ...
    'SliderStep', [0.1 0.1], 'Position', [20, 20, 200, 20], 'Callback', brightnessCallback);

% Callback function for gamma slider
gammaCallback = @(hObject, ~) updateGamma(hObject, gammaLabel);
gammaSlider = uicontrol('Style', 'slider', 'Min', 1./2.4, 'Max', 2.4, 'Value', 0.8, ...
    'SliderStep', [0.1 0.1], 'Position', [20, 50, 200, 20], 'Callback', gammaCallback);

% Ask user for confirmation
confirmation = input('Proceed? (Y/N): ', 's');

while(strcmpi(confirmation, 'Y'))
    % Brighten the image if necessary
    scale_factor = brightnessValue * max(max(rgb2gray(color_balanced_img)));
    brightened_img = color_balanced_img * scale_factor;
    brightened_img = brightened_img * 2.^exposureValue;
    gamma_corrected_img = zeros(size(brightened_img));
    % Gamma correction
    threshold = 0.0031308;
    % Linear transformation for values below threshold
    linear_transform = 12.92 * brightened_img;
    % Non-linear transformation for values above threshold
    non_linear_transform = (1.0 + 0.055) * (brightened_img .^ (1./gammaValue)) - 0.055;
    % Apply threshold condition using logical indexing
    below_threshold = brightened_img <= threshold;
    gamma_corrected_img(below_threshold) = linear_transform(below_threshold);
    gamma_corrected_img(~below_threshold) = non_linear_transform(~below_threshold);

    % Plot the result
    figure(15);
    imshow(gamma_corrected_img);
    title('Gamma corrected image');
    if(strcmpi(auto, 'Y'))
        break
    end
    confirmation = input('Try again? (Y/N): ', 's');
end

%% 8. Compression
imwrite(gamma_corrected_img, 'IMG_0691.png');
imwrite(gamma_corrected_img, 'IMG_0691.jpg', "Quality", 60);


% Update brightness value and label
function updateBrightness(hObject, brightnessLabel)
    global brightnessValue;
    brightnessValue = get(hObject, 'Value');
    set(brightnessLabel, 'String', ['Brightness: ', num2str(brightnessValue)]);
end

% Update gamma value and label
function updateGamma(hObject, gammaLabel)
    global gammaValue;
    gammaValue = get(hObject, 'Value');
    set(gammaLabel, 'String', ['Gamma: ', num2str(gammaValue)]);
end
