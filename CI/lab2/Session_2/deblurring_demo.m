close all;
clear all;
% Read data
aperture = imread('apertures/circular.bmp');
image = imread('images/burano.jpg');
image = image(:, :, 1);

% Noise level (Gaussian noise)
sigma = 0.005;

% Blur size
blurSize = 7;


f0 = im2double(image);
[height, width, channel] = size(f0);

% Prior matrix: 1/f law
A_star = eMakePrior(height, width) + 0.00000001;
C = sigma.^2 * height * width ./ A_star;

% Normalization
temp = fspecial('disk', blurSize);
flow = max(temp(:));

% Calculate effective PSF
k1 = im2double(...
    imresize(aperture, [2*blurSize + 1, 2*blurSize + 1], 'nearest')...
);

k1 = k1 * (flow / max(k1(:)));

% Apply blur
f1 = zDefocused(f0, k1, sigma, 0);

% Recover
f0_hat_wnr = zDeconvWNR(f1, k1, C);
f0_hat_lucy = deconvlucy(f1,k1);
f0_hat_wnr_noprior = deconvwnr(f1,k1);

% Display results

%% WNR prior
figure;

subplot_tight(1, 3, 1, 0.0, false)
imshow(f0);
title('Focused');

subplot_tight(1, 3, 2, 0.0, false)
imshow(f1);
title('Defocused');

subplot_tight(1, 3, 3, 0.0, false)
imshow(f0_hat_wnr);
title('Recovered Wiener');

%% Lucy
figure;

subplot_tight(1, 3, 1, 0.0, false)
imshow(f0);
title('Focused');

subplot_tight(1, 3, 2, 0.0, false)
imshow(f1);
title('Defocused');

subplot_tight(1, 3, 3, 0.0, false)
imshow(f0_hat_lucy);
title('Recovered Lucy');

%% Wnr no priors
figure;

subplot_tight(1, 3, 1, 0.0, false)
imshow(f0);
title('Focused');

subplot_tight(1, 3, 2, 0.0, false)
imshow(f1);
title('Defocused');

subplot_tight(1, 3, 3, 0.0, false)
imshow(f0_hat_wnr_noprior);
title('Recovered Wiener (no prior)');
