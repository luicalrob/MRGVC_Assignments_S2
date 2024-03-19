close all;
clear all;

% Read data
aperture = imread('apertures/Levin.bmp');
image = imread('images/burano.jpg');
image = image(:, :, 1);

[height, width, channel] = size(im2double(image));

% Define parameter ranges
sigma_values = [0.001, 0.005, 0.01]; % Different noise levels (Gaussian noise)
blurSize_values = [3, 7, 11]; % Different blur sizes

% Initialize arrays to store quality metrics
psnr_values = zeros(numel(sigma_values), numel(blurSize_values));
ssim_values = zeros(numel(sigma_values), numel(blurSize_values));

% Iterate over parameter combinations
for i = 1:numel(sigma_values)
    for j = 1:numel(blurSize_values)
        sigma = sigma_values(i);
        blurSize = blurSize_values(j);
        
        % Apply blur and noise to the image
        f0 = im2double(image);
        k1 = calculate_psf(aperture, blurSize);
        f1 = zDefocused(f0, k1, sigma, 0);
        
        % Perform deblurring using Wiener deconvolution
        f0_hat_wnr = zDeconvWNR(f1, k1, calculate_prior(height, width, sigma));
        
        % Compute PSNR and SSIM
        psnr_values(i, j) = psnr(f0, f0_hat_wnr);
        ssim_values(i, j) = ssim(f0, f0_hat_wnr);
    end
end

% Visualize results

% Peak Signal-to-Noise Ratio

% PSNR measures the quality of the deblurred image by comparing it to the 
% original image in terms of signal-to-noise ratio. Higher PSNR values 
% indicate better image quality.
figure;
subplot(2, 1, 1);
imagesc(psnr_values);
colorbar;
xlabel('Blur Size');
ylabel('Noise Level');
title('PSNR');

% Structural Similarity Index

% SSIM measures the similarity between the deblurred image and the original 
% image in terms of luminance, contrast, and structure. SSIM values range 
% from -1 to 1, where 1 indicates perfect similarity.
subplot(2, 1, 2);
imagesc(ssim_values);
colorbar;
xlabel('Blur Size');
ylabel('Noise Level');
title('SSIM');


% Interpretation of Results:

% - Higher PSNR and SSIM values indicate better image quality after deblurring.
% - Analyze the heatmaps to identify regions where PSNR and SSIM values are high, 
% indicating optimal combinations of noise levels and blur sizes for effective 
% deblurring.
% - Look for trends in the heatmaps to understand how changes in noise levels 
% and blur sizes affect the quality of deblurred images.


% Calculate correlation coefficients
% For PSNR
psnr_correlation = corrcoef(sigma_values, blurSize_values);
psnr_correlation_coefficient = psnr_correlation(1, 2);

% For SSIM
ssim_correlation = corrcoef(sigma_values, blurSize_values);
ssim_correlation_coefficient = ssim_correlation(1, 2);

% The corrcoef function returns a correlation matrix, 
% and then, we extract the correlation coefficient from it. 

% If the correlation coefficients are close to 0, it 
% suggests little to no linear relationship between 
% sigma and blur size, indicating they are independent variables. 

disp(['Correlation coefficient for PSNR: ', num2str(psnr_correlation_coefficient)]);
disp(['Correlation coefficient for SSIM: ', num2str(ssim_correlation_coefficient)]);


% Helper functions
function k = calculate_psf(aperture, blurSize)
    temp = fspecial('disk', blurSize);
    flow = max(temp(:));
    k = im2double(imresize(aperture, [2*blurSize + 1, 2*blurSize + 1], 'nearest'));
    k = k * (flow / max(k(:)));
end

function prior = calculate_prior(height, width, sigma)
    A_star = eMakePrior(height, width) + 0.00000001;
    C = sigma.^2 * height * width ./ A_star;
    prior = C;
end
