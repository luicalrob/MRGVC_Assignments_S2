close all;
clear all;

% Read data
image = imread('images/burano.jpg');
image = image(:, :, 1);
sigma = 0.01; % Noise level (Gaussian noise)
blurSize = 3; % Blur size
aperture_names = {'circular', 'Levin', 'raskar', 'zhou'}; % Names of apertures
deconv_names = {'Wiener', 'Lucy', 'Wiener (no prior)'};
numDeconv = numel(deconv_names);
numApertures = numel(aperture_names); % Number of apertures

f0 = im2double(image);
[height, width, ~] = size(f0);

% Prior matrix: 1/f law
A_star = eMakePrior(height, width) + 0.00000001;
C = sigma.^2 * height * width ./ A_star;

% Normalization
temp = fspecial('disk', blurSize);
flow = max(temp(:));

% Initialize cell arrays to store recovered images
f0_hat_wnr_cell = cell(numApertures, 1);
f0_hat_lucy_cell = cell(numApertures, 1);
f0_hat_wnr_noprior_cell = cell(numApertures, 1);

% Loop through each aperture
for i = 1:numApertures
    % Read aperture
    aperture = imread(['apertures/', aperture_names{i}, '.bmp']);
    
    % Calculate effective PSF
    k1 = im2double(imresize(aperture, [2*blurSize + 1, 2*blurSize + 1], 'nearest'));
    k1 = k1 * (flow / max(k1(:)));
    
    % Apply blur
    f1 = zDefocused(f0, k1, sigma, 0);
    
    % Recover using different methods
    f0_hat_wnr_cell{i} = zDeconvWNR(f1, k1, C);
    f0_hat_lucy_cell{i} = deconvlucy(f1, k1);
    f0_hat_wnr_noprior_cell{i} = deconvwnr(f1, k1);
end

figure();

for i = 1:numApertures

     % Focused image
    subplot_tight(numApertures, numDeconv+2, (i-1)*(numDeconv+2) + 1 , 0.025, false);
    imshow(f0);
    h = ylabel(aperture_names{i});
    set(h, 'FontSize', 10, 'FontWeight', 'bold');
    if(i == 1) 
        title('Focused Image');
    end
    
    % Defocused image
    subplot_tight(numApertures, numDeconv+2, (i-1)*(numDeconv+2) + 2, 0.025, false);
    imshow(f1);
    if(i == 1) 
        title('Defocused Image');
    end
    % Wiener with prior
    subplot_tight(numApertures, numDeconv+2, (i-1)*(numDeconv + 2) + 3, 0.025, false);
    imshow(f0_hat_wnr_cell{i});
   
    if(i == 1) 
        title([deconv_names{1}]);
    end    
    
    % Lucy
    subplot_tight(numApertures, numDeconv+2, (i-1)*(numDeconv + 2) + 4, 0.025, false);
    imshow(f0_hat_lucy_cell{i});
    if(i == 1) 
        title([deconv_names{2}]);
    end
    
    % Wiener with no prior
    subplot_tight(numApertures, numDeconv + 2, (i-1)*(numDeconv + 2) + 5, 0.025, false);
    imshow(f0_hat_wnr_noprior_cell{i});
    if(i == 1) 
        title([deconv_names{3}]);
    end
    
end

