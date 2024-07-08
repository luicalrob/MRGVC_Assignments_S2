
% Read data
aperture_names = {'circular', 'Levin', 'raskar', 'zhou'};
aperture_images = cell(1, numel(aperture_names));

for i = 1:numel(aperture_names)
    aperture_images{i} = imread(['apertures/', aperture_names{i}, '.bmp']);
end

image = imread('images/burano.jpg');
image = image(:, :, 1);

% Noise level (gaussian noise)
sigma = 0.005;

% Blur size
blurSize = 7;

f0 = im2double(image);
[height, width, ~] = size(f0);

% Prior matrix: 1/f law
A_star = eMakePrior(height, width) + 0.00000001;
C = sigma.^2 * height * width ./ A_star;

% Normalization
temp = fspecial('disk', blurSize);
flow = max(temp(:));

% Image power spectra
F0 = fft2(f0);
F0 = fftshift(F0 .* conj(F0));

S0 = log(F0);
S0_X = S0(:, round(length(S0)/2) + 1);
S0_Y = S0(round(length(S0)/2) + 1, :);

% Display results
figure(2);

% Plot original X and Y in the first row
subplot(5, 2, 1)
plot(linspace(-1, 1, length(S0_X)), S0_X)
grid('on')
title('Original X');

subplot(5, 2, 2)
plot(linspace(-1, 1, length(S0_Y)), S0_Y)
grid('on')
title('Original Y');



% Initialize figure
figure(1);
for i = 1:numel(aperture_names)
    aperture = aperture_images{i};
    
    % Calculate effective PSF
    k1 = im2double(...
        imresize(aperture, [2*blurSize + 1, 2*blurSize + 1], 'nearest')...
    );

    k1 = k1 * (flow / max(k1(:)));

    % Apply blur
    f1 = zDefocused(f0, k1, sigma, 0);

    % Padding aperture
    k1P = zPSFPad(k1, max(height, width), max(height, width));

    % Aperture power spectra
    F = fft2(k1P);
    F = fftshift(F .* conj(F));

    S = log(F);
    S_X = S(:, round(length(S)/2) + 1);
    S_Y = S(round(length(S)/2) + 1, :);
    
    figure(1);
    % Display results
    subplot(4, 4, (i-1)*4 + 1)
    imagesc(k1P);
    axis('image')
    axis('on')
    title('Aperture');
    h = ylabel(aperture_names{i});
    set(h, 'FontSize', 10, 'FontWeight', 'bold');
    % Zoom out the plot
    ax = gca;
    ax.XLim = [240 270]; % Adjust according to your data dimensions
    ax.YLim = [240 270]; % Adjust according to your data dimensions
    set(ax, 'YTick', [], 'YTickLabel', [])
    set(ax, 'XTick', [], 'XTickLabel', [])


    subplot(4, 4, (i-1)*4 + 2)
    imagesc(S);
    axis('image')
    axis('off')
    title('Aperture Frec');

    subplot(4, 4, (i-1)*4 + 3)
    plot(linspace(-1, 1, length(S_X)), S_X)
    grid('on')
    title('Normalized f_x');


    subplot(4, 4, (i-1)*4 + 4)
    plot(linspace(-1, 1, length(S_Y)), S_Y)
    grid('on')
    title('Normalized f_y');

    F1 = fft2(f1);
    F1 = fftshift(F1 .* conj(F1));
    
    S1 = log(F1);
    S1_X = S1(:, round(length(S1)/2) + 1);
    S1_Y = S1(round(length(S1)/2) + 1, :);
    
    figure(2);
    subplot(5, 2, (i-1)*2 + 3)
    plot(linspace(-1, 1, length(S1_X)), S1_X)
    grid on
    title('Defocused X')
    h = ylabel(aperture_names{i});
    set(h, 'FontSize', 10, 'FontWeight', 'bold');

    subplot(5, 2, (i-1)*2 + 4)
    plot(linspace(-1, 1, length(S1_Y)), S1_Y)
    grid on
    title('Defocused Y')
end


function outK = zPSFPad(inK, height, width)
    % This function is to zeropadding the psf

    [sheight, swidth] = size(inK);

    outK = zeros(height, width);

    outK(floor(end/2-sheight/2) + 1:floor(end/2-sheight/2) + sheight, ...
         floor(end/2-swidth/2) + 1:floor(end/2-swidth/2) + swidth) = inK;
end