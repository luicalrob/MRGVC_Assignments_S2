% Read Image
img = imread('./cara.jpg');
grayImg = rgb2gray(img);
figure(1);
imshow(img);

%% Edges detection

edges = edge(grayImg, 'Canny');
figure(2);
imshow(edges);

%% Color Thresholding
hsvImg = rgb2hsv(img);

% Define thresholds for a specific color range, e.g., red
hueRange = [0.1 1.0];
saturationMin = 0.15;  % minimum saturation
valueMin = 0.15;       % minimum value (brightness)

% Create a binary mask based on thresholds
mask = (hsvImg(:,:,1) >= hueRange(1) & hsvImg(:,:,1) <= hueRange(2)) & ...
       (hsvImg(:,:,2) >= saturationMin) & ...
       (hsvImg(:,:,3) >= valueMin);
figure(3);
imshow(mask);


%% Laplacian of Gaussian filter
logImg = imfilter(grayImg, fspecial('log', [5 5], 0.5), 'replicate');
scaledLogImg = rescale(logImg);

figure(4);
imshow(scaledLogImg);

% Convert the filtered image to a binary image by thresholding
threshold = 0.25; % More threshold? More features
binaryImg = abs(scaledLogImg) > threshold;


[rows, columns] = find(binaryImg); % Find the coordinates of the points

figure(5);
imshow(img); hold on;
plot(columns, rows, 'r+'); % Plots the features as red crosses
