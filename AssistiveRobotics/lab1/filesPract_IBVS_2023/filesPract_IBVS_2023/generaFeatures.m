% Read Image
img = imread('./cara.jpg');
grayImg = rgb2gray(img);

% Apply a Laplacian of Gaussian filter
logImg = imfilter(grayImg, fspecial('log', [5 5], 0.5), 'replicate');
scaledLogImg = rescale(logImg);

figure();
imshow(scaledLogImg);

% Convert the filtered image to a binary image by thresholding
threshold = 0.25; % More threshold? More features
binaryImg = abs(scaledLogImg) > threshold;


[rows, columns] = find(binaryImg); % Find the coordinates of the points

figure();
imshow(img); hold on;
plot(columns, rows, 'r+'); % Plots the features as red crosses
