% Load the input image
inputImage = imread('./Fotos_luis/1_4000.png');

% Display the input image
figure;
imshow(inputImage);
title('Original Image');

% Ask the user to input the coordinates for cropping
disp('Please enter the coordinates of the top-left corner of the cropping window:');
x = 370;
y = 770;

% Define the cropping rectangle
cropRect = [x, y, 649, 649]; % Adjusted to 599x599 to fit within the boundaries

% Perform cropping
croppedImage = imcrop(inputImage, cropRect);

% Display the cropped image
figure;
imshow(croppedImage);
title('Cropped Image');

% Save the cropped image if needed
imwrite(croppedImage, './Fotos_luis/crop/1_4000.png');

disp('Cropping complete.');
