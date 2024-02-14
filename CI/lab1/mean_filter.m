function output_image = mean_filter(input_image, filter_size)
    % Input:
    % input_image: Input grayscale image
    % filter_size: Size of the filter (odd integer)
    % Output:
    % output_image: Image after mean filtering
    
    % Define mean filter kernel
    mean_kernel = ones(filter_size) / filter_size^2;
    
    % Apply mean filter using imfilter
    output_image = imfilter(input_image, mean_kernel, 'replicate');
end
