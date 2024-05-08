function G = backProjectionReconstruction(dataset)
    %volumeSize_rounded = ceil(dataset.volumeSize);
    
    % Initialize reconstructed volume
    %G = zeros(volumeSize_rounded, volumeSize_rounded, volumeSize_rounded);
    G = zeros(16, 16, 16);
    
    %calcular tiempo/inidices a partir de lo que hace cosas
    %imagesc(squeeze(dataset.data(1, 1, :, :, 1000)))
    % Iterate over every voxel in the hidden scene
    for x = 1:size(G, 1)
        for y = 1:size(G, 2)
            for z = 1:size(G, 3)
                % Initialize voxel value
                G(x, y, z) = 0;
                
                % Iterate over laser positions
                for xl_idx = 1:size(dataset.laserPositions, 1)
                    xl = dataset.laserPositions(xl_idx, xl_idx, :);  
        
                    % Iterate over SPAD positions
                    for xs_idx = 1:size(dataset.spadPositions, 1)
                        xs = dataset.spadPositions(xs_idx, xs_idx, :);
                        
                        % Calculate time-of-flight for third-bounce light paths
                        tv = calculateTimeOfFlight(xl, xs, dataset);
                        
                        % Update reconstructed voxel value using back-projection
                        %tv
                        H_value = squeeze(dataset.data(xl_idx, xl_idx, xs_idx, xs_idx, tv));
                        G(x, y, z)  = G(x, y, z) + H_value;
                    end
                end
            end
        end
    end
    % Apply Laplacian filter to improve reconstruction quality
    f_lap = fspecial('lap');
    G = imfilter(G, -f_lap, 'symmetric');
end

function tv = calculateTimeOfFlight(xl, xs, dataset)
    % Calculate distances
    d1 = sqrt(sum((dataset.laserOrigin - xl).^2, 'all'));
    d2 = sqrt(sum((xl - xs).^2, 'all'));
    d3 = sqrt(sum((dataset.spadOrigin - xs).^2, 'all'));
    
    % Calculate time-of-flight
    tv = round((d1 + d2 + d3 - dataset.t0) / dataset.deltaT);
end
