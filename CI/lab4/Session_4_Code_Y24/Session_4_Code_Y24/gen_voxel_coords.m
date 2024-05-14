function vox_coords = gen_voxel_coords()
    global params
    global data
    n_steps = params.rec.disc_env_size(1);
    step_size = data.volumeSize / n_steps;
    
    x_0 = data.volumePosition(1) - step_size*(0.5+n_steps/2);
    y_0 = data.volumePosition(2) - step_size*(0.5+n_steps/2);
    z_0 = data.volumePosition(3) - step_size*(0.5+n_steps/2);
    
    vox_coords = zeros([n_steps,n_steps,n_steps,3]);
    for i = 1:n_steps
        for j = 1:n_steps
            for k = 1:n_steps
                vox_coords(i,j,k,1) = x_0 + i*step_size;
                vox_coords(i,j,k,2) = y_0 + j*step_size;
                vox_coords(i,j,k,3) = z_0 + k*step_size;
            end
        end
    end
end