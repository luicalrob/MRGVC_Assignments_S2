%dataset = load_hdf5_dataset('Z_l[0.00,-0.50,0.00]_r[1.57,0.00,3.14]_v[0.81,0.01,0.81]_s[256]_l[1]_gs[1.00].hdf5');
reconstructed_volume = backProjectionReconstruction(dataset);

load("volshow_config.mat");

% Display the reconstructed volume
volshow(reconstructed_volume, volshow_config);