clear all;
%% PARAMETERS
global params
global data
global rec

volSize = 32;
disp(volSize)
params.isConfocal = false;
params.correctAttenuation = true;

params.data.folder = "../data";
params.data.file = "Z_d=0.5_l=[1x1]_s=[256x256].hdf5";
params.data.path = fullfile(params.data.folder,params.data.file);
params.rec.disc_env_size = [1,1,1]*volSize;
load("volshow_config.mat");
data = load_hdf5_dataset(params.data.path);
[~,o_filename,~] = fileparts(params.data.path);
disp(o_filename);

%% RECONSTRUCTION
tic
if params.isConfocal
    fprintf("Confocal reconstruction");
    confocal_reconstruction_fast()
else
    fprintf("Normal reconstruction");
    reconstruction_fast() 
end
toc

%% Laplacian filter
f_lap = fspecial3('lap');
G_lap = imfilter(rec.G,-f_lap,'symmetric');


% Before the filter
volumeViewer(rec.G);
%volshow(rec.G, volshow_config);
close all;

% After the filter
volumeViewer(G_lap);
close all;




