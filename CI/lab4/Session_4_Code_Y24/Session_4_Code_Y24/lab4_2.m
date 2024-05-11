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
params.data.file = "planes_d=0.5_l=[16x16]_s=[16x16].hdf5";
params.data.path = fullfile(params.data.folder,params.data.file);
params.rec.disc_env_size = [1,1,1]*volSize;
params.saveFolder = "../images_comp";
load("volshow_config.mat");
mkdir(params.saveFolder);
data = load_hdf5_dataset(params.data.path);
[~,o_filename,~] = fileparts(params.data.path);
disp(o_filename);

%% RECONSTRUCTION
tic
if params.isConfocal
    fprintf("Confocal reconstruction");
    confocal_rec_fast()
else
    fprintf("Normal reconstruction");
    normal_rec_fast() 
end
toc
%% Laplacian filter
f_lap = fspecial3('lap');
G_lap = imfilter(rec.G,-f_lap,'symmetric');

volumeViewer(rec.G);
%volshow(rec.G, volshow_config);
filename = strcat(o_filename,"_",num2str(volSize),"_nf",".jpg");
filename = fullfile(params.saveFolder,filename);
imwrite(getframe(gcf).cdata, filename)
close all;
volumeViewer(G_lap);
filename = strcat(o_filename,"_",num2str(volSize),"_f",".jpg");
filename = fullfile(params.saveFolder,filename);
imwrite(getframe(gcf).cdata, filename)
close all;

%% Functions

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

function normal_rec_fast()

global params
global rec
global data

rec.G = zeros(params.rec.disc_env_size);
rec.G_c = gen_voxel_coords();
s_s = size(data.spadPositions);
nsp = s_s(1)*s_s(2);
l_s = size(data.laserPositions);
nlp = l_s(1)*l_s(2);

n1s = reshape(data.laserNormals,1,[],3);
n3s = reshape(data.spadNormals,1,[],3);

v1s = reshape(reshape(data.laserOrigin, 1,1,3)-data.laserPositions,1,[],3);
d1s = sqrt(sum(v1s.^2,3));
v1s = v1s ./ d1s;
cos1s = abs(sum(n1s.*v1s,3));
d4s = sqrt(sum(reshape(data.spadPositions - reshape(data.spadOrigin, 1,1,3),1,[],3).^2,3));


for i_v = 1:params.rec.disc_env_size(1) % loop over x
    for j_v = 1:params.rec.disc_env_size(1) % loop over y
        for k_v = 1:params.rec.disc_env_size(1) % loop over z
            x_v = reshape(rec.G_c(i_v,j_v,k_v,:),[1,1,3]);
            
            d2s = sqrt(sum(reshape(data.laserPositions - x_v,1,[],3).^2,3));
            
            v3s = reshape(x_v-data.spadPositions,1,[],3);
            
            d3s = sqrt(sum(v3s.^2,3));
            v3s = v3s ./ d3s;
            cos3s = abs(sum(n3s.*v3s,3));
            cos3s(cos3s<0.5) = 0.5;
            d12s = d1s+d2s;
            d34s = d3s+d4s;
                        
            t=reshape(repmat(d34s,[nlp,1]),1,[])+repmat(d12s,[1,nsp]); % this allows to avoid the use of for loops to iterate over relay positions
            
            d2s_ = repmat(d2s,[1,nsp]); % this allows to avoid the use of for loops to iterate over relay positions
            d3s_ = reshape(repmat(d3s,[nlp,1]),1,[]); % this allows to avoid the use of for loops to iterate over relay positions
            cos1s_ = repmat(cos1s,[1,nsp]); % this allows to avoid the use of for loops to iterate over relay positionsb
            cos3s_ = reshape(repmat(cos3s,[nlp,1]),1,[]); % this allows to avoid the use of for loops to iterate over relay positions
            
            t = uint32((t-data.t0)/data.deltaT); % from distances to time
            access_index = uint32(1:(nlp*nsp)); % creating absolute indexes to access the data in the data array.
            access_index = access_index + uint32(t-1) * (nlp*nsp);
            h_info = data.data(access_index); % raw data from the dataset
            if(params.correctAttenuation)
                %fprintf("Attenuation compensation");
                h_info = h_info .* d2s_.*d3s_ ./ cos1s_./cos3s_; % apply corrections
            end
            
            rec.G(i_v,j_v,k_v) = sum(h_info,'all'); % generate value for the pixel
        end
    end
end

end

function confocal_rec_fast()
global params
global rec
global data

rec.G = zeros(params.rec.disc_env_size);
rec.G_c = gen_voxel_coords();
s_s = size(data.spadPositions);
nsp = s_s(1)*s_s(2); % number of spad/laser positions

v1s = reshape(reshape(data.laserOrigin, 1,1,3)-data.laserPositions,1,[],3); % vectors
n1s = reshape(data.laserNormals,1,[],3); % normals to calculate the firts cosine component
n3s = reshape(data.spadNormals,1,[],3);  % normals to calculate the second cosine component

d1s = sqrt(sum(v1s.^2,3)); % distances 1
v1s = v1s ./ d1s; % normalization
cos1s = sum(n1s.*v1s,3); % cosine term of d1

v4s = reshape(reshape(data.spadOrigin, 1,1,3) - data.spadPositions,1,[],3); % vectors 4 (from relay to spad)
d4s = sqrt(sum(v4s.^2,3)); % distances 4 (from relay to spad)


for i_v = 1:params.rec.disc_env_size(1) % loop over x
    for j_v = 1:params.rec.disc_env_size(2) % loop over y
        for k_v = 1:params.rec.disc_env_size(3) % loop over z
            x_v = reshape(rec.G_c(i_v,j_v,k_v,:),[1,1,3]);    
            
            v3s = reshape(x_v-data.spadPositions,1,[],3); % vectors
            
            d2s = sqrt(sum(v3s.^2,3)); % distances 2 (from wall to scene)
            d3s = d2s; % confocal reco: distances 3 equal distances 2 (from scene to relay wall)
            
            v3s = v3s./d3s; % normalizaton
            cos3s = sum(n3s.*v3s,3); % cosine term of d3
            cos3s(cos3s<0.5) = 0.5;
            t = d1s + d2s + d3s + d4s;

            t = uint32((t-data.t0)/data.deltaT);
            access_index = uint32(1:(nsp));
            access_index = access_index + uint32(t-1) * (nsp);
            h_info=data.data(access_index);
            if(params.correctAttenuation)
                %fprintf("Attenuation compensation");
                h_info = h_info .* d2s.*d3s./ cos1s./cos3s;
            end
            
            rec.G(i_v,j_v,k_v) = sum(h_info,'all');

        end
    end
end

end