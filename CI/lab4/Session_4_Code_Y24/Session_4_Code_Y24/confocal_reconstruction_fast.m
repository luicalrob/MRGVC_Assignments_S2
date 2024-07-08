function confocal_reconstruction_fast()
global params
global rec
global data

rec.G = zeros(params.rec.disc_env_size);
rec.G_c = gen_voxel_coords();
spadSize = size(data.spadPositions);
numSpadPositions = spadSize(1)*spadSize(2); % number of spad/laser positions

v1s = reshape(reshape(data.laserOrigin, 1,1,3)-data.laserPositions,1,[],3); % vectors
n1s = reshape(data.laserNormals,1,[],3); % normals to calculate the firts cosine component
n3s = reshape(data.spadNormals,1,[],3);  % normals to calculate the second cosine component

d1s = sqrt(sum(v1s.^2,3)); % distances 1
v1s = v1s ./ d1s; % normalization
cos1s = sum(n1s.*v1s,3); % cosine term of d1

v4s = reshape(reshape(data.spadOrigin, 1,1,3) - data.spadPositions,1,[],3); % vectors 4 (from relay to spad)
d4s = sqrt(sum(v4s.^2,3)); % distances 4 (from relay to spad)


for i = 1:params.rec.disc_env_size(1) % loop over x
    for j = 1:params.rec.disc_env_size(2) % loop over y
        for k = 1:params.rec.disc_env_size(3) % loop over z
            x_v = reshape(rec.G_c(i,j,k,:),[1,1,3]);    
            
            v3s = reshape(x_v-data.spadPositions,1,[],3); % vectors
            
            d2s = sqrt(sum(v3s.^2,3)); % distances 2 (from wall to scene)
            d3s = d2s; % confocal: distances 3 equal distances 2 (from scene to relay wall)
            
            v3s = v3s./d3s; % normalizaton
            cos3s = sum(n3s.*v3s,3); % cosine term of d3
            cos3s(cos3s<0.5) = 0.5;
            t = d1s + d2s + d3s + d4s;

            t = uint32((t-data.t0)/data.deltaT);
            access_index = uint32(1:(numSpadPositions));
            access_index = access_index + uint32(t-1) * (numSpadPositions);
            h_info=data.data(access_index);
            if(params.correctAttenuation)
                %fprintf("Attenuation compensation");
                h_info = h_info .* d2s.*d3s./ cos1s./cos3s;
            end
            
            rec.G(i,j,k) = sum(h_info,'all');

        end
    end
end

end