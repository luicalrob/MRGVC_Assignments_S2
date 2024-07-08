function reconstruction_fast()

global params
global rec
global data

rec.G = zeros(params.rec.disc_env_size);
rec.G_c = gen_voxel_coords();
s_s = size(data.spadPositions);
numSpadPositions = s_s(1)*s_s(2);
laserSize = size(data.laserPositions);
numLaserPositions = laserSize(1)*laserSize(2);

n1s = reshape(data.laserNormals,1,[],3);
n3s = reshape(data.spadNormals,1,[],3);

v1s = reshape(reshape(data.laserOrigin, 1,1,3)-data.laserPositions,1,[],3);
d1s = sqrt(sum(v1s.^2,3));
v1s = v1s ./ d1s;
cos1s = abs(sum(n1s.*v1s,3));
d4s = sqrt(sum(reshape(data.spadPositions - reshape(data.spadOrigin, 1,1,3),1,[],3).^2,3));


for i = 1:params.rec.disc_env_size(1) % loop over x
    for j = 1:params.rec.disc_env_size(1) % loop over y
        for k = 1:params.rec.disc_env_size(1) % loop over z
            x_v = reshape(rec.G_c(i,j,k,:),[1,1,3]);
            
            d2s = sqrt(sum(reshape(data.laserPositions - x_v,1,[],3).^2,3));
            v3s = reshape(x_v-data.spadPositions,1,[],3);
            d3s = sqrt(sum(v3s.^2,3));
            v3s = v3s ./ d3s;

            d12s = d1s+d2s;
            d34s = d3s+d4s;
            cos3s = abs(sum(n3s.*v3s,3));
            cos3s(cos3s<0.5) = 0.5;
               
            % repmat allows to avoid the use of for loops to iterate over relay positions
            t=reshape(repmat(d34s,[numLaserPositions,1]),1,[])+repmat(d12s,[1,numSpadPositions]);
            
            % repmat allows to avoid the use of for loops to iterate over relay positions
            d2s_repmat = repmat(d2s,[1,numSpadPositions]); 
            d3s_repmat = reshape(repmat(d3s,[numLaserPositions,1]),1,[]);
            cos1s_repmat = repmat(cos1s,[1,numSpadPositions]);
            cos3s_repmat = reshape(repmat(cos3s,[numLaserPositions,1]),1,[]);
            
            t = uint32((t-data.t0)/data.deltaT); % get time from distances
            access_index = uint32(1:(numLaserPositions*numSpadPositions)); % creating absolute indexes to access the data in the data array.
            access_index = access_index + uint32(t-1) * (numLaserPositions*numSpadPositions);
            h_info = data.data(access_index); % Histogram data from the dataset

            if(params.correctAttenuation)
                h_info = h_info .* d2s_repmat.*d3s_repmat ./ cos1s_repmat./cos3s_repmat; % attenuation corrections
            end
            
            rec.G(i,j,k) = sum(h_info,'all'); % generate pixel value
        end
    end
end

end
