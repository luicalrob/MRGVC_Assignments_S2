function H = SINGLES (prediction, observations, compatibility)
%-------------------------------------------------------
% University of Zaragoza
% Authors:  J. Neira, J. Tardos
%-------------------------------------------------------
%-------------------------------------------------------
global chi2;
global configuration;

H = zeros(1, observations.m);

% You have observations.m observations, and prediction.n
% predicted features.
%
% For every observation i, check whether it has only one neighbour,
% say feature j, and whether that feature j  has only that one neighbour
% observation i.  If so, H(i) = j.
%
% You will need to check the compatibility.ic matrix
% for this:
%
% compatibility.ic(i,j) = 1 if observation i is a neighbour of
% feature j.
positions = [];
for i = 1:observations.m,
    neighbours = 0;
    for j = 1:prediction.n,
        is_neighbour = compatibility.ic (i, j);
        if is_neighbour
            neighbours = neighbours + 1;
            if neighbours == 1
                position = j;
            elseif neighbours > 1
                break;
            end
        end
    end 

    if (neighbours == 1 && not(ismember(position, positions)))
        positions = [positions, position];
        H(i) = position;
    else
        H(i) = 0;
    end
end
            
configuration.name = 'SINGLES';
