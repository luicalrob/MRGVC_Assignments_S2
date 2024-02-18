%-------------------------------------------------------
function map = erase_features (map, unreliable),
%-------------------------------------------------------

for i = 1:length(unreliable)    
        % remove
        map.x(unreliable(i)-(i-1), :) = [];
        map.P(unreliable(i)-(i-1), :) = [];
        map.P(:, unreliable(i)-(i-1)) = [];

        map.ground_id(:, unreliable(i)-(i-1)) = [];
        map.hits(unreliable(i)-(i-1), :) = [];
        map.first(unreliable(i)-(i-1), :) = [];
        % map.covisibility(unreliable(i)-(i-1), :) = [];
        % map.covisibility(:, unreliable(i)-(i-1)) = [];
        % map.estimated(end+1).x = map.x(1:3);
        % map.estimated(end).P = map.P(1:3,1:3);

        map.n = map.n - 1;

    end

