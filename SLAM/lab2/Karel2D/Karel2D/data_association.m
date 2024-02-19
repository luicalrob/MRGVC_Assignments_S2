%-------------------------------------------------------
function [H, GT, compatibility] = data_association(map, observations, step),
%-------------------------------------------------------
global configuration ground;

% individual compatibility
prediction = predict_observations (map);
compatibility = compute_compatibility (prediction, observations);

% ground truth
GT = ground_solution(map, observations);
disp(['GROUND  TRUTH: ' sprintf('%2d  ', GT)]);

% your algorithm here!
% 1. Try NN
% 2. Complete SINGLES and try it
% 3. Include people and try SINGLES
% 4. Complete JCBB and try it
% 5. Try JCBB without odometry
% 6. Eliminate features included in the map two steps ago, and never seen again

%H = NN (prediction, observations, compatibility);
%H = SINGLES (prediction, observations, compatibility);

% 1 idea simple: 
% bucle
% obetner random subsets de 1 predictions and observations, es decir, uno
% Individually compatible
% sacar una H con eso usando JCBB, mi hipotesis
% obtener votos para esa H, usando mahalanobis y chiX2, medidas respecto a features
% end

[i_ic, j_ic] = find(compatibility.ic == 1);
best_H = zeros(1, observations.m);
global chi2;

n_hyp = 1000;
p = 0.99;
iteration = 0;
max_voted = 0;

while iteration < n_hyp
    votes = 0;
    rnd_index = randi(length(i_ic));
    i_rnd = i_ic(rnd_index);
    j_rnd = j_ic(rnd_index);

    H_hyp = JCBB_RANSAC (prediction, observations, compatibility, j_rnd);

    compatible = find(H_hyp == 1);
    for i = compatible
        j = H_hyp(i);
        Dij2 = compatibility.d2 (i, j);
        if Dij2 < chi2(2)
           votes = votes + 1;
        end
    end

    if max_voted < votes
        max_voted = votes;
        best_H = H_hyp;

        epsilon = 1 - max_voted / length(i_ic);
        n_hyp = log(1-p) / log(1-(1-epsilon));
    end

    iteration = iteration + 1;
end

H = best_H;
% H = JCBB (prediction, observations, compatibility);

disp(['MY HYPOTHESIS: ' sprintf('%2d  ', H)]);
disp(['Correct (1/0)? ' sprintf('%2d  ', GT == H)]);
disp(' ');

draw_map (map, ground, step);
draw_observations (observations, ground, step);

draw_compatibility (prediction, observations, compatibility);

draw_hypothesis (prediction, observations, compatibility, H, configuration.name, 'b-');
draw_tables (compatibility, GT, H);
