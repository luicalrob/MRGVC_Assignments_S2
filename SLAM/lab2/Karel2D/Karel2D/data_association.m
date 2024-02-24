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
best_prediction = prediction;
best_compatibility = compatibility;
global chi2;

n_hyp = 1000;
w = 0.9; % probability of a measure to be good
z = 0.05; % probability of failure
t = ceil(log(z) / log(1-w)); % number of iterations
iteration = 0;
max_voted = 0;

while iteration < t
    rnd_index = randi(length(j_ic)); % select a random feature with 1 or more compatibilities
    j_rnd = j_ic(rnd_index);
    obsv_compatible = find(compatibility.ic(:,j_rnd) == 1);

    %H_hyp = JCBB_RANSAC (prediction, observations, compatibility, j_rnd);

    for i = 1:length(obsv_compatible)
        H_hyp = zeros(1, observations.m);
        H_hyp(obsv_compatible(i)) = j_rnd;
        map_hyp = EKF_update (map, observations, H_hyp);

        prediction_hyp = predict_observations (map_hyp);
        compatibility_hyp = compute_compatibility (prediction_hyp, observations);

        columns_with_only_ceros = find(all(compatibility_hyp.ic == 0));
        votes = size(compatibility_hyp.ic,2) - length(columns_with_only_ceros) - 1;
        if max_voted < votes
            max_voted = votes;
            best_prediction = prediction_hyp;
            best_compatibility = compatibility_hyp;
        end
    end
    
    iteration = iteration + 1;
end

%best_compatibility = compute_compatibility (best_prediction, observations);
H = NN(best_prediction, observations, best_compatibility);
% H = JCBB (prediction, observations, compatibility);

disp(['MY HYPOTHESIS: ' sprintf('%2d  ', H)]);
disp(['Correct (1/0)? ' sprintf('%2d  ', GT == H)]);
disp(' ');

draw_map (map, ground, step);
draw_observations (observations, ground, step);

draw_compatibility (prediction, observations, compatibility);

draw_hypothesis (prediction, observations, compatibility, H, configuration.name, 'b-');
draw_tables (compatibility, GT, H);
