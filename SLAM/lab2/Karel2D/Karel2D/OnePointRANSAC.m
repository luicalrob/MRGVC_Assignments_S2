function H = OnePointRANSAC (map, prediction, observations, compatibility)

[~, j_ic] = find(compatibility.ic == 1);

w = 0.9; % probability of a measure to be good
z = 0.05; % probability of failure
t = ceil(log(z) / log(1-w)); % number of iterations
iteration = 0;
max_votes = 0;
best_compatibility = compatibility;

while iteration < t
    rnd_index = randi(length(j_ic)); % select a random feature with 1 or more compatibilities
    j_rnd = j_ic(rnd_index);
    obsv_compatible = find(compatibility.ic(:,j_rnd) == 1);

    for i = 1:length(obsv_compatible)
        H_hyp = zeros(1, observations.m);
        H_hyp(obsv_compatible(i)) = j_rnd;
        map_hyp = EKF_update (map, observations, H_hyp);

        prediction_hyp = predict_observations (map_hyp);
        compatibility_hyp = compute_compatibility (prediction_hyp, observations);

        columns_with_only_ceros = find(all(compatibility_hyp.ic == 0));
        votes = size(compatibility_hyp.ic,2) - length(columns_with_only_ceros) - 1;
        if max_votes < votes
            max_votes = votes;
            prediction = prediction_hyp;
            best_compatibility = compatibility_hyp;
        end
    end
    
    iteration = iteration + 1;
end
H = NN(prediction, observations, best_compatibility);

end
