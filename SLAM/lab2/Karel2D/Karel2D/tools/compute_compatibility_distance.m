%-------------------------------------------------------
function compatibility = compute_compatibility_distance (prediction, observations)
%-------------------------------------------------------
global configuration chi2;

% Compute individual distances
compatibility.d2 = zeros(observations.m, prediction.n);

for i = 1:observations.m
     [ix, iy, indi] = obs_rows(i);
     z = observations.z(indi);
     for j = 1:prediction.n
         [jx, jy, indj] = obs_rows(j);
         e = z - prediction.h(indj);
         compatibility.d2(i,j) = sqrt(e(1)*e(1)+e(2)*e(2));
     end
end

%dof = 2*observations.m;
dof = 2;

%compatibility.ic = compatibility.d2 < chi2(dof);
compatibility.ic = compatibility.d2 < 0.5;
compatibility.candidates.features = find(sum(compatibility.ic, 1));
compatibility.candidates.observations = find(sum(compatibility.ic, 2))';

compatibility.AL = (sum (compatibility.ic, 2))';
compatibility.HS = prod(compatibility.AL + 1);

