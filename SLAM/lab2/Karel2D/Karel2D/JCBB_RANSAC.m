%-------------------------------------------------------
function H = JCBB_RANSAC (prediction, observations, compatibility, j)
% 
%-------------------------------------------------------
global Best;
global configuration;

Best.H = zeros(1, observations.m);

JCBB_R (prediction, observations, compatibility, [], 1, j);

H = Best.H;
configuration.name = 'JCBB';

%-------------------------------------------------------
function JCBB_R (prediction, observations, compatibility, H, i, j)
% 
%-------------------------------------------------------
global Best;
global configuration;

if i > observations.m % leaf node?
    if pairings(H) > pairings(Best.H) % did better?
        Best.H = H;
    end
else
    % complete JCBB here
    if compatibility.ic(i, j) && jointly_compatible(prediction, observations, [H j])
       JCBB_R (prediction, observations, compatibility, [H j], i + 1, j);
    end
    if pairings(H) + observations.m - i > pairings(Best.H) % did better?
        JCBB_R (prediction, observations, compatibility, [H 0], i + 1, j);
    end
end

%-------------------------------------------------------
% 
%-------------------------------------------------------
function p = pairings(H)

p = length(find(H));