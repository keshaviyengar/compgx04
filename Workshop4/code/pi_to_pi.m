function theta = pi_to_pi(theta)

% Work out angular error. Note that there is a discontinuity.

idx = find(theta > pi);
theta(idx) = theta(idx) - 2* pi;

idx = find(theta < - pi);
theta(idx) = theta(idx) + 2* pi;

end