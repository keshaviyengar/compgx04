%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% GTSAM Copyright 2010, Georgia Tech Research Corporation, 
% Atlanta, Georgia 30332-0415
% All Rights Reserved
% Authors: Frank Dellaert, et al. (see THANKS for the full author list)
% 
% See LICENSE for the license information
%
% @brief Example of a simple 2D localization example
% @author Frank Dellaert
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Test code from Workshop 4 to create a chain of 10 poses and link together
% and compare a good initial estimate vs. a poor initial estimate.

import gtsam.*
    
%% Assumptions
%  - Robot poses are facing along the X axis (horizontal, to the right in 2D)
%  - The robot moves 2 meters each step
%  - The robot is on a grid, moving 2 meters each step

%% Create the graph (defined in pose2SLAM.h, derived from NonlinearFactorGraph)
graph = NonlinearFactorGraph;

%% Add a Gaussian prior on pose x_1
priorMean = Pose2(0.0, 0.0, 0.0); % prior mean is at origin
priorNoise = noiseModel.Diagonal.Sigmas([0.3; 0.3; 0.1]); % 30cm std on x,y, 0.1 rad on theta
graph.add(PriorFactorPose2(1, priorMean, priorNoise)); % add directly to graph

%% Add two odometry factors
odometry = Pose2(2.0, 0.0, 0.0); % create a measurement for both factors (the same in this case)
odometryNoise = noiseModel.Diagonal.Sigmas([0.2; 0.2; 0.1]); % 20cm std on x,y, 0.1 rad on theta
num_poses = 10;
step = 2;
for pose=1:num_poses-1
    graph.add(BetweenFactorPose2(pose, pose+1, odometry, odometryNoise));
end

%% print
graph.print(sprintf('\nFactor graph:\n'));

%% Initialize to noisy points
initialEstimate = Values;
%good initial estimate
for pose=1:num_poses
    initialEstimate.insert(pose, Pose2(step*(pose-1) + normrnd(0,0.3), normrnd(0,0.3), normrnd(0,0.1)));
end
%inital estimate at origin
% for pose=1:num_poses
%     initialEstimate.insert(pose, Pose2(0, 0, 0));
% end
initialEstimate.print(sprintf('\nInitial estimate:\n  '));

%% Optimize using Levenberg-Marquardt optimization with an ordering from colamd
optimizer = LevenbergMarquardtOptimizer(graph, initialEstimate);
result = optimizer.optimizeSafely();
result.print(sprintf('\nFinal result:\n  '));

%% Plot trajectory and covariance ellipses
cla;
hold on;

plot2DTrajectory(result, [], Marginals(graph, result));

axis equal
view(2)
