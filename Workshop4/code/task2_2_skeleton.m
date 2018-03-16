clear variables
close all

addpath(genpath('/opt/gtsam/compgx04'));
import gtsam.*

% Supress figures while debugging
set(0,'DefaultFigureVisible','on');

% Basic simulation parameters
steps = 100;

% Parameters

% Covariance in measuring the vehicle control inputs
RU = diag([0.1 0.5*pi/180]).^2;

% Covariance of measuring position and orientation
RZ = diag([0.1; 0.1; 10]);

% Initial covariance
P0 = 1e-6 * diag([0.1 0.1 0.1*pi/180]).^2;

% Set up figure to visualise the scenario
figure(1)
clf
hold on
axis equal

% Set up the simulation and the time history of the pose. Note we log the
% initial condition, which is why we store step+1 entries.
xTrue = zeros(3,1);
xTrueStore = NaN(3, steps + 1);
uTrue = zeros(2, 1);

xTrueStore(:, 1) = xTrue;

% The measured control input, which is corrupted by noise
u = zeros(2, 1);

% Set up the initial filter state
xEst = [0;0;0];
PEst = P0;

% Set up arrays to store the true history and how the robot is evolving
xKFEstStore = NaN(3, steps + 1);
PKFEstStore = NaN(3, steps + 1);
xKFEstStore(:, 1) = xEst;
PKFEstStore(:, 1) = diag(PEst);

% Setup GTSAM
graph = NonlinearFactorGraph();
initialEstimate = Values();

% Set up the initial GTSAM state

% GTSAM has a slightly weird way of setting up the pose estimate.
% Rather than take a vector input, you need to create a point for
% translation (even though translation isn't a point) and a rotation
% and build the results up.
priorMean = Pose2(xTrue(3), Point2(xTrue(1:2)));
measurementNoise = noiseModel.Gaussian.Covariance(P0);

xGTSAMStore = NaN(3, steps + 1);
PGTSAMStore = NaN(3, steps + 1);

% Create the key corresponding to the initial state and insert an initial
% value (which is a prior). Note the class type for the key. This is very
% important.
poseKey=uint8(0);
initialEstimate.insert(poseKey, priorMean);

% Set up the initial conditions. This prior pose factor tells GTSAM that we
% know the initial conditions with a prescribed level of accuracy.
graph.add(PriorFactorPose2(poseKey, priorMean, measurementNoise));

% Iterate through the experimental run.

for k = 1:steps
    
    % Get the control inputs; the change in heading oscillates at 0.1Hz and
    % looks a bit like when Simon tries to drive.
    uTrue(1) = 1;
    uTrue(2) = 0.1 * pi * sin(2 * k * pi / 90);

    % Predict how the true state evolves
    xTrue(1) = xTrue(1) + uTrue(1) * cos(xTrue(3) + 0.5 * uTrue(2));
    xTrue(2) = xTrue(2) + uTrue(1) * sin(xTrue(3) + 0.5 * uTrue(2));
    xTrue(3) = xTrue(3) + uTrue(2);
    
    % Store the current true value
    xTrueStore(:, k) = xTrue;
    
    % Get the measured control inputs. Note that we model the process noise
    % as a measurement error on the control input.
    u = uTrue + sqrtm(RU) * randn(2, 1);
    
    % Get the measurement. We assume that it's only available for a few
    % time steps. The measurement itself is corrupted by noise.s
    measurementReceived = (k < 10) || (k > 80);

    z = xTrue + sqrtm(RZ) * randn(3, 1);
    
    %% Kalman filter code
    
    %% TODO
    % Do the Kalman filter prediction using an EKF. You will need to
    % (a) predict the mean
    % (b) predict the covariance
    %
    % For the covariance, you will also have to treat observation noise as
    % an error which is added to the covariance matrix.
    xPred(1,1) = xEst(1) + u(1) * cos(xEst(3) + 0.5 * u(2));
    xPred(2,1) = xEst(2) + u(1) * sin(xEst(3) + 0.5 * u(2));
    xPred(3,1) = xEst(3) + u(2);
    
    jac_f = [1, 0, -u(1)*sin(xEst(3) + u(2)/2);
             0, 1,  u(2)*cos(xEst(3) + u(2)/2);
             0, 0,  1];
       
    jac_h = [1 0 0;0 1 0; 0 0 1];
    
    PPred = jac_f*PEst*jac_f' + RZ;
    
    % Update if we have a measurement
    if (measurementReceived == true)
        %% TODO: Implement the Kalman filter update
        K_k = PPred*jac_h' / (jac_h*PPred*jac_h + RZ);
        xEst = xPred + K_k*(z - xPred);
        PEst = (eye(size(PEst)) - K_k*jac_h)*PPred;
    else
        xEst = xPred;
        PEst = PPred;
    end
    
    % Store the estimate and covariance
    xKFEstStore(:, k + 1) = xEst;
    PKFEstStore(:, k + 1) = diag(PEst);

    % Draw the output as we go
    covarianceEllipse(xEst(1:2), PEst(1:2, 1:2), 'g');
    
    %% GTSAM
        
    % Following on from the OdometryExample, we need to create the relative
    % transformation. We define this as follows. Suppose P1 is the
    % Pose2 at time k and P2 is the Pose2 at time k+1 and P12 is the
    % relative pose. Therefore,
    % P2 = P1.compose(P12)
    
    % As noted above, GTSAM has a slightly weird way of setting up the pose
    % estimate. Rather than take a vector input, you need to create a point
    % for translation (even though translation isn't a point) and a
    % rotation and build the results up.
    
    %% TODO:
    % Compute the relative translation and rotation
    relativeTranslation = Point2([u(1) * cos(xEst(3) + 0.5 * u(2))...
                                  u(1) * sin(xEst(3) + 0.5 * u(2))]');   
    relativeRotation = u(2);
    
    % Assemble the relative transformation.
    relativePose = Pose2(relativeRotation, relativeTranslation);
    
    %% TODO:
    % Compute the covariance on this transformation. Note that the control
    % input enters into a nonlinear way, so you'll have to linearise again
    % to figure out how the noise enters.
    relativePoseCovariance = eye(3)*0.01;
    
    % This is needed to provide numerical stability for GTSAM. Do not remove!    
    relativePoseCovariance = relativePoseCovariance + 1e-6 * eye(3);
    
    % Create the key for the pose at the current time
    newPoseKey = poseKey + uint8(1);
    
    % Insert the initial estimate for this new pose key using the relative
    % transformation we computed above.
    priorMean = priorMean.compose(relativePose);
    initialEstimate.insert(newPoseKey, priorMean);    
    
    % Insert the link between our newly created pose key and the previous
    % one
    graph.add(BetweenFactorPose2(poseKey, newPoseKey, relativePose, noiseModel.Gaussian.Covariance(relativePoseCovariance)));

    % poseKey now points to the current time.
    poseKey = newPoseKey;    

    %% TODO:
    % If a measurement is received, add a measurement to the graph. Check
    % the LocalizationExample to see how to do this. Remember that you will
    % have to use the trick above to create the pose from the observation.
    % You will also need to set the covariance on this to RZ.        
    if (measurementReceived == true)
        measurementNoise = noiseModel.Diagonal.Sigmas([RZ(1,1); RZ(2,2); RZ(3,3)]);
        graph.add(PriorFactorPose2(poseKey, Pose2(z(1),z(2),z(3)), measurementNoise));
    end
end

% At this point, the Kalman filter has done its business. However, GTSAM
% has not done its optimisation yet.

% Uncomment these if you want to print the structure of the graph out.
%graph.print('\nFactor graph:\n');
%initialEstimate.print('\nInitial estimate:\n  ');

% Create the optimiser. This takes the initial conditions and the graph
% structure.
optimizer = LevenbergMarquardtOptimizer(graph, initialEstimate);

% To optimise directly, run:
% result = optimizer.optimizeSafely();

% This is a more verbose way to run the optimisation on each step. Note
% that the graphics drawing is super slow, not the optimiser.
maxIterations = 100;
minImprovement = 1e-5;
errors = [];
figure(5)
for k=1:maxIterations
    % Iterate
    optimizer.iterate();
    result = optimizer.values();
    errors = [errors optimizer.error()];
    fprintf('Iteration %i error: %f\n', k, errors(end));
    
    % Plot
    subplot(2,1,1);
    bar(errors,'r'); % error bars
    subplot(2,1,2);
    cla;
    hold on;
    axis equal;
    %plot2DTrajectory(result, [], Marginals(graph, result)); % trajectory
    drawnow;
    hold off;
    
    % Stop condition
    if((length(errors) >= 2) && (abs(errors(end) - errors(length(errors)-1)) < minImprovement))
        break;
    end
end

% Uncomment if you want to see the final values   
%result.print(sprintf('\nFinal result:\n  '));

% For GTSAM, we now extract the estimate and covariance matrices. This is a
% bit more cumbersome because we have to "read it out" of the results.
marginals = Marginals(graph, result);

keys = KeyVector(result.keys);
for k = 0:keys.size-1
    key = keys.at(k);
    pose = result.atPose2(key);
    P = marginals.marginalCovariance(key);
    xGTSAMStore(1, k+1) = pose.x;
    xGTSAMStore(2, k+1) = pose.y;
    xGTSAMStore(3, k+1) = pose.theta;
    PGTSAMStore(:, k+1) = diag(P);
end

% Now plot the results. For each state we show the error and the 1 sigma
% standard deviation line.
for s = 1 : 3
    figure(s + 1)
    clf
    dXEKF = xKFEstStore(s, :)-xTrueStore(s,:);
    dGTSAM = xGTSAMStore(s, :)-xTrueStore(s,:);
    if (s == 3)
        dXEKF = pi_to_pi(dXEKF);
        dGTSAM = pi_to_pi(dGTSAM);
    end
    plot(dXEKF,'g');
    hold on
    plot(sqrt(PKFEstStore(s,:)), 'r', 'LineWidth', 2);
    plot(dGTSAM,'m')
    plot(sqrt(PGTSAMStore(s,:)), 'k', 'LineWidth', 2);
    legend({'EKF Error', 'EKF SSD', 'GTSAM Error', 'GTSAM SSD'});
end
figure(5)

% Plot the trajectory. This is slow.
cla;
hold on;
figure(1)
plot2DTrajectory(result, [], marginals);
plot(xTrueStore(1,:), xTrueStore(2,:), 'r+')

