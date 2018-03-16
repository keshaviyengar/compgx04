% This script runs the odometry

% Configure to disable other sensor types
parameters = minislam.event_generators.simulation.Parameters();
parameters.enableGPS = false;
parameters.enableLaser = false;

% Set up the simulator and the output
simulator = minislam.event_generators.simulation.Simulator(parameters);

kalmanFilterOdometrySystem = answers.kalman.OdometryOnlyLocalizationSystem();
kalmanResults = minislam.mainLoop(simulator, kalmanFilterOdometrySystem);

% Run the two simulations
gtsamOdometrySystem = answers.gtsam.OdometryOnlyLocalizationSystem();
gtsamResults = minislam.mainLoop(simulator, gtsamOdometrySystem);

% Plot optimisation times
minislam.graphics.FigureManager.getFigure('Optimization times');
clf
plot(kalmanResults.optimizationTimes)
hold on
plot(gtsamResults.optimizationTimes)
