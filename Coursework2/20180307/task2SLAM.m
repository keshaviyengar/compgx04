% This script runs the odometry

% Configure to disable unneed sensors
parameters = minislam.event_generators.simulation.Parameters();
parameters.enableGPS = false;

% Set up the simulator and the output
simulator = minislam.event_generators.simulation.Simulator(parameters);

% Run the localization system
gtsamOdometrySystem = answers.gtsam.OdometryOnlyLocalizationSystem();
gtsamResults = minislam.mainLoop(simulator, gtsamOdometrySystem);

% Plot optimisation times
minislam.graphics.FigureManager.getFigure('Optimization times');
clf
plot(gtsamResults.optimizationTimes)
