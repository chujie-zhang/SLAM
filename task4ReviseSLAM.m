% This script can be used to compare your SLAM system

% Configure to disable other sensor types
parameters = minislam.event_generators.simulation.Parameters();

% Magic tuning for the no-prediction case
parameters.laserDetectionRange = 20;

% By setting true / false you can enable different combinations of sensors
parameters.enableGPS = false;
parameters.enableLaser = true;

% Set up the simulator and the output
simulator = minislam.event_generators.simulation.Simulator(parameters, 'task3');

% Create and run the different localization systems
g2oSLAMSystem = minislam.slam.g2o.G2OSLAMSystem();
results = minislam.mainLoop(simulator, g2oSLAMSystem);


% Plot the error curves
minislam.graphics.FigureManager.getFigure('Errors');
clf
plot(results{1}.vehicleStateHistory'-results{1}.vehicleStateHistory')

% Plot covariance
minislam.graphics.FigureManager.getFigure('Vehicle Covariances');
clf
plot(results{1}.vehicleCovarianceHistory')

% Plot errors
minislam.graphics.FigureManager.getFigure('Errors');
clf
plot(results{1}.vehicleStateHistory'-results{1}.vehicleTrueStateHistory')

% Here's how to print out the number of edges and vertices
g2oGraph = g2oSLAMSystem.optimizer();

numVertices = length(g2oGraph.vertices());
numEdges = length(g2oGraph.edges());

%Number of landmarks initialized
numLandmarks = results{1}.numOfLandmarks;
%stepNumber
time = results{1}.time;
%Number of vehicle poses sotred
numVehicle = numVertices - numLandmarks;
edgeObservation = numEdges - (numVehicle - 1);
disp("The number of landmarks initialized");
disp(numLandmarks);
disp("The number of vehicle poses sotred");
disp(numVehicle);
disp("The average number of observations made by a robot at each timestep");
disp(edgeObservation/time)
disp("The average number of observations received by a landmark");
disp(edgeObservation/numLandmarks);
disp("Running time");
disp(time);
disp("number of vertices");
disp(numVertices);
disp("number of edges");
disp(numEdges);
