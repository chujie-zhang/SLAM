% This class implements an event-based estimation system using g2o and
% the barebones for building up a minimal, ideal SLAM system. The system is
% event-based and responds to a sequence of events which are time stamped
% and served in order. To implement your SLAM system, you will need to
% implement various methods which mostly involve working with the graph.
% These methods are initially stubbed out and will generate exceptions if
% you try to call them.

classdef G2OSLAMSystem < minislam.slam.VehicleSLAMSystem
    
    properties(Access = protected)
                
        % Flag to run the detailed graph validation checks
        validateGraphOnInitialization;        
        
        % The graph used for performing estimation.
        graph;
        
        % The optimization algorithm
        optimizationAlgorithm;
        
        % The most recently created vehicle vertex.
        currentVehicleVertex;
        
        % The set of all vertices associated with the vehicle state over
        % time.
        vehicleVertices;
        vehicleVertexId;
        
        vehicleEdges;
        % The landmark vertices. Confusingly enough, "Map" here refers to
        % the data structure which is used to store the landmarks. (It
        % allows random access of landmarkID to landmark object.)
        landmarksMap;
        counts
        numOfLandmarks=0;
        time=1;


    end
        
    methods(Access = public)
        
        % Create the localization system and start it up.
        function this = G2OSLAMSystem()
            
            % Call the base class constructor
            this = this@minislam.slam.VehicleSLAMSystem();
            
            % Create the graph and the optimization algorithm
            this.graph = g2o.core.SparseOptimizer();
            algorithm = g2o.core.LevenbergMarquardtOptimizationAlgorithm();
            this.graph.setAlgorithm(algorithm);
            
            % Do detailed checking by default
            this.validateGraphOnInitialization = true;
            
            % Preallocate; this is a lower bound on size
            this.vehicleVertices = cell(1, 10000);
            
            % No vehicle vertices initally set
            this.vehicleVertexId = 0;
            
            % Allocate the landmark map
            this.landmarksMap = containers.Map('KeyType', 'int64', 'ValueType', 'any');
            this.counts = containers.Map('KeyType', 'int64', 'ValueType', 'any');
        end
        
        % Get the underlying g2o graph
        function graph = optimizer(this)
            graph = this.graph;
        end
        
        % Set the flag to enable / disable validation. This spots errors in
        % graph construction, but repeated calls can slow things down by
        % 20% or more.
        function setValidateGraph(this, validateGraphOnInitialization)
            this.validateGraphOnInitialization = validateGraphOnInitialization;
        end
        
        function validateGraphOnInitialization = validateGraph(this)
            validateGraphOnInitialization = this.validateGraphOnInitialization;
        end
        
        % Recommend if an optimization is a good idea. Based on an event,
        % some activities (e.g., such as loop closing) can have a very big
        % impact on the estimates. Therefore, this method returns if the
        % localization algorithm thinks optimizing is a good idea. Here we
        % always return false because this gives the fastest results
        % because you just optimize once, right and the very end
        function recommendation = recommendOptimization(this)
            recommendation = false;
            
            % This is how to do it after every 500 steps
            %recommendation = rem(this.stepNumber, 100) == 0;
        end
        
        % This method runs the graph optimiser and the outputs of the
        % optimisation are used as the initial conditions for the next
        % round of the graph. This makes it possible to iterative compute
        % solutions as more information becomes available over time.
        function optimize(this, maximumNumberOfOptimizationSteps)
			%-------------------------------------------------------------%
            %-----------------task 4.2------------------------------------%
			%-------------------------------------------------------------%
            numEdges = size(this.vehicleEdges)
            %Note: the first case may lead to an error and fail.
            %for i = 1:numEdges(2)   %the first case: delete all vehicle prediction edges
            for i = 2:numEdges(2)    % the second case: all but the rst vehicle prediction edges
                %this.graph.removeEdge(this.vehicleEdges{i});
            end
            
            this.graph.initializeOptimization(this.validateGraphOnInitialization);
            if (nargin > 1)
                this.graph.optimize(maximumNumberOfOptimizationSteps);
            else
                this.graph.optimize();
            end
        end
                
        % Return the current mean and covariance estimate of the robot.
        % This is only valid after optimization has been called.
        function [x, P] = robotEstimate(this)
            [xS, PS] = this.graph.computeMarginals(this.currentVehicleVertex);
            x=full(xS);
            P=full(PS);
        end
        
        function [T, X, P] = robotEstimateHistory(this)
            
            % Extract the graph
            [xS, PS] = this.graph.computeMarginals();
            
            % Create the output array
            X = zeros(3, this.vehicleVertexId);
            P = zeros(3, this.vehicleVertexId);
            T = zeros(1, this.vehicleVertexId);
            
            % Copy the outputs over
            for v = 1 : this.vehicleVertexId
                idx = this.vehicleVertices{v}.hessianIndex();
                
                T(v) = this.vehicleVertices{v}.time();
                
                % Copy the estimate into the array. If the vertices is
                % fixed (conditioned), its estimate is okay. The covariance
                % is not explicitly defined, but has a value of zero.
                % Therefore we fill this manually.
                if (isempty(idx) == true)
                    X(:, v) = this.vehicleVertices{v}.estimate();
                    P(:, v) = zeros(3, 1);
                else
                    X(:, v) = full(xS(idx));
                    P(:, v) = full(diag(PS(idx, idx)));
                end
            end
        end
        
        % Return the means and covariances of the landmark estimates. These
        % are only valid after optimization has been called.
        function [x, P, landmarkIds] = landmarkEstimates(this) 
            
            landmarkVertices = values(this.landmarksMap);
            
            numberOfLandmarks = length(landmarkVertices);
            
            landmarkIds = NaN(1, numberOfLandmarks);
            x = NaN(2, numberOfLandmarks);
            P = NaN(2, 2, numberOfLandmarks);
            
            [xS, PS] = this.graph.computeMarginals();
            
            for l = 1 : numberOfLandmarks
                landmarkIds(l) = landmarkVertices{l}.landmarkId();
                idx = landmarkVertices{l}.hessianIndex();
                x(:, l) = full(xS(idx));
                if (isempty(idx == true))
                    P(:, :, l) = zeros(3, 3);
                else
                    P(:, :, l) = full(PS(idx, idx));
                end
            end
        end
    end
        
    % These are the methods you will need to overload
    methods(Access = protected)
                        
        % Declare bodies all of these methods. This makes it possible to
        % instantiate this object.
         
        function handleInitialConditionEvent(this, event)
            
            % Add the first vertex and the initial condition edge
            this.currentVehicleVertex = minislam.slam.g2o.VehicleStateVertex(this.currentTime);
            this.currentVehicleVertex.setEstimate(event.data);
            this.currentVehicleVertex.setFixed(true);
            this.graph.addVertex(this.currentVehicleVertex);
            this.vehicleVertexId = 1;
            this.vehicleVertices{this.vehicleVertexId} = this.currentVehicleVertex;
       end
       
        function handleNoPrediction(~)
            % Nothing to do
        end
        
        function handlePredictToTime(this, time, dT)
            this.time = this.time + 1;
            % Create the next vehicle vertex and add it to the graph
            this.currentVehicleVertex = minislam.slam.g2o.VehicleStateVertex(time);
            this.graph.addVertex(this.currentVehicleVertex);
            
            % Handle the prediction
            %error('handlePredictToTime: implement me');
            
            U = this.u;
            U(3) =  g2o.stuff.normalize_theta(U(3));
            
            priorX =  this.vehicleVertices{this.vehicleVertexId}.estimate();
            c=cos(priorX(3));
            s=sin(priorX(3));
            Mi = [c,-s,0;
                  s,c,0;
                  0 0 1];
            target = priorX + dT * Mi * U;
            target(3) = g2o.stuff.normalize_theta( target(3) );
            this.currentVehicleVertex.setEstimate(target);
            
            omegaQ = pinv(this.uCov);
            edge =  minislam.slam.g2o.VehicleKinematicsEdge(dT);
            edge.setVertex(1, this.vehicleVertices{this.vehicleVertexId});
            edge.setVertex(2, this.currentVehicleVertex ) ;
            edge.setMeasurement(U);
            edge.setInformation(omegaQ);
            this.graph.addEdge(edge);
            
            this.vehicleEdges{this.vehicleVertexId} = edge;
            % Bump the indices
            this.vehicleVertexId = this.vehicleVertexId + 1;
            this.vehicleVertices{this.vehicleVertexId} = this.currentVehicleVertex;
        end
        
        function handleGPSObservationEvent(this, event)
            
            % Handle a GPS measurement
            %error('handleGPSObservationEvent: implement me');
            edge = minislam.slam.g2o.GPSMeasurementEdge();
            edge.setVertex(1,this.currentVehicleVertex);
            edge.setMeasurement(event.data);
            edge.setInformation(pinv(event.covariance) );
            this.graph.addEdge(edge);         
            
        end
        
        function handleLandmarkObservationEvent(this, event)
  
            % Iterate over all the landmark measurements
            for l = 1 : length(event.landmarkIds)

                % Get the landmark vertex associated with this measurement.
                % If necessary, a new landmark vertex is created and added
                % to the graph.
                [landmarkVertex, newVertexCreated] = this.createOrGetLandmark(event.landmarkIds(l));
                z = event.data(:, l);
                range = z(1); 
                rad = z(2); 
    
                
                if (isKey(this.counts, event.landmarkIds(l)) == true)     
                    this.counts(event.landmarkIds(l)) = this.counts(event.landmarkIds(l)) + 1; 
                else
                    this.counts(event.landmarkIds(l)) = 1;
                end
                %{
                disp('ggggggggggg');
                disp(this.counts(event.landmarkIds(l)));
                disp('hhhhhhhhhh');
                disp(this.landmarksMap(event.landmarkIds(l)));
                %}
                %if  this.counts(event.landmarkIds(l)) < 100
                if newVertexCreated
                   X =  this.vehicleVertices{this.vehicleVertexId}.estimate();
                   Xi=X(1)+range*cos(X(3)+rad);
                   Yi=X(2)+range*sin(X(3)+rad);

                   landmarkVertex.setEstimate([Xi;Yi]);
                   this.graph.addVertex(landmarkVertex);        
                end
                
                edge=minislam.slam.g2o.LandmarkEdge();
                edge.setVertex(1,this.vehicleVertices{this.vehicleVertexId});
                edge.setVertex(2,landmarkVertex);
                edge.setMeasurement(z);
                edge.setInformation(pinv(event.covariance));
                this.graph.addEdge(edge);
                %end
            end
                    
        end
        
   end
    
    methods(Access = protected)
        
        % This method returns a landmark associated with landmarkId. If a
        % landmark exists already, it is returned. If it does not exist, a
        % vertex is created and is added to the graph.
        function [landmarkVertex, newVertexCreated] = createOrGetLandmark(this, landmarkId)
            
            % If the landmark exists already, return it
            if (isKey(this.landmarksMap, landmarkId) == true)
                landmarkVertex = this.landmarksMap(landmarkId);
                newVertexCreated = false;
            else
                % If landmark does not exist, a vertex is created and is added to the graph.
                landmarkVertex = minislam.slam.g2o.LandmarkStateVertex(landmarkId);
                this.landmarksMap(landmarkId) = landmarkVertex;
                newVertexCreated = true;
            end
            
        end
                
    end
    
    methods(Access = public)    
        function numOfLandmarks = getNumOfLandmarks(this)
            numbers = size(this.landmarksMap);
            numOfLandmarks = numbers(1);
        end
        
        function time = getTime(this)
            time = this.time;
        end      
    end
    
end
