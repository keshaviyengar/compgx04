classdef GPSLocalizationSystem < answers.gtsam.OdometryOnlyLocalizationSystem
   
    % This class extends the odometry system to process GPS measurements
    % when they are available.
    
    methods(Access = public)
        
        function this = GPSLocalizationSystem()
            this = this@answers.gtsam.OdometryOnlyLocalizationSystem();
        end
        
    end
    
    methods(Access = protected)        
        
        % Implement the GPS measurement.
        function handleGPSEvent(this, event)
            
            % You will need to create a suitable factor and noise model
            % insert it into the graph. You will need:
            
            % Specify the measurement noise model, e.g.,
            gpsCovariance = zeros(3);
            gpsNoise = gtsam.noiseModel.Gaussian.Covariance(gpsCovariance);
            
            % Construct the suitable factor
            %gpsObservationFactor = ...
            
            % Insert it into the graph. Use this method:
            %this.add(gpsObservationFactor);
        end
    end    
end