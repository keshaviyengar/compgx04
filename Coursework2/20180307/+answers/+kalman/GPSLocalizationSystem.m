classdef GPSLocalizationSystem < answers.kalman.OdometryOnlyLocalizationSystem
   
    % This class extends the odometry system to process GPS measurements
    % when they are available.
    
    methods(Access = public)
        
        function this = GPSLocalizationSystem()
            this = this@answers.kalman.OdometryOnlyLocalizationSystem();
        end
        
    end
    
    methods(Access = protected)        
        
        % Implement the GPS measurement.
        function handleGPSEvent(this, event)
            
            % You will need to write a Kalman filter update
            
            % The variables this.xPred, this.PPred contain the predicted
            % mean and covariance.
            
            % Your update should produce a new estimate this.xEst and
            % this.PEst
        end
    end    
end