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
            z = event.data;
            RZ = event.covariance;
            % You will need to write a Kalman filter update
            % The variables this.xPred, this.PPred contain the predicted
            % mean and covariance.
            PPred = this.PPred(1:2,1:2);
            xPred = this.xPred(1:2);
            
            
            % Your update should produce a new estimate this.xEst and
            % this.PEst
            K_k = PPred*eye(2)' / (eye(2)*PPred*eye(2) + RZ);
            xEst = xPred + K_k*(z - xPred);
            PEst = (eye(2) - K_k*eye(2))*PPred;

            this.xEst(1:2) = xEst;
            this.PEst(1:2,1:2) = PEst;
        end
    end    
end