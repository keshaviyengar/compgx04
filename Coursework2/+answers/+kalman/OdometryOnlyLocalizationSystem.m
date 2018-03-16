classdef OdometryOnlyLocalizationSystem < minislam.localization.KalmanFilterLocalizationSystem
    
    % This class extends the localization system to properly handle vehicle
    % odometry
    
    methods(Access = public)
        
        % Call the base class constructor
        function this = OdometryOnlyLocalizationSystem()
            this = this@minislam.localization.KalmanFilterLocalizationSystem();
        end
        
    end
       
    methods(Access = protected)
    
        % This method should return the relative transformation and the
        % covariance in this relative transformation given a time step length
        % dT. The solution below simply predicts forwards at the speed and
        % ignores steer information. The covariance is also arbitrary.
        
        function [xPred, Fd, Qd] = predictMeanJacobianNoise(this, dT)
            
            % The control inputs are in this.u
            vDT = dT * this.u(1);
            
            % The previous mean in the Kalmnan filter is this.xEst
            xPred = this.xEst;
            xPred(1) = xPred(1) + vDT * cos(xPred(3) + 0.5*this.u(2));
            xPred(2) = xPred(2) + vDT * sin(xPred(3) + 0.5*this.u(2));
            xPred(3) = xPred(3) + this.u(2);

            % The covariance prediction equation is
            Fd = [1, 0, -vDT*sin(this.xEst(3) + this.u(2)/2);
                  0, 1,  vDT*cos(this.xEst(3) + this.u(2)/2);
                  0, 0,  1];
            Qd = diag([0.2 0.1 0.1])*dT^2;
            this.PPred = Fd * this.PEst * Fd' + Qd;

        end
    end
end