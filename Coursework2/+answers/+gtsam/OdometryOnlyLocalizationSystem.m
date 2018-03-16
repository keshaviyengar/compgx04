classdef OdometryOnlyLocalizationSystem < minislam.localization.ISAMLocalizationSystem
   
    % This class extends the localization system to properly handle vehicle
    % odometry
    
    methods(Access = public)
        
        % Call the base class constructor
        function this = OdometryOnlyLocalizationSystem()
            this = this@minislam.localization.ISAMLocalizationSystem();
        end
        
    end
    
    methods(Access = protected)
        
        % This method should return the relative transformation and the
        % covariance in this relative transformation given a time step length
        % dT. The solution below simply predicts forwards at the speed and
        % ignores steer information. The covariance is also arbitrary.
        function [relativePose, relativePoseCovariance] = computeRelativeVehicleTransformAndCovariance(this, dT)
            
            % The vector this.u contains the most recent control inputs in
            % the form [speed, turn angle rate].
            
            vDT = dT * this.u(1);
            phi = this.currentVehiclePose.theta;
            relativePose = gtsam.Pose2([vDT*cos(phi + 0.5*dT*this.u(2));
                                        vDT*sin(phi + 0.5*dT*this.u(2));
                                        dT*this.u(2)]);           
            relativePoseCovariance = diag([0.2 0.1 0.1])*dT^2;

        end
    end
end