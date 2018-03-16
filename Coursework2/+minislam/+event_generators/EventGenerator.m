% This class generates a stream of events. It could, for example, run a
% simulator, or even be a wrapper over a ROS node.

classdef EventGenerator < handle
    
    properties(Access = protected)
        stepNumber;
    end
    
    methods(Access = public)
        
        % Get the step number; this starts at 0 and advances by 1 after
        % each call to step.
        
        function stepNumber = getStepNumber(this)
            stepNumber = this.stepNumber;
        end
        
    end
    
    methods(Access = public, Abstract = true)
        
        % This method returns true as long as we should keep running
        carryOn = keepRunning(this);

        % Step the event generator
        step(this);
        
        % Get any outstanding events
        nextEvents = getEvents(this);
        
        % Get the ground truth; empty if not available. Flag shows if this
        % should be full and contain all the landmarks
        groundTruthState = getGroundTruth(this, getFullStateInformation);
        
    end
end