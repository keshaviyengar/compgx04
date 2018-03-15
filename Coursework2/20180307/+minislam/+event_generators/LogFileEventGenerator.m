% This class implements the event generator for blender-based log files.
% The events are pulled from a log file.

classdef LogFileEventGenerator < event_generators.EventGenerator
   
    properties(Access = protected)
        
        % Debug flag to plot data as it's imported
        plotFramesOnImport;        
        
        % The loaded data
        cachedData;
        
        % The timestep between frames
        dT;
        
        % The current frame number
        frameNumber;
    end
        
    methods(Access = public)
        
        function this = LogFileEventGenerator(directory, plotFramesOnImport)
                        
            this = this@event_generators.EventGenerator();

            if (nargin < 2)
                this.plotFramesOnImport = false;
            else
                this.plotFramesOnImport = plotFramesOnImport;
            end
            
            
            % Check the directory exists
            assert(exist(directory, 'dir') == 7, ...
                'logfileeventgenerator:cannotfinddirectory', ...
                'Cannot find directory %s', directory);
            
            % Load the log files        
            this.loadLogFiles(directory);
            
            % Assume 10 fps
            this.DT = 0.1;
            
            % Set the frame number at the start
            this.frameNumber = 0;
            
        end
    
    
        % This method returns true as long as we should keep running
        function carryOn = keepRunning(this)
            carryOn = false;
        end

        % Get the next event from the generator
        function nextEvent = step(this)
            
            % Code to work out the relative transformation between on
            % vertex and the next!
            
            nextEvent = [];
        end
    
    end
    
    methods(Access = protected)
        
        function loadLogFiles(this, logDir)
            
            % First clean up the directory if needed
            logDir = fullfile(logDir);
            
            % First see if we have the cache file. If so, load it
            cacheFile = fullfile(logDir, 'cache.mat');
            
            if (exist(cacheFile, 'file') == 2)
                load(cacheFile);
                this.cachedData = cachedData;
                return
            end
            
            disp('Loading log files and building cache');
            
            % Load the files
            disp('Loading ground truth 3D landmarks')
            cachedData.landMarks = load(fullfile(logDir, 'landmarks_3d.txt'));
            
            disp('Loading camera parameters')
            cachedData.cameraParams = load(fullfile(logDir, 'calibration.txt'));
            
            disp('Loading camera poses');
            cachedData.cameraPoses = load(fullfile(logDir, 'camera_poses.txt'));
            
            cachedData.numberOfCameraPoses = size(cachedData.cameraPoses, 1);

            disp('Loading tracks');
            fileID = fopen(fullfile(logDir, 'tracks.txt'));
            frames = {};

            if (this.plotFramesOnImport == true)
                figure(1)
                hold off
                H = plot(NaN, NaN, 'b.');
                hold on
                axis([0 1024 0 768]);
            end
            
            tline = fgetl(fileID);
            frameNumber = 1;
            while (tline~= -1)
                fields = textscan(tline, '%f');
                frames{frameNumber}.id = fields{1}(1:3:end);
                frames{frameNumber}.pixels = [fields{1}(2:3:end)';fields{1}(3:3:end)'];
                if (this.plotFramesOnImport == true)
                    set(H, 'XData', frames{frameNumber}.pixels(1,:), 'YData', frames{frameNumber}.pixels(2,:));
                    drawnow
                end
                tline = fgetl(fileID);
                frameNumber = frameNumber + 1;
            end
            cachedData.frames = frames;
            
            this.cachedData = cachedData;
            
            save(cacheFile, 'cachedData');
        end
    end
        
end
