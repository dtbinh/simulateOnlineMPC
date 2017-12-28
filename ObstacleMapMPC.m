classdef ObstacleMapMPC < handle
  % Obstacle map class for MPC
  
  properties
    % 2D Obtacles
    global_obs
    local_obs
    padded_obs
    vOb
    
    % handles for plots
    hG
    hL
    hP
    hS
  end
  
  methods
    %% Constructor.
    function self = ObstacleMapMPC(vOb,resolution)
      self.global_obs = initObs(vOb,resolution);
      nObs = length(self.global_obs);    % Number of individual obstacles
      self.local_obs = cell(1,nObs);
      self.padded_obs = cell(1,nObs);
      self.vOb = [];      
    end
    
    %% SenseAndUpdate
    % sense for obstacles that are within sense_range of the point
    % then update local and padded obstacles.
    function sense_update(self, x0, sense_radius, track_err)
        self.local_obs = updateLocal(x0, self.global_obs, self.local_obs, sense_radius);
        self.padded_obs = updatePadded(self.local_obs, track_err);
        self.vOb = padded2vOb(self.padded_obs);
    end
    
    %% plotGlobal
    function plotGlobal(self, color, linestyle)
      if nargin < 2
        color = 'k';
      end
      
      if nargin < 3
        linestyle = ':';
      end
      
      extraArgs.LineStyle = linestyle;
      self.hG = plotGlobal(self.global_obs, color, linestyle, extraArgs);
    end
    
    %% plotLocal
    function plotLocal(self, color, linestyle)
      if nargin < 2
        color = 'r';
      end
      
      if nargin < 3
        linestyle = '-';
      end
      
      delete(self.hL)
      
      extraArgs.LineStyle = linestyle;
      self.hL = plotLocal(self.local_obs, color, linestyle, extraArgs);
    end
    
    %% plotPadded
    function plotPadded(self, color, linestyle)
      if nargin < 2
        color = 'b';
      end
      
      if nargin < 3
        linestyle = '--';
      end
      
      delete(self.hP)
      
      extraArgs.LineStyle = linestyle;
      self.hP = plotPadded(self.padded_obs, color, linestyle, extraArgs);
    end
    
    %% plotSenseRegion
    function plotSenseRegion(self, x0, senseRadius, varargin)
      if nargin < 3
        senseRadius = 2.0;
      end  
        
      if nargin < 4
        varargin = 'g--';
      end
      
      delete(self.hS)
      
      self.hS = plotDisk(x0, senseRadius, varargin);
    end
  % END OF METHODS
  end
end