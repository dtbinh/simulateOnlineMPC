%% MpcPlanner
classdef MpcPlanner < handle
  % *Description:* MAIN FUNCTION: This function is a basic implementation of
  % the model predictive control (MPC) path planning algorithm with
  % optimization-based collision avoidance approach.
  %
  % *Authors:* Haimin Hu
  %
  % *Last Updated:* 22nd December 2017
  %
  % *Features:*
  %   - 2D search space
  %   - Obstacle avoidance with polyhedron formulation
  %   - Path smoothing
  %
  % *To do:*
  %   - N/A
  
  %% Usage
  % mpc = MpcPlanner()
  % mpc.SetObs(lOb,nOb)
  %     lOb; %Obstacle in lOb representation (check getlOb.m)
  %     nOb = [3 4 5 ... 4]; %Number of single obstacles and their vertices
  % mpc.SetStart([0.0 0.0]);
  % mpc.SetGoal([9.0 9.0]);
  % plot(mpc.smoothedPath(:,1),mpc.smoothedPath(:,2),'k*');
  % mpc.Run()
  % delete(mpc);
  
  properties
    %% RUN OPTIONS
    %  ************************************************************************
    % Display                           => Activate visual display, SLIGHTLY SLOWER
    doDraw=false;
    
    % Skip this number of drawings before doing an update to the figure (speeds up plotting)
    drawingSkipsPerDrawing = 10;
    
    % Obstacle Avoidance                => Include obstacles at all
    avoidObstacles=true;
    
    % Path Smoothing                    => Shorten final path
    smooth_path=true;
    
    %% Extra properties
    % DONE:

    % binary variable: whether to plot traced path or not
    plotTrace;
    
    % binary variable: whether to plot smoothed path or not
    plotSmooth;
    
    %% VARIABLES
    %  ************************************************************************
    % The main data structure
    mpc;
    
    % Default Start and Goal  => Point [x y]
    start=[1.0 1.0];
    goal =[9.0 9.0];        
    
    % Predoction horizon for iterative warmstart and final optimization
    N = 5;
    N_opt = 10;
    
    % Nominal sampling interval
    Ts = 0.5;
    
    % Plant is modelled as a square with length PlantL
    PlantL = 0.05;
    
    % MPC outputs of state and control sequences
    stateP2P = [];
    controlP2P = [];
    stateIteWS = {};
    controlIteWS = {};
    stateOpt = [];
    controlOpt= [];
    
    % Obstacle in lOb representation (check getlOb.m)
    lOb;
    
    % Number of single obstacles and their vertices
    nOb = [4 4 4];
    
    % Whether to perform a final optimization
    isFinalOpt = false;
    
    % Scaling range for iterative warmstart
    ScaleRange = 0.8:0.2:1.0;
    
    % Figure handle
    figure_h = [];
    
    % The number of drawing that have been skipped since the last draw command
    skippedDrawings = 0;    
    
    % The handles for the lines and points in the figure
    plotHandles = [];
    
    % The path from start to goal
    path = [];
    
    % Smoothed path
    smoothedPath = [];
    
    % Number of attempts at path smoothing
    nsmooth=1000;
    
    % Runtime
    runtimeP2P = 0;
    runtimeIteWS = 0;
    runtimeOpt = 0;
    
    % Handles for plots
    hT % Final MPC trajectory
    hE % Ending (target) postion
    hSO % Scaled obstacles
    hST % Trajecotry associated with scaled obstacles
    
  end
  
  properties (SetAccess=protected)
    % Obstacle definitions              => Use objects.txt object definition or default
  end
  
  methods
    %% constructor
    % DONE:
    % *Inputs:*
    % see properties for explanation of inputs
    function self = MpcPlanner(start, goal, N, N_opt, Ts, lOb)
      self.start = start;
      self.goal = goal;
      self.N = N;
      self.N_opt = N_opt;
      self.Ts = Ts;
      self.lOb = lOb;
    end
        
    %% Run
    % Main MPC path planning algorithm
    function Run(self)
      
        % Solve point-to-point OCP for initialization
        [self.stateP2P, self.controlP2P, self.runtimeP2P] = ...
            p2pWS(self.start,self.goal,self.N,self.Ts);

        % Iterative warmstart
        stateWS = self.stateP2P;
        controlWS = self.controlP2P;
        self.runtimeIteWS = 0;
        %   Start Iteration
        for scaleFactor = self.ScaleRange
            %   Scale Obstacles
            lObNew = scaleObs(length(self.nOb),self.nOb,self.lOb,scaleFactor);
            %   obtain H-rep of obstacles
            [AObNew, bObNew] = obsHrep(length(self.nOb), self.nOb, lObNew);
            %   Run MPC
            fprintf('**** Iterative Warm Start at Scaling Factor %f ****\n',scaleFactor)
            [stateOut, controlOut, solverTime, out] = ...
                IteWS(self.start,self.goal,self.N,self.Ts,self.PlantL,stateWS,controlWS,AObNew,bObNew);
            if out.CONVERGENCE_ACHIEVED==1
                fprintf('**** Problem solved SUCCESSFULLY ****\n')
                stateWS = stateOut;
                controlWS = controlOut;
                self.stateIteWS{end+1} = stateOut;
                self.controlIteWS{end+1} = controlWS;
                delete(self.hSO);
                delete(self.hST);
                [self.hSO,self.hST] = plotScaled(stateOut,lObNew, [0.5 0.5 0.5], '--',true);
            else
                delete(self.hSO);
                delete(self.hST);
                fprintf('**** WARNING: Problem could not be solved ****\n')
            end
            self.runtimeIteWS = self.runtimeIteWS + solverTime;
        end
                
        % Final optimization
        flag = false;
        if self.isFinalOpt
            [AOb, bOb] = obsHrep(length(self.nOb), self.nOb, self.lOb);
            [self.stateOpt, self.controlOpt, self.runtimeOpt, out] = ...
                FinalOpt(self.start,self.goal,self.N_opt,self.Ts,...
                self.PlantL,self.stateIteWS{end},self.controlIteWS{end},AOb,bOb);
            flag = out.CONVERGENCE_ACHIEVED;
        end
        
        % Obtain planned path and visualize
        if flag
            fprintf('**** Final OPT solved SUCCESSFULLY ****\n')
%             path = self.stateOpt(:,2:3);
            delete(self.hSO);
            delete(self.hST);
            [~,self.hT] = plotScaled(self.stateOpt, self.lOb, [0.5 0.5 0.5], '-',false);
        else
            if self.isFinalOpt
                fprintf('**** WARNING: Final OPT could not be solved ****\n')
            else
                fprintf('**** NO Final OPT, using warmstart path ****\n')
            end
%             path = self.stateOpt(:,2:3);
            delete(self.hSO);
            delete(self.hST);
            [~,self.hT] = plotScaled(self.stateIteWS{end}, self.lOb, [0.5 0.5 0.5], '-',false);
            self.stateOpt = self.stateIteWS{end};
        end      
        
      % PATH SMOOTHING ***TODO***      
      
    end
    
    %% Delete last plot 
    function delete(self)
      try delete(self.figure_h);end %#ok<TRYNC>
    end
    
    %% InitDisplay
    % *Description:* Plots and outputs some info
    function InitDisplay(self)
      if ~self.doDraw
        return;
      end
      % Output to command window
      fprintf('\n******************************************\n');
      fprintf('***   Rapidly-Exploring Random Trees   ***\n');
      fprintf('******************************************\n\n');
      fprintf('Max. number of steps: %d \n',self.maxIterations);
      fprintf('Max. number of trees: %d \n\n',self.treesMax);
      
      % Output to figure
      try self.figure_h = figure(self.figure_h);
      catch  %#ok<CTCH>
        self.figure_h = figure;
      end
      title('Rapidly-Exploring Random Trees (Step 1)');
      
      % Plot initial node
      try delete(self.startNodePlot_h);end %#ok<TRYNC>
%       self.startNodePlot_h = plot3(self.start(1), self.start(2), ...
%         self.start(3), 'marker', '.', 'color', 'k', 'Parent', ...
%           self.GetAxisHandle());
      
%       % Plot goal node
%       try delete(self.goalNodePlot_h);end %#ok<TRYNC>
%       self.goalNodePlot_h = plot3(self.goal(1),self.goal(2),self.goal(3),'marker','.','color','b','Parent',self.GetAxisHandle());

      % Delete all rrt lines and previous paths
      for i = 1:length(self.plotHandles)
        try delete(self.plotHandles(i).lines);end; %#ok<TRYNC>
      end
      
      % Try and delete the smoothed path line
      try delete(self.smoothedPathPlot_h);end; %#ok<TRYNC>
    end
    
    %% Function SmoothPath
    %
    % *Description:* Shortens a path by repeatedly attempting to connect two
    % randomly selected points along the path together.
    function SmoothPath(self)
      final_path = self.path;
      
      % Perform path smoothing
      for i=1:self.nsmooth
        % Randomly select two path segments
        path_length=size(final_path,1);
        p1=ceil((path_length-1)*rand);
        p2=ceil((path_length-1)*rand);
        while (p2==p1); p2=ceil((path_length-1)*rand); end
        pt=p1; if p1>p2; p1=p2; p2=pt; end
        
        % Randomly select two points from the two path segments
        r1 = rand;
        if final_path(p1,1)>final_path(p1+1,1); pnt1(1)=final_path(p1,1)-abs(final_path(p1,1)-final_path(p1+1,1))*r1; else pnt1(1)=final_path(p1,1)+abs(final_path(p1,1)-final_path(p1+1,1))*r1; end
        if final_path(p1,2)>final_path(p1+1,2); pnt1(2)=final_path(p1,2)-abs(final_path(p1,2)-final_path(p1+1,2))*r1; else pnt1(2)=final_path(p1,2)+abs(final_path(p1,2)-final_path(p1+1,2))*r1; end
        if final_path(p1,3)>final_path(p1+1,3); pnt1(3)=final_path(p1,3)-abs(final_path(p1,3)-final_path(p1+1,3))*r1; else pnt1(3)=final_path(p1,3)+abs(final_path(p1,3)-final_path(p1+1,3))*r1; end
        
        r2 = rand;
        if final_path(p2,1)>final_path(p2+1,1); pnt2(1)=final_path(p2,1)-abs(final_path(p2,1)-final_path(p2+1,1))*r2; else pnt2(1)=final_path(p2,1)+abs(final_path(p2,1)-final_path(p2+1,1))*r2; end
        if final_path(p2,2)>final_path(p2+1,2); pnt2(2)=final_path(p2,2)-abs(final_path(p2,2)-final_path(p2+1,2))*r2; else pnt2(2)=final_path(p2,2)+abs(final_path(p2,2)-final_path(p2+1,2))*r2; end
        if final_path(p2,3)>final_path(p2+1,3); pnt2(3)=final_path(p2,3)-abs(final_path(p2,3)-final_path(p2+1,3))*r2; else pnt2(3)=final_path(p2,3)+abs(final_path(p2,3)-final_path(p2+1,3))*r2; end
        
        % Connect the two points
        if ~self.CollisionCheck(pnt1,pnt2);
          % Update Path
          path1=final_path(1:p1,:);
          path2=[pnt1;pnt2];
          path3=final_path(p2+1:end,:);
          final_path=[path1;path2;path3];
        end
      end
      
      self.smoothedPath = final_path;
      
      % Plot final path
      % DONE:
      if self.plotSmooth
        plotSmoothedPath(self);
      end
    end
    
    % DONE: separated path plotting functions
    function plotSmoothedPath(self)
      if self.doDraw
        self.smoothedPathPlot_h = plot3(self.smoothedPath(:,1),self.smoothedPath(:,2),self.smoothedPath(:,3),'LineWidth',2,'Color','g','Parent',self.GetAxisHandle());
        title_h = get(self.GetAxisHandle(),'title');
        title_string=get(title_h,'String');
        title_string=[title_string,'. Initial(Red), Smoothed(Green)'];
        set(title_h,'String',title_string)
      end
    end
    
    function plotTracedPath(self)
      for t=1:size(self.rrt,2)
        for i=2:size(self.rrt(t).parent,1)
          try delete(self.plotHandles(t).lines(i));end  %#ok<TRYNC>
        end
        try delete(self.plotHandles(t).points);end  %#ok<TRYNC>
      end
      self.plotHandles(t).lines = plot3(self.path(:,1),self.path(:,2),self.path(:,3),'LineWidth',2,'Color','r','Parent',self.GetAxisHandle());
    end
    
  end
  
end