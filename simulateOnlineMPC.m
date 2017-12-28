% function simulateOnlineMPC(data_filename, obs_filename, extraArgs)
% simulateOnline_MPC(data_filename, obs_filename, extraArgs)
%     Includes tracking
%
%inputs:
%       data_filename  -  Lookup table for Hybrid Tracking Controller
%       obs_filename   -  Obstacle data
%       extraArgs      -  this structure can be used to leverage other
%                       additional functionalities within this function.
%                       Its subfields are:
%           .visualize      -  Whether to visualize the results

clear; close all; clc

%% Problem setup
start = [0.0; 0.0];
goal = [2.0; 2.0];
trackErr = 0.05; %%%%%%%
sense_range = 1.0;

virt_v = 0.3;

% if nargin < 1
% %   data_filename = 'Q8D_Q4D_RS_1.00aMax.mat'; %???%
%   data_filename = 'Quad10D_g61_dt01_t50_veryHigh_quadratic';
% end


% data_filename = 'Quad10D_g61_dt01_t50_veryHigh_quadratic';
% 
% load(data_filename)
% dt = tau(2) - tau(1);
% delta_x = virt_v*dt;
% 
% % Initial list of tracking error bounds, now in ascending-time order
% TEB_list = flip(TEB(1:end-1));
% min_level = minData(end)*1.01;


% if nargin < 2
%   obs_filename = 'OBS_FILE_NAME_HERE.mat';
% end
% 
% if nargin < 3
%   extraArgs = [];
% end
% 
% % matrix to compare position states (virt vs. true)
% if ~isfield(extraArgs,'Q')
%   Q = zeros(8,4);
%   Q(1,1) = 1;
%   Q(2,2) = 1;
%   Q(5,3) = 1;
%   Q(6,4) = 1;
% end
% 
% if ~isfield(extraArgs, 'visualize')
%   vis = true;
% end

obs_filename = 'vOb.mat';

vis = true;

%% Before Looping
load(obs_filename)

resolution = 0.1;

obsMap = ObstacleMapMPC(vOb, resolution);

% plot global obstacles
if vis
  f = figure;
  f.Color = 'white';
  f.Position = [100 100 1280 720];
%   f.Children.FontSize = 16;
  obsMap.plotGlobal()

%   hold on
  goalTol = 0.1;
  plotDisk(goal(1:2), goalTol, 'b-');

end

% % set initial states to zero
% true_x = zeros(5,1);
% true_x(1:3) = start;
virt_x = start;

% plots handles
hV = plotDisk(virt_x, 0.05, 'r-');
hf = [];
hd = [];

% % Create real quadrotor system
% rl_ui = [2 4 6];
% trueQuad = Quad10D(start_x, dynSysX.uMin(rl_ui), dynSysX.uMax(rl_ui), ...
%   dynSysX.dMin, dynSysX.dMax, 1:10);

%% Start loop! Tracking error bound block
%input: environment, sensing
%output: augmented obstacles
newStates = [];
trueStates = [];
iter = 0;
runtime = [];

max_iter = 5000;
lookup_time = 0;

% while iter < max_iter && norm(trueQuad.x([1 5 9]) - goal) > 0.5
while iter < max_iter && norm(virt_x - goal) > goalTol
  iter = iter + 1;
  
  %% Map Update Block
  % 1. Sense your environment, locate obstacles
  % 2. Expand sensed obstacles by tracking error bound
  obsMap.sense_update(virt_x, sense_range, trackErr)
  
  % Plot current position of our controlled vehicle
  delete(hV);
  hV = plotDisk(virt_x, 0.05, 'r-');
  if vis
    obsMap.plotLocal;
    obsMap.plotPadded;
    obsMap.plotSenseRegion(virt_x,sense_range);
  end
  
  %% Path Planner Block
  % Replan if a new obstacle is seen
  if isempty(newStates) || mod(iter,10) == 0
    if iter ~= 1
        delete(mpc.hT)            
        delete(hf)  
    end
    % Update next virtual state    
    mpc = mpcNextState(virt_x, goal, 10, 10, 0.7, obsMap.vOb);
    
    delta_x = 0.03; % TO BE REMOVED
    
    % interpolate the reference trajectory
    refPath = trajInterp(mpc.stateOpt(:,2:3),delta_x);
    hf = plot(refPath(:,1),refPath(:,2),'k');
    
    % Report total tuntime
    fprintf('\n******************************************\n');
    fprintf('***     MPC Path Planning Complete     ***\n');
    fprintf('******************************************\n');        
    fprintf('***   Total Runtime: %f s   ***\n', mpc.runtimeP2P+mpc.runtimeIteWS+mpc.runtimeOpt);
    fprintf('******************************************\n');
    fprintf('Point-to-point: %f s\n',mpc.runtimeP2P);
    fprintf('Iterative warmstart: %f s\n',mpc.runtimeIteWS);
    fprintf('Final Optimization: %f s\n',mpc.runtimeOpt);
    fprintf('******************************************\n');
    runtime = [runtime mpc.runtimeP2P+mpc.runtimeIteWS+mpc.runtimeOpt];
  end
  
  % Get the next state to track
  nextState = findNextState(virt_x,refPath,delta_x);
  newStates(end+1,:) = nextState;
  
  virt_x = newStates(end,:)';
  
  delete(hd)
  hd = plot(nextState(1),nextState(2),'k.','MarkerSize',10);
  
  %% Hybrid Tracking Controller
  true_x = virt_x; % perfect tracking
  trueStates = [trueStates; virt_x'];
%   % 1. find relative state
%   local_start = tic;
%   rel_x = trueQuad.x - Q*virt_x;
%   
%   % 2. Determine which controller to use, find optimal control
%   %get spatial gradients
%   pX = eval_u(gX, derivX, rel_x(XDims));
%   pY = eval_u(gX, derivX, rel_x(YDims));
%   pZ = eval_u(gZ, derivZ, rel_x(ZDims));
%   
%   % Find optimal control of relative system (no performance control)
%   uX = dynSysX.optCtrl([], rel_x(XDims), pX, uMode);
%   uY = dynSysX.optCtrl([], rel_x(YDims), pY, uMode);
%   uZ = dynSysZ.optCtrl([], rel_x(ZDims), pZ, uMode);
%   u = [uX uY uZ];
%   u = u(rl_ui);
%   lookup_time = lookup_time + toc(local_start);
  
  %% True System Block
  % 1. add random disturbance to velocity within given bound
%   d = dynSysX.dMin + rand(3,1).*(dynSysX.dMax - dynSysX.dMin);
  
  % 2. update state of true vehicle
%   trueQuad.updateState(u, dt, [], d);

%   % Make sure error isn't too big (shouldn't happen)
%   if norm(virt_x - trueQuad.x([1 5 9])) > 3
%     keyboard
%   end
  
  %% Virtual System Block  
%   fprintf('Iteration took %.2f seconds\n', toc);
  
  % Visualize
  if vis
    % Local obstacles and true position
    obsMap.plotLocal;
    obsMap.plotPadded;
    obsMap.plotSenseRegion(virt_x,sense_range);    

    drawnow

%     export_fig(sprintf('pics/%d', iter), '-png')
%     savefig(sprintf('pics/%d.fig', iter))    
  end
end

plot(trueStates(:,1),trueStates(:,2),'g')

fprintf('\n******************************************\n');
fprintf('***         Navigation Complete        ***\n');
fprintf('******************************************\n');        
fprintf('***   Average MPC Runtime: %f s   ***\n', mean(runtime));
fprintf('***   Worst Case Runtime: %f s   ***\n', max(runtime));
fprintf('******************************************\n');