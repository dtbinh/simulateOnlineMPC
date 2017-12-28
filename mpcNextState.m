function mpc = mpcNextState(start, goal, N, N_opt, Ts, vOb)
% [newState, path, rrt] = rrtNextState(delta_x, start) 

if nargin < 1
    start = [0.0; 0.0];
end

if iscolumn(start)
    start = start';
end

if nargin < 2
    goal = [2.0; 2.0];
end

if iscolumn(goal)
    goal = goal';
end

if nargin < 3
    load('vOb.mat');    
end

lOb = getlOb([vOb{1,1};vOb{1,2};vOb{1,3};vOb{1,4}],...
             [vOb{2,1};vOb{2,2};vOb{2,3};vOb{2,4}],...
             [vOb{3,1};vOb{3,2};vOb{3,3};vOb{3,4}]);

if nargin < 4
  delta_x = 0.1;
end

if nargin < 5
  mpcSoFar = [];
end

if nargin < 6
  vis = true;
end

% runs everything
mpc = MpcPlanner(start, goal, N, N_opt, Ts, lOb);

mpc.Run()

end