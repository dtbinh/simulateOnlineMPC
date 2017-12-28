function generate_example_obstacles_MPC()

s = 1.0;

% Obstacle 1
obs1 = {[0.4 0.7]*s [0.7 0.7]*s [0.7 1.0]*s [0.4 1.0]*s};

% Obstavle 2
obs2 = {[1.4 0.3]*s [2.0 0.3]*s [2.0 1.4]*s [1.4 1.4]*s};

% Obstavle 3
obs3 = {[0.5 1.7]*s [1.1 1.7]*s [1.1 2.0]*s [0.5 2.0]*s};

vOb = [obs1; obs2; obs3];

save vOb vOb

end