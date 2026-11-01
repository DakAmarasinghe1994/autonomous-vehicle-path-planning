clear,clc
%RRT Algorithm for path planning for an autonomous vehicle

%% Static

startPos.coord = [0 0.8];
endPos.coord = [0.95 0.1];
radGoal = 0.04;
obst1c = [0.4, 0.8];
obst1r = 0.2;
obst2c = [0.8, 0.5];
obst2r = 0.1;
obst3c = [0.4, 0.35];
obst3r = 0.15;
obst4c = [0.1, 0.2];
obst4r = 0.1;
epsilon = 0.05;

figure(1)
axis([0, 1, 0, 1])
title("RRT Algorithm for 4 static obstacles")
text(-0.05,0.9,'Start');
text(0.95,0.18,'Goal');
text(0.35,0.8,'House');
text(0.05,0.2,'Tree');
text(0.75,0.5,'Rock');
text(0.35,0.35,'Shop');
viscircles(startPos.coord, radGoal, 'Color', [0, 0, 1]);
viscircles(endPos.coord, radGoal, 'Color', [0, 1, 0]);
viscircles(obst1c, obst1r, 'Color', [1, 0, 0]);
viscircles(obst2c, obst2r, 'Color', [1, 0, 0]);
viscircles(obst3c, obst3r, 'Color', [1, 0, 0]);
viscircles(obst4c, obst4r, 'Color', [1, 0, 0]);
axis equal
hold on

% initialize tree
startPos.parent = 0;
nodes(1) = startPos;        

for i = 1:1000

    % Break if goal node is already reached
    if norm(nodes(end).coord - endPos.coord) < radGoal
        break
    end

    % sample random joint position
    if rand(1) < 0.9
        theta_rand = [rand(1), rand(1)];
    else
        theta_rand = endPos.coord;
    end
    plot(theta_rand(1), theta_rand(2), 'o', 'Color',  'k')

    % find node theta_near closest to theta_rand
    ndist = [];
    for j = 1:1:length(nodes)
        n = nodes(j);
        tmp = norm(n.coord - theta_rand);
        ndist = [ndist tmp];
    end
    [val, idx] = min(ndist);
    theta_near = nodes(idx);

    % move towards random position
    if val < epsilon
        theta_new.coord = theta_rand;
    else
        direction = (theta_rand - theta_near.coord) / norm(theta_rand - theta_near.coord);
        theta_new.coord = theta_near.coord + epsilon * direction;
    end

    % if collision free, add to tree
    if norm(theta_new.coord - obst1c) < obst1r
        continue
    end
    if norm(theta_new.coord - obst2c) < obst2r
        continue
    end
    if norm(theta_new.coord - obst3c) < obst3r
        continue
    end
    if norm(theta_new.coord - obst4c) < obst4r
        continue
    end
    line([theta_near.coord(1), theta_new.coord(1)], [theta_near.coord(2), theta_new.coord(2)], 'Color', 'k', 'LineWidth', 1);
    drawnow
    theta_new.parent = idx;
    nodes = [nodes theta_new];

end

% search backwards from closest node to start
ndist = [];
for j = 1:1:length(nodes)
    n = nodes(j);
    tmp = norm(n.coord - endPos.coord);
    ndist = [ndist tmp];
end
[val, idx] = min(ndist);
theta_end = nodes(idx);

% trace the parents of the final node back to the start
while theta_end.parent ~= 0
    start_idx = theta_end.parent;
    start = nodes(start_idx);
    line([theta_end.coord(1), start.coord(1)], [theta_end.coord(2), start.coord(2)], 'Color', [0, 1, 0], 'LineWidth', 3);
    theta_end = start;
end



%% Static and Dynamic

carStart = [-0.15 0.5]; %vehicle starting position
carGoal = [1, 0.5]; %vehicle goal position
rtree = 0.1; 
ctree = [0.95 0.2];
rrock = 0.07;
crock = [0, 0.2];
rhouse = 0.12;
chouse = [0.6, 0.7];
rtruck = 0.04; 
finalGoal.coord = carGoal;
rgoal = 0.05;

truckStart = [0.25, 0.5]; %Truck positions
truckGoal1 = [0.25,0.9];  
truckGoal2 = [0.25,0.1]; 
truckGoal3 = [0.7,0.2]; 
truckGoalMat = [truckGoal1; truckGoal2; truckGoal3];
step = 25;

truckTraj_x1 = linspace(truckStart(1), truckGoal1(1), step); %Truck goal trajectories
truckTraj_y1 = linspace(truckStart(2), truckGoal1(2), step);
truckTraj1 = [truckTraj_x1; truckTraj_y1]'; 
truckTraj_x2 = linspace(truckStart(1), truckGoal2(1), step);
truckTraj_y2 = linspace(truckStart(2), truckGoal2(2), step);
truckTraj2 = [truckTraj_x2; truckTraj_y2]'; 
truckTraj_x3 = linspace(truckStart(1), truckGoal3(1), step);
truckTraj_y3 = linspace(truckStart(2), truckGoal3(2), step);
truckTraj3 = [truckTraj_x3; truckTraj_y3]'; 

%% Testing Prediction
trust = [1/length(truckGoalMat), 1/length(truckGoalMat), 1/length(truckGoalMat)]; % initial trust is even distribution

% if belief in path is less than 28% (threshold), the path can be crossed by the
% vehicle
threshold = 0.28;
figure(2)
hold on
for k = 1:length(truckTraj1)
    viscircles(truckTraj1(k, :), rtruck, 'Color', 'k');
    viscircles(truckTraj2(k, :), rtruck, 'Color', 'k');
    viscircles(truckTraj3(k, :), rtruck, 'Color', 'k'); 
end

% Create for loop that moves along human's path
prePosition = truckStart;

truckPath = randi([1,3]); %Randomly select the path of the truck
%truckPath = 3; % Change to 1 or 2 or 3 to select manually

for i = 2:length(truckTraj1)
    if truckPath == 1 
    truckPosition = [truckTraj1(i, 1), truckTraj1(i, 2)]; %Truck path 1
    elseif truckPath == 2
    truckPosition = [truckTraj2(i, 1), truckTraj2(i, 2)]; %Truck path 2
    else
    truckPosition = [truckTraj3(i, 1), truckTraj3(i, 2)]; %Truck path 3
    end
    viscircles(truckPosition, rtruck, 'Color', 'r');
    %viscircles(truckStart, rtruck, 'Color', 'r');
    expGrow = 0.75;
    R1 = - norm(truckGoal1 - truckPosition);
    R2 = - norm(truckGoal2 - truckPosition);
    R3 = - norm(truckGoal3 - truckPosition);
    diffExp = exp(expGrow * R1) + exp(expGrow * R2) + exp(expGrow * R3);
    ProbDG1 = exp(expGrow * R1) / diffExp;
    ProbDG2 = exp(expGrow * R2) / diffExp;
    ProbDG3 = exp(expGrow * R3) / diffExp;
    
    PriorProb = 1/length(truckGoalMat);
    
    dis1 = norm(truckGoal1 - truckPosition) / norm(truckGoal1 - truckStart);
    dis2 = norm(truckGoal2 - truckPosition) / norm(truckGoal2 - truckStart);
    dis3 = norm(truckGoal3 - truckPosition) / norm(truckGoal3 - truckStart);
    
    ProbDe = ( ProbDG1 + ProbDG2 + ProbDG3) * PriorProb;    
    
    trust(1) = (ProbDG1 * PriorProb ) / ProbDe;
    trust(2) = (ProbDG2 * PriorProb ) / ProbDe;
    trust(3) = (ProbDG3 * PriorProb ) / ProbDe;
    
    RRTPredict(carStart, carGoal, rtree, ctree, rrock, crock, chouse, rhouse, truckStart, truckGoalMat, trust, threshold, rtruck, 0, 1, 0, 1.5)
    prePosition = truckPosition;
end

RRTPredict(carStart, carGoal, rtree, ctree, rrock, crock, chouse, rhouse, truckStart, truckGoalMat, trust, threshold, rtruck, 1, 1, 0, 3)
viscircles(finalGoal.coord, rgoal, 'Color', [1 1 0]);

%% Function
function RRTPredict(startPos, goalPos, r1, c1, r2, c2, c3, r3, truckStart, truckGoalMat, trust, thres, rcar, color1, color2, color3, leng)

rgoal = 0.05;
chouse = c2;
rhouse = r2;
crock = c1;
rrock = r1;
ctree = c3;
rtree = r3;
epsilon = 0.05; %Hyperparameter

truckGoal1 = [truckGoalMat(1,1), truckGoalMat(1,2)];
truckGoal2 = [truckGoalMat(2,1), truckGoalMat(2,2)];
truckGoal3 = [truckGoalMat(3,1), truckGoalMat(3,2)]; 

truckTrajx1 = linspace(truckStart(1), truckGoal1(1), 20);
truckTrajy1 = linspace(truckStart(2), truckGoal1(2), 20);
truckTraj1 = [truckTrajx1; truckTrajy1]';
truckTrajx2 = linspace(truckStart(1), truckGoal2(1), 20);
truckTrajy2 = linspace(truckStart(2), truckGoal2(2), 20);
truckTraj2 = [truckTrajx2; truckTrajy2]';
truckTrajx3 = linspace(truckStart(1), truckGoal3(1), 20);
truckTrajy3 = linspace(truckStart(2), truckGoal3(2), 20);
truckTraj3 = [truckTrajx3; truckTrajy3]';

vehicleStart.coord = startPos; 
vehicleGoal.coord = goalPos; 

axis([0, 1, 0, 1])
title("RRT Algorithm for 3 Static and 1 dynamic obstacles")
viscircles(vehicleStart.coord, rgoal, 'Color', [0 0 1]);
viscircles(vehicleGoal.coord, rgoal, 'Color', [0 1 0]);
viscircles(chouse, rhouse, 'Color', [1 0 0]);
viscircles(crock, rrock, 'Color', [1 0 0]);
viscircles(ctree, rtree, 'Color', [1 0 0]);

text(-0.19,0.4,'Start');
text(0.95,0.4,'Goal');
text(-0.04,0.2,'Rock');
text(0.9,0.2,'Tree');
text(0.55,0.7,'House');
text(0.1,0.5,'Truck');
text(0.3,0.9,'End1');
text(0.3,0.05,'End2');
text(0.68,0.12,'End3');

axis equal
hold on
grid on

% initialize tree
vehicleStart.parent = 0;
nodes(1) = vehicleStart;

for i = 1:1000
    cross1 = false; 
    cross2 = false; 
    cross3 = false;
    % Break if goal node is already reached
    if norm(nodes(end).coord - vehicleGoal.coord) < rgoal
        break
    end
    
    % sample random joint position
    if rand(1) < .9
        theta_rand = [rand(1), rand(1)];
    else
        theta_rand = vehicleGoal.coord;
    end
    
    % find node theta_near closest to theta_rand
    ndist = [];
    for j = 1:1:length(nodes)
        n = nodes(j);
        tmp = norm(n.coord - theta_rand);
        ndist = [ndist tmp];
    end
    [val, idx] = min(ndist);
    theta_near = nodes(idx);
    
    % move towards random position
    if val < epsilon
        theta_new.coord = theta_rand;
    else
        direction = (theta_rand - theta_near.coord) / norm(theta_rand - theta_near.coord);
        theta_new.coord = theta_near.coord + epsilon * direction;
    end
    
    % if collision free, add to tree
    if trust(1) > thres
        for j = 1:length(truckTraj1)
            if norm(theta_new.coord - truckTraj1(j, :)) < rcar
                cross1 = true;
            end
        end
        
        if cross1 == true
            continue
        end
    end
    if trust(2) > thres
        for k = 1:length(truckTraj2)
            if norm(theta_new.coord - truckTraj2(k, :)) < rcar
                cross2 = true;
            end
        end
        if cross2 == true
            continue
        end
    end
    if trust(3) > thres
        for kk = 1:length(truckTraj3)
            if norm(theta_new.coord - truckTraj3(kk, :)) < rcar
                cross3 = true;
            end
        end
        if cross3 == true
            continue
        end
    end
    
    
    if norm(theta_new.coord - chouse) < rhouse
        continue
    end
    if norm(theta_new.coord - crock) < rrock
        continue
    end
    if norm(theta_new.coord - ctree) < rtree
        continue
    end
    
    drawnow;
    theta_new.parent = idx;
    nodes = [nodes theta_new];
    
end

% search backwards from closest node to start
ndist = [];
for j = 1:1:length(nodes)
    n = nodes(j);
    tmp = norm(n.coord - vehicleGoal.coord); 
    ndist = [ndist tmp];
end
[val, idx] = min(ndist);
theta_end = nodes(idx);

% trace the parents of the final node back to the start
while theta_end.parent ~= 0
    start_idx = theta_end.parent;
    start = nodes(start_idx);
    line([theta_end.coord(1), start.coord(1)], [theta_end.coord(2), start.coord(2)], 'Color', [color1, color2, color3], 'LineWidth', leng);
    theta_end = start;
end

end


