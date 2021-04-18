close all
clear
clc

addpath ../pkg

maxSteps = 200;

dt = 0.4; % timeStep
N = 15;   % timeHorizon
minDist = 2; % seems useless
maxNN = 5;  % meaningless for 2 robot case

MODEL = 'effective'; % enlarge radius
% MODEL = 'holonomic'; % do not enlarge radius

robotIDs = [1 2];
% robotPos = [-1.7 -1.7; 1.7 1.7];
% robotPos = [-1 -1; 1 1]*1.5;
robotPos = [-1 0;0 -1]*1.5;
robotAng = [pi/2; pi/2];
robotLiV = [0.3; 0.3];
robotAgV = [-pi/7; -pi/7];
robotColor = {'r','b'};
robots = RVO2.robotModel.TurtleBot2(robotIDs, robotPos, robotAng, robotLiV, robotAgV, robotColor);

% https://github.com/kobuki-base/kobuki_core/commit/cb24b455a8036c51a9d0464a6a1fefb6eadf1348?branch=cb24b455a8036c51a9d0464a6a1fefb6eadf1348&diff=split#diff-b004872bba157abc8a9c6f3fea2b9e04caa36702e889e227879e4c1f4d5d2e0a

% robotGoals = [1 1;-1 -1];
robotGoals = [1 0;0 1]*1.5;
sim = RVO2.Simulator(dt,N,minDist,maxNN,MODEL);
sim.setAgentGoalList(robotGoals);
sim.defineField(4,4);
sim.setAgentList(robots);
sim.initVisualization();

sim.visualizeVO(1,2,0);
sim.visualizeVO(2,1,0);

sim.visualizeORCA(1);
sim.visualizeORCA(2);

for ii = 1:maxSteps
    sim.doStep();
    sim.updateVisualVO(1, 2, 0);
    sim.updateVisualVO(2, 1, 0);
    pause(dt/2);
end
