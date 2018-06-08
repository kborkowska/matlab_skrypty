%% Script for simulating joint movement
% type of trajectory: trapezoidal
% equations and notations takem from "Trajectory Planning for
%                                     Automatic Machines and Robots"
% written by Luigi Biagiotti and Claudio Melchiorri
% copyright Springer-Verlag Berlin Heidelberg 2008

%% given variables of the movement
% time of start
% t_0 = 0;
% positions (initial and final)
% q_0 = 1;
% q_1 = 2;
% velocities (initial and final)
% v_0 = 0;
% v_1 = 0;
% const vel
% v_v = 2;
% max acceleration
% a_max = -1;
% frequency of controller
% freq = 0.002;

clc
clear all

%% Testing
t_0 = 0.0;
q_0 = 30.0;
q_1 = 0.000000;
v_0 = 0.0;
v_1 = 0.0;
T = 3.46410161513800;
a_max = 10.0;
freq = 0.002;

% SimplestConditionsTest(true)
SimulateTrajectoryDuration(t_0,q_0,q_1,v_0,v_1,T,a_max, freq,...
    'time > T',true);


% Test(true);

%% Calling test functions
function [res,testNo]=Test(plotEnabled)

disp('Starting testing.');

res = 0;
testNo = 0;

[resTmp,testNoTmp]=BoundryPosVelTest(plotEnabled);
res = res + resTmp;
testNo = testNo + testNoTmp;

[resTmp,testNoTmp]=BoundryVelocitiesTest(plotEnabled);
res = res + resTmp;
testNo = testNo + testNoTmp;

[resTmp,testNoTmp]=BoundryPositionsTest(plotEnabled);
res = res + resTmp;
testNo = testNo + testNoTmp;

[resTmp,testNoTmp]=MaximumAccelerationTests(plotEnabled);
res = res + resTmp;
testNo = testNo + testNoTmp;

[resTmp,testNoTmp]=ConstantVelocityTests(plotEnabled);
res = res + resTmp;
testNo = testNo + testNoTmp;

[resTmp,testNoTmp]=SimplestConditionsTest(plotEnabled);
res = res + resTmp;
testNo = testNo + testNoTmp;

disp('Finished testing.');
disp(['Accuracy = ' num2str(res/testNo)]);
disp(['Correct = ' num2str(res)]);
disp(['Number of tests = ' num2str(testNo)]);
end


%% Boundry positions and velocities test
function [res,testNo]=BoundryPosVelTest(plotEnabled)

t_0 = 0;
q_0 = 1;
q_1 = 2;
% v_0 = 0;
% v_1 = 0;
T = 1;
a_max = 12;
freq = 0.002;

res = 0;
testNo  = 0;

% ascending path, positive velocities
v_0 = 1;
v_1 = 0;
resTemp = SimulateTrajectoryDuration(t_0,q_0,q_1,v_0,v_1,T,a_max, freq,...
    '[BPVT] asc path, positive vels',plotEnabled);
res = res + resTemp;
testNo = testNo+1;

v_0 = 0;
v_1 = 1;
resTemp = SimulateTrajectoryDuration(t_0,q_0,q_1,v_0,v_1,T,a_max, freq,...
    '[BPVT] asc path, positive vels',plotEnabled);
res = res + resTemp;
testNo = testNo+1;

v_0 = 1;
v_1 = 1;
resTemp = SimulateTrajectoryDuration(t_0,q_0,q_1,v_0,v_1,T,a_max, freq,...
    '[BPVT] asc path, positive vels', plotEnabled);
res = res + resTemp;
testNo = testNo+1;

% ascending path, negative velocitie
v_0 = -1;
v_1 = 0;
resTemp = SimulateTrajectoryDuration(t_0,q_0,q_1,v_0,v_1,T,a_max, freq,...
    '[BPVT] asc path, negative vels',plotEnabled);
res = res + resTemp;
testNo = testNo+1;

v_0 = 0;
v_1 = -1;
resTemp = SimulateTrajectoryDuration(t_0,q_0,q_1,v_0,v_1,T,a_max, freq,...
    '[BPVT] asc path, negative vels',plotEnabled);
res = res + resTemp;
testNo = testNo+1;

v_0 = -1;
v_1 = -1;
resTemp = SimulateTrajectoryDuration(t_0,q_0,q_1,v_0,v_1,T,a_max, freq,...
    '[BPVT] asc path, negative vels', plotEnabled);
res = res + resTemp;
testNo = testNo+1;

% descending path, negative velocitie
q_0 = 2;
q_1 = 1;

v_0 = -1;
v_1 = 0;
resTemp = SimulateTrajectoryDuration(t_0,q_0,q_1,v_0,v_1,T,a_max, freq,...
    '[BPVT] desc path, negative vels', plotEnabled);
res = res + resTemp;
testNo = testNo+1;

v_0 = 0;
v_1 = -1;
resTemp = SimulateTrajectoryDuration(t_0,q_0,q_1,v_0,v_1,T,a_max, freq,...
    '[BPVT] desc path, negative vels',plotEnabled);
res = res + resTemp;
testNo = testNo+1;

v_0 = -1;
v_1 = -1;
resTemp = SimulateTrajectoryDuration(t_0,q_0,q_1,v_0,v_1,T,a_max, freq,...
    '[BPVT] desc path, negative vels',plotEnabled);
res = res + resTemp;
testNo = testNo+1;

% descending path, positive velocitie
v_0 = 1;
v_1 = 0;
resTemp = SimulateTrajectoryDuration(t_0,q_0,q_1,v_0,v_1,T,a_max, freq,...
    '[BPVT] desc path, positive vels',plotEnabled);
res = res + resTemp;
testNo = testNo+1;

v_0 = 0;
v_1 = 1;
resTemp = SimulateTrajectoryDuration(t_0,q_0,q_1,v_0,v_1,T,a_max, freq,...
    '[BPVT] desc path, positive vels',plotEnabled);
res = res + resTemp;
testNo = testNo+1;

v_0 = 1;
v_1 = 1;
resTemp = SimulateTrajectoryDuration(t_0,q_0,q_1,v_0,v_1,T,a_max, freq,...
    '[BPVT] desc path, positive vels',plotEnabled);
res = res + resTemp;
testNo = testNo+1;

end

%% Boundry positions test
function [res,testNo]=BoundryPositionsTest(plotEnabled)

t_0 = 0;
% q_0 = 1;
% q_1 = 2;
v_0 = 0;
v_1 = 0;
T = 2;
a_max = 7;
freq = 0.002;

res = 0;
testNo  = 0;

% no change in position
q_0 = 1;
q_1 = 1;
resTemp = SimulateTrajectoryDuration(t_0,q_0,q_1,v_0,v_1,T,a_max, freq,...
    '[BPT] no change in position',plotEnabled);
res = res + resTemp;
testNo = testNo+1;

q_0 = -1;
q_1 = -1;
resTemp = SimulateTrajectoryDuration(t_0,q_0,q_1,v_0,v_1,T,a_max, freq,...
    '[BPT] no change in position',plotEnabled);
res = res + resTemp;
testNo = testNo+1;

% negative positions
q_0 = -1;
q_1 = 1;
resTemp = SimulateTrajectoryDuration(t_0,q_0,q_1,v_0,v_1,T,a_max, freq,...
    '[BPT] negative positions',plotEnabled);
res = res + resTemp;
testNo = testNo+1;

q_0 = 1;
q_1 = -1;
resTemp = SimulateTrajectoryDuration(t_0,q_0,q_1,v_0,v_1,T,a_max, freq,...
    '[BPT] negative positions',plotEnabled);
res = res + resTemp;
testNo = testNo+1;

q_0 = -2;
q_1 = -1;
resTemp = SimulateTrajectoryDuration(t_0,q_0,q_1,v_0,v_1,T,a_max, freq,...
    '[BPT] negative positions',plotEnabled);
res = res + resTemp;
testNo = testNo+1;

% descending movement
% q_0 = 1;
% q_1 = -1;
% resTemp = SimulateTrajectoryDuration(t_0,q_0,q_1,v_0,v_1,T,a_max, freq,...
%     '[BPT] descending movement',plotEnabled);
% res = res + resTemp;
% testNo = testNo+1;

q_0 = 3;
q_1 = 2;
resTemp = SimulateTrajectoryDuration(t_0,q_0,q_1,v_0,v_1,T,a_max, freq,...
    '[BPT] descending movement',plotEnabled);
res = res + resTemp;
testNo = testNo+1;

% big distance
q_0 = 0;
q_1 = 5;
resTemp = SimulateTrajectoryDuration(t_0,q_0,q_1,v_0,v_1,T,a_max, freq,...
    '[BPT] big distance',plotEnabled);
res = res + resTemp;
testNo = testNo+1;

end

%% Boundry velocities test
function [res,testNo]=BoundryVelocitiesTest(plotEnabled)

t_0 = 0;
q_0 = 1;
q_1 = 2;
% v_0 = 0;
v_1 = 0;
T = 1;
a_max = 7;
freq = 0.002;

res = 0;
testNo = 0;

% non-zero border velocities
v_0 = 1;
resTemp = SimulateTrajectoryDuration(t_0,q_0,q_1,v_0,v_1,T,a_max, freq,...
    '[BVT] non-zero border velocities',plotEnabled);
res = res + resTemp;
testNo = testNo+1;

v_1 = 1;
resTemp = SimulateTrajectoryDuration(t_0,q_0,q_1,v_0,v_1,T,a_max, freq,...
    '[BVT] non-zero border velocities',plotEnabled);
res = res + resTemp;
testNo = testNo+1;

v_0 = 0;
resTemp = SimulateTrajectoryDuration(t_0,q_0,q_1,v_0,v_1,T,a_max, freq,...
    '[BVT] non-zero border velocities',plotEnabled);
res = res + resTemp;
testNo = testNo+1;

% negative velocites
v_1 = -1;
resTemp = SimulateTrajectoryDuration(t_0,q_0,q_1,v_0,v_1,T,a_max, freq,...
    '[BVT] negative velocites',plotEnabled);
res = res + resTemp;
testNo = testNo+1;

v_0 = -1;
resTemp = SimulateTrajectoryDuration(t_0,q_0,q_1,v_0,v_1,T,a_max, freq,...
    '[BVT] negative velocites',plotEnabled);
res = res + resTemp;
testNo = testNo+1;

v_1 = 0;
resTemp = SimulateTrajectoryDuration(t_0,q_0,q_1,v_0,v_1,T,a_max, freq,...
    '[BVT] negative velocites',plotEnabled);
res = res + resTemp;
testNo = testNo+1;

% velocities higher than the constant velocity
v_0 = 3;
resTemp = SimulateTrajectoryDuration(t_0,q_0,q_1,v_0,v_1,T,a_max, freq,...
    '[BVT] velocities > T',plotEnabled);
res = res + resTemp;
testNo = testNo+1;

v_1 = 3;
resTemp = SimulateTrajectoryDuration(t_0,q_0,q_1,v_0,v_1,T,a_max, freq,...
    '[BVT] velocities > T',plotEnabled);
res = res + resTemp;
testNo = testNo+1;

v_0 = 0;
resTemp = SimulateTrajectoryDuration(t_0,q_0,q_1,v_0,v_1,T,a_max, freq,...
    '[BVT] velocities > T',plotEnabled);
res = res + resTemp;
testNo = testNo+1;

end

%% Simplest conditions test
function [res,testNo]=SimplestConditionsTest(plotEnabled)

t_0 = 0;
q_0 = 1;
q_1 = 2;
v_0 = 0;
v_1 = 0;
T = 1;
a_max = 7;
freq = 0.002;

% symulation for the simplests conditions
res = SimulateTrajectoryDuration(t_0,q_0,q_1,v_0,v_1,T,a_max, freq,...
    '[SVT] simplest conditions',plotEnabled);
testNo = 1;
end

%% Maximum acceleration tests
function [res,testNo]=MaximumAccelerationTests(plotEnabled)

t_0 = 0;
q_0 = 1;
q_1 = 2;
v_0 = 0;
v_1 = 0;
T = 1;
% a_max = 3;
freq = 0.002;

res = 0;
testNo = 0;

% negative acceleration
a_max = -1;
resTemp = SimulateTrajectoryDuration(t_0,q_0,q_1,v_0,v_1,T,a_max, freq,...
    '[MAT] negative acceleration',plotEnabled);
res = res + resTemp;
testNo = testNo+1;


% excessively small acceleration
a_max = 0.2;
resTemp = SimulateTrajectoryDuration(t_0,q_0,q_1,v_0,v_1,T,a_max, freq,...
    '[MAT] excessively small acceleration',plotEnabled);
res = res + resTemp;
testNo = testNo+1;

end

%% Constant velocity tests
function [res,testNo]=ConstantVelocityTests(plotEnabled)

t_0 = 0;
q_0 = 1;
q_1 = 2;
v_0 = 0;
v_1 = 0;
% v_v = 2;
a_max = 7;
freq = 0.002;

res = 0;
testNo = 0;

% negative time
T = -1;
resTemp = SimulateTrajectoryDuration(t_0,q_0,q_1,v_0,v_1,T,a_max, freq,...
    '[CVT] negative time',plotEnabled);
res = res + resTemp;
testNo = testNo+1;

% zero time
T = 0;
resTemp = SimulateTrajectoryDuration(t_0,q_0,q_1,v_0,v_1,T,a_max, freq,...
    '[CVT] zero time',plotEnabled);
res = res + resTemp;
testNo = testNo+1;

% excessively small time
T = 0.2;
resTemp = SimulateTrajectoryDuration(t_0,q_0,q_1,v_0,v_1,T,a_max, freq,...
    '[CVT] excessively small time',plotEnabled);
res = res + resTemp;
testNo = testNo+1;
end





