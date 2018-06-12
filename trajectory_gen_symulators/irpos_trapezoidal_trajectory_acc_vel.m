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
q_0 = 0.0;
q_1 = 400.0;
q_2 = 100.0;
q_3 = 20.0;
q_4 = 0.0;
v_0 = 0;
v_1 = 0;
v_v = 20;
a_max = 10;
freq = 0.002;


SimulateTrajectoryAccVel(t_0,q_0,q_1,v_0,v_1,v_v,a_max, freq,...
                    '[BVT] velocities > v_v',false);
% SimulateTrajectoryAccVel(t_0,q_1,q_2,v_0,v_1,v_v,a_max, freq,...
%                     '[BVT] velocities > v_v',false);
% SimulateTrajectoryAccVel(t_0,q_2,q_3,v_0,v_1,v_v,a_max, freq,...
%                     '[BVT] velocities > v_v',false);
% SimulateTrajectoryAccVel(t_0,q_3,q_4,v_0,v_1,v_v,a_max, freq,...
%                     '[BVT] velocities > v_v',false);       
% Test(true);

%% Calling test functions
function [res,testNo]=Test(plotEnabled)

disp('Starting testing.');

res = 0;
testNo = 0;

[resTmp,testNoTmp]=BoundryPosVelTests(plotEnabled);
res = res + resTmp;
testNo = testNo + testNoTmp;

[resTmp,testNoTmp]=BoundryVelocitiesTests(plotEnabled);
res = res + resTmp;
testNo = testNo + testNoTmp;

[resTmp,testNoTmp]=BoundryPositionsTests(plotEnabled);
res = res + resTmp;
testNo = testNo + testNoTmp;

[resTmp,testNoTmp]=MaximumAccelerationTests(plotEnabled);
res = res + resTmp;
testNo = testNo + testNoTmp;

[resTmp,testNoTmp]=ConstantVelocityTests(plotEnabled);
res = res + resTmp;
testNo = testNo + testNoTmp;

[resTmp,testNoTmp]=SimplestConditionsTests(plotEnabled);
res = res + resTmp;
testNo = testNo + testNoTmp;

disp('Finished testing.');
disp(['Accuracy = ' num2str(res/testNo)]);
disp(['Correct = ' num2str(res)]);
disp(['Number of tests = ' num2str(testNo)]);

end


%% Boundry positions and velocities test
function [res,testNo]=BoundryPosVelTests(plotEnabled)

t_0 = 0;
q_0 = 1;
q_1 = 2;
% v_0 = 0;
% v_1 = 0;
v_v = 2;
a_max = 4;
freq = 0.002;

res = 0;
testNo = 0;

% ascending path, negative velocitie
v_0 = -1;
v_1 = 0;
[resTmp]=SimulateTrajectoryAccVel(t_0,q_0,q_1,v_0,v_1,v_v,a_max, freq,...
    '[BPVT] asc path, negative vels',plotEnabled);
res = res + resTmp;
testNo = testNo +1;

v_0 = 0;
v_1 = -1;
[resTmp]=SimulateTrajectoryAccVel(t_0,q_0,q_1,v_0,v_1,v_v,a_max, freq,...
    '[BPVT] asc path, negative vels',plotEnabled);
res = res + resTmp;
testNo = testNo +1;

v_0 = -1;
v_1 = -1;
[resTmp]=SimulateTrajectoryAccVel(t_0,q_0,q_1,v_0,v_1,v_v,a_max, freq,...
    '[BPVT] asc path, negative vels',plotEnabled);
res = res + resTmp;
testNo = testNo +1;

% descending path, negative velocitie
q_0 = 2;
q_1 = 1;

v_0 = -1;
v_1 = 0;
[resTmp]=SimulateTrajectoryAccVel(t_0,q_0,q_1,v_0,v_1,v_v,a_max, freq,...
    '[BPVT] desc path, negative vels',plotEnabled);
res = res + resTmp;
testNo = testNo +1;

v_0 = 0;
v_1 = -1;
[resTmp]=SimulateTrajectoryAccVel(t_0,q_0,q_1,v_0,v_1,v_v,a_max, freq,...
    '[BPVT] desc path, negative vels',plotEnabled);
res = res + resTmp;
testNo = testNo +1;

v_0 = -1;
v_1 = -1;
[resTmp]=SimulateTrajectoryAccVel(t_0,q_0,q_1,v_0,v_1,v_v,a_max, freq,...
    '[BPVT] desc path, negative vels',plotEnabled);
res = res + resTmp;
testNo = testNo +1;

% descending path, positive velocitie
v_0 = 1;
v_1 = 0;
[resTmp]=SimulateTrajectoryAccVel(t_0,q_0,q_1,v_0,v_1,v_v,a_max, freq,...
    '[BPVT] desc path, positive vels',plotEnabled);
res = res + resTmp;
testNo = testNo +1;

v_0 = 0;
v_1 = 1;
[resTmp]=SimulateTrajectoryAccVel(t_0,q_0,q_1,v_0,v_1,v_v,a_max, freq,...
    '[BPVT] desc path, positive vels',plotEnabled);
res = res + resTmp;
testNo = testNo +1;

v_0 = 1;
v_1 = 1;
[resTmp]=SimulateTrajectoryAccVel(t_0,q_0,q_1,v_0,v_1,v_v,a_max, freq,...
    '[BPVT] desc path, positive vels',plotEnabled);
res = res + resTmp;
testNo = testNo +1;

% descending path, abs of velocities higher than const velocity
v_0 = 3;
v_1 = 0;
[resTmp]=SimulateTrajectoryAccVel(t_0,q_0,q_1,v_0,v_1,v_v,a_max, freq,...
    '[BPVT] desc path, abs(vels)>v_v',plotEnabled);
res = res + resTmp;
testNo = testNo +1;

v_0 = 0;
v_1 = 3;
[resTmp]=SimulateTrajectoryAccVel(t_0,q_0,q_1,v_0,v_1,v_v,a_max, freq,...
    '[BPVT] desc path, abs(vels)>v_v',plotEnabled);
res = res + resTmp;
testNo = testNo +1;

v_0 = 3;
v_1 = 3;
[resTmp]=SimulateTrajectoryAccVel(t_0,q_0,q_1,v_0,v_1,v_v,a_max, freq,...
    '[BPVT] desc path, abs(vels)>v_v',plotEnabled);
res = res + resTmp;
testNo = testNo +1;

v_0 = -3;
v_1 = 0;
[resTmp]=SimulateTrajectoryAccVel(t_0,q_0,q_1,v_0,v_1,v_v,a_max, freq,...
    '[BPVT] desc path, abs(vels)>v_v',plotEnabled);
res = res + resTmp;
testNo = testNo +1;

v_0 = 0;
v_1 = -3;
[resTmp]=SimulateTrajectoryAccVel(t_0,q_0,q_1,v_0,v_1,v_v,a_max, freq,...
    '[BPVT] desc path, abs(vels)>v_v',plotEnabled);
res = res + resTmp;
testNo = testNo +1;

v_0 = -3;
v_1 = -3;
[resTmp]=SimulateTrajectoryAccVel(t_0,q_0,q_1,v_0,v_1,v_v,a_max, freq,...
    '[BPVT] desc path, abs(vels)>v_v',plotEnabled);
res = res + resTmp;
testNo = testNo +1;

% ascending path, abs of velocities higher than const velocity
q_0 = 1;
q_1 = 2;

v_0 = -3;
v_1 = 0;
[resTmp]=SimulateTrajectoryAccVel(t_0,q_0,q_1,v_0,v_1,v_v,a_max, freq,...
    '[BPVT]asc path, abs(vels)>v_v',plotEnabled);
res = res + resTmp;
testNo = testNo +1;

v_0 = 0;
v_1 = -3;
[resTmp]=SimulateTrajectoryAccVel(t_0,q_0,q_1,v_0,v_1,v_v,a_max, freq,...
    '[BPVT] asc path, abs(vels)>v_v',plotEnabled);
res = res + resTmp;
testNo = testNo +1;

v_0 = -3;
v_1 = -3;
[resTmp]=SimulateTrajectoryAccVel(t_0,q_0,q_1,v_0,v_1,v_v,a_max, freq,...
    '[BPVT] asc path, abs(vels)>v_v',plotEnabled);
res = res + resTmp;
testNo = testNo +1;

end

%% Boundry positions test
function [res,testNo]=BoundryPositionsTests(plotEnabled)

t_0 = 0;
% q_0 = 1;
% q_1 = 2;
v_0 = 0;
v_1 = 0;
v_v = 2;
a_max = 4;
freq = 0.002;

res = 0;
testNo = 0;

% no change in position
q_0 = 1;
q_1 = 1;
[resTmp]=SimulateTrajectoryAccVel(t_0,q_0,q_1,v_0,v_1,v_v,a_max, freq,...
    '[BPT] no change in position',plotEnabled);
res = res + resTmp;
testNo = testNo +1;

q_0 = -1;
q_1 = -1;
[resTmp]=SimulateTrajectoryAccVel(t_0,q_0,q_1,v_0,v_1,v_v,a_max, freq,...
    '[BPT] no change in position',plotEnabled);
res = res + resTmp;
testNo = testNo +1;

% negative positions
q_0 = -1;
q_1 = 1;
[resTmp]=SimulateTrajectoryAccVel(t_0,q_0,q_1,v_0,v_1,v_v,a_max, freq,...
    '[BPT] negative positions',plotEnabled);
res = res + resTmp;
testNo = testNo +1;

q_0 = 1;
q_1 = -1;
[resTmp]=SimulateTrajectoryAccVel(t_0,q_0,q_1,v_0,v_1,v_v,a_max, freq,...
    '[BPT] negative positions',plotEnabled);
res = res + resTmp;
testNo = testNo +1;

q_0 = -2;
q_1 = -1;
[resTmp]=SimulateTrajectoryAccVel(t_0,q_0,q_1,v_0,v_1,v_v,a_max, freq,...
    '[BPT] negative positions',plotEnabled);
res = res + resTmp;
testNo = testNo +1;

% descending movement
q_0 = 1;
q_1 = -1;
[resTmp]=SimulateTrajectoryAccVel(t_0,q_0,q_1,v_0,v_1,v_v,a_max, freq,...
    '[BPT] descending movement',plotEnabled);
res = res + resTmp;
testNo = testNo +1;

q_0 = 3;
q_1 = 2;
[resTmp]=SimulateTrajectoryAccVel(t_0,q_0,q_1,v_0,v_1,v_v,a_max, freq,...
    '[BPT] descending movement',plotEnabled);
res = res + resTmp;
testNo = testNo +1;

% big distance
q_0 = 0;
q_1 = 5;
[resTmp]=SimulateTrajectoryAccVel(t_0,q_0,q_1,v_0,v_1,v_v,a_max, freq,...
    '[BPT] big distance',plotEnabled);
res = res + resTmp;
testNo = testNo +1;

end

%% Boundry velocities test
function [res,testNo]=BoundryVelocitiesTests(plotEnabled)

t_0 = 0;
q_0 = 1;
q_1 = 2;
% v_0 = 0;
v_1 = 0;
v_v = 2;
a_max = 4;
freq = 0.002;

res = 0;
testNo = 0;

% non-zero border velocities
v_0 = 1;
[resTmp]=SimulateTrajectoryAccVel(t_0,q_0,q_1,v_0,v_1,v_v,a_max, freq,...
    '[BVT] non-zero border velocities',plotEnabled);
res = res + resTmp;
testNo = testNo +1;

v_1 = 1;
[resTmp]=SimulateTrajectoryAccVel(t_0,q_0,q_1,v_0,v_1,v_v,a_max, freq,...
    '[BVT] non-zero border velocities',plotEnabled);
res = res + resTmp;
testNo = testNo +1;

v_0 = 0;
[resTmp]=SimulateTrajectoryAccVel(t_0,q_0,q_1,v_0,v_1,v_v,a_max, freq,...
    '[BVT] non-zero border velocities',plotEnabled);
res = res + resTmp;
testNo = testNo +1;

% negative velocites
v_1 = -1;
[resTmp]=SimulateTrajectoryAccVel(t_0,q_0,q_1,v_0,v_1,v_v,a_max, freq,...
    '[BVT] negative velocites',plotEnabled);
res = res + resTmp;
testNo = testNo +1;

v_0 = -1;
[resTmp]=SimulateTrajectoryAccVel(t_0,q_0,q_1,v_0,v_1,v_v,a_max, freq,...
    '[BVT] negative velocites',plotEnabled);
res = res + resTmp;
testNo = testNo +1;

v_1 = 0;
[resTmp]=SimulateTrajectoryAccVel(t_0,q_0,q_1,v_0,v_1,v_v,a_max, freq,...
    '[BVT] negative velocites',plotEnabled);
res = res + resTmp;
testNo = testNo +1;

% velocities higher than the constant velocity
v_0 = 3;
[resTmp]=SimulateTrajectoryAccVel(t_0,q_0,q_1,v_0,v_1,v_v,a_max, freq,...
    '[BVT] velocities > v_v',plotEnabled);
res = res + resTmp;
testNo = testNo +1;

v_1 = 3;
[resTmp]=SimulateTrajectoryAccVel(t_0,q_0,q_1,v_0,v_1,v_v,a_max, freq,...
    '[BVT] velocities > v_v',plotEnabled);
res = res + resTmp;
testNo = testNo +1;

v_0 = 0;
[resTmp]=SimulateTrajectoryAccVel(t_0,q_0,q_1,v_0,v_1,v_v,a_max, freq,...
    '[BVT] velocities > v_v',plotEnabled);
res = res + resTmp;
testNo = testNo +1;

end

%% Simplest conditions test
function [res,testNo]=SimplestConditionsTests(plotEnabled)

t_0 = 0;
q_0 = 1;
q_1 = 2;
v_0 = 0;
v_1 = 0;
v_v = 2;
a_max = 4;
freq = 0.002;

res = 0;
testNo = 0;

% symulation for the simplests conditions
[resTmp]=SimulateTrajectoryAccVel(t_0,q_0,q_1,v_0,v_1,v_v,a_max, freq,...
    '[SVT] simplest conditions',plotEnabled);
res = res + resTmp;
testNo = testNo +1;

end

%% Maximum acceleration tests
function [res,testNo]=MaximumAccelerationTests(plotEnabled)

t_0 = 0;
q_0 = 1;
q_1 = 2;
v_0 = 0;
v_1 = 0;
v_v = 2;
% a_max = 3;
freq = 0.002;

res = 0;
testNo = 0;

% negative acceleration
a_max = -1;
[resTmp]=SimulateTrajectoryAccVel(t_0,q_0,q_1,v_0,v_1,v_v,a_max, freq,...
    '[MAT] negative acceleration',plotEnabled);
res = res + resTmp;
testNo = testNo +1;

% excessively small acceleration
a_max = 0.2;
[resTmp]=SimulateTrajectoryAccVel(t_0,q_0,q_1,v_0,v_1,v_v,a_max, freq,...
    '[MAT] excessively small acceleration',plotEnabled);
res = res + resTmp;
testNo = testNo +1;

end

%% Constant velocity tests
function [res,testNo]=ConstantVelocityTests(plotEnabled)

t_0 = 0;
q_0 = 1;
q_1 = 2;
v_0 = 0;
v_1 = 0;
% v_v = 2;
a_max = 4;
freq = 0.002;

res = 0;
testNo = 0;

% negative velocity
v_v = -1;
[resTmp]=SimulateTrajectoryAccVel(t_0,q_0,q_1,v_0,v_1,v_v,a_max, freq,...
    '[CVT] negative velocity',plotEnabled);
res = res + resTmp;
testNo = testNo +1;

% zero velocity
v_v = 0;
[resTmp]=SimulateTrajectoryAccVel(t_0,q_0,q_1,v_0,v_1,v_v,a_max, freq,...
    '[CVT] zreo velocity',plotEnabled);
res = res + resTmp;
testNo = testNo +1;

% excessively small velocity
v_v = 0.2;
a_max = 3;
[resTmp]=SimulateTrajectoryAccVel(t_0,q_0,q_1,v_0,v_1,v_v,a_max, freq,...
    '[CVT] excessively small velocity',plotEnabled);
res = res + resTmp;
testNo = testNo +1;

end


