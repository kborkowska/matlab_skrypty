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
q_0 = 1.0;
q_1 = -0.3;
v_0 = 0;
v_1 = 0;
v_v = 5;
a_max = 8;
freq = 0.002;


SimulateTrajectory(t_0,q_0,q_1,v_0,v_1,v_v,a_max, freq,...
                    '[BVT] velocities > v_v',true);


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
[resTmp]=SimulateTrajectory(t_0,q_0,q_1,v_0,v_1,v_v,a_max, freq,...
    '[BPVT] asc path, negative vels',plotEnabled);
res = res + resTmp;
testNo = testNo +1;

v_0 = 0;
v_1 = -1;
[resTmp]=SimulateTrajectory(t_0,q_0,q_1,v_0,v_1,v_v,a_max, freq,...
    '[BPVT] asc path, negative vels',plotEnabled);
res = res + resTmp;
testNo = testNo +1;

v_0 = -1;
v_1 = -1;
[resTmp]=SimulateTrajectory(t_0,q_0,q_1,v_0,v_1,v_v,a_max, freq,...
    '[BPVT] asc path, negative vels',plotEnabled);
res = res + resTmp;
testNo = testNo +1;

% descending path, negative velocitie
q_0 = 2;
q_1 = 1;

v_0 = -1;
v_1 = 0;
[resTmp]=SimulateTrajectory(t_0,q_0,q_1,v_0,v_1,v_v,a_max, freq,...
    '[BPVT] desc path, negative vels',plotEnabled);
res = res + resTmp;
testNo = testNo +1;

v_0 = 0;
v_1 = -1;
[resTmp]=SimulateTrajectory(t_0,q_0,q_1,v_0,v_1,v_v,a_max, freq,...
    '[BPVT] desc path, negative vels',plotEnabled);
res = res + resTmp;
testNo = testNo +1;

v_0 = -1;
v_1 = -1;
[resTmp]=SimulateTrajectory(t_0,q_0,q_1,v_0,v_1,v_v,a_max, freq,...
    '[BPVT] desc path, negative vels',plotEnabled);
res = res + resTmp;
testNo = testNo +1;

% descending path, positive velocitie
v_0 = 1;
v_1 = 0;
[resTmp]=SimulateTrajectory(t_0,q_0,q_1,v_0,v_1,v_v,a_max, freq,...
    '[BPVT] desc path, positive vels',plotEnabled);
res = res + resTmp;
testNo = testNo +1;

v_0 = 0;
v_1 = 1;
[resTmp]=SimulateTrajectory(t_0,q_0,q_1,v_0,v_1,v_v,a_max, freq,...
    '[BPVT] desc path, positive vels',plotEnabled);
res = res + resTmp;
testNo = testNo +1;

v_0 = 1;
v_1 = 1;
[resTmp]=SimulateTrajectory(t_0,q_0,q_1,v_0,v_1,v_v,a_max, freq,...
    '[BPVT] desc path, positive vels',plotEnabled);
res = res + resTmp;
testNo = testNo +1;

% descending path, abs of velocities higher than const velocity
v_0 = 3;
v_1 = 0;
[resTmp]=SimulateTrajectory(t_0,q_0,q_1,v_0,v_1,v_v,a_max, freq,...
    '[BPVT] desc path, abs(vels)>v_v',plotEnabled);
res = res + resTmp;
testNo = testNo +1;

v_0 = 0;
v_1 = 3;
[resTmp]=SimulateTrajectory(t_0,q_0,q_1,v_0,v_1,v_v,a_max, freq,...
    '[BPVT] desc path, abs(vels)>v_v',plotEnabled);
res = res + resTmp;
testNo = testNo +1;

v_0 = 3;
v_1 = 3;
[resTmp]=SimulateTrajectory(t_0,q_0,q_1,v_0,v_1,v_v,a_max, freq,...
    '[BPVT] desc path, abs(vels)>v_v',plotEnabled);
res = res + resTmp;
testNo = testNo +1;

v_0 = -3;
v_1 = 0;
[resTmp]=SimulateTrajectory(t_0,q_0,q_1,v_0,v_1,v_v,a_max, freq,...
    '[BPVT] desc path, abs(vels)>v_v',plotEnabled);
res = res + resTmp;
testNo = testNo +1;

v_0 = 0;
v_1 = -3;
[resTmp]=SimulateTrajectory(t_0,q_0,q_1,v_0,v_1,v_v,a_max, freq,...
    '[BPVT] desc path, abs(vels)>v_v',plotEnabled);
res = res + resTmp;
testNo = testNo +1;

v_0 = -3;
v_1 = -3;
[resTmp]=SimulateTrajectory(t_0,q_0,q_1,v_0,v_1,v_v,a_max, freq,...
    '[BPVT] desc path, abs(vels)>v_v',plotEnabled);
res = res + resTmp;
testNo = testNo +1;

% ascending path, abs of velocities higher than const velocity
q_0 = 1;
q_1 = 2;

v_0 = -3;
v_1 = 0;
[resTmp]=SimulateTrajectory(t_0,q_0,q_1,v_0,v_1,v_v,a_max, freq,...
    '[BPVT]asc path, abs(vels)>v_v',plotEnabled);
res = res + resTmp;
testNo = testNo +1;

v_0 = 0;
v_1 = -3;
[resTmp]=SimulateTrajectory(t_0,q_0,q_1,v_0,v_1,v_v,a_max, freq,...
    '[BPVT] asc path, abs(vels)>v_v',plotEnabled);
res = res + resTmp;
testNo = testNo +1;

v_0 = -3;
v_1 = -3;
[resTmp]=SimulateTrajectory(t_0,q_0,q_1,v_0,v_1,v_v,a_max, freq,...
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
[resTmp]=SimulateTrajectory(t_0,q_0,q_1,v_0,v_1,v_v,a_max, freq,...
    '[BPT] no change in position',plotEnabled);
res = res + resTmp;
testNo = testNo +1;

q_0 = -1;
q_1 = -1;
[resTmp]=SimulateTrajectory(t_0,q_0,q_1,v_0,v_1,v_v,a_max, freq,...
    '[BPT] no change in position',plotEnabled);
res = res + resTmp;
testNo = testNo +1;

% negative positions
q_0 = -1;
q_1 = 1;
[resTmp]=SimulateTrajectory(t_0,q_0,q_1,v_0,v_1,v_v,a_max, freq,...
    '[BPT] negative positions',plotEnabled);
res = res + resTmp;
testNo = testNo +1;

q_0 = 1;
q_1 = -1;
[resTmp]=SimulateTrajectory(t_0,q_0,q_1,v_0,v_1,v_v,a_max, freq,...
    '[BPT] negative positions',plotEnabled);
res = res + resTmp;
testNo = testNo +1;

q_0 = -2;
q_1 = -1;
[resTmp]=SimulateTrajectory(t_0,q_0,q_1,v_0,v_1,v_v,a_max, freq,...
    '[BPT] negative positions',plotEnabled);
res = res + resTmp;
testNo = testNo +1;

% descending movement
q_0 = 1;
q_1 = -1;
[resTmp]=SimulateTrajectory(t_0,q_0,q_1,v_0,v_1,v_v,a_max, freq,...
    '[BPT] descending movement',plotEnabled);
res = res + resTmp;
testNo = testNo +1;

q_0 = 3;
q_1 = 2;
[resTmp]=SimulateTrajectory(t_0,q_0,q_1,v_0,v_1,v_v,a_max, freq,...
    '[BPT] descending movement',plotEnabled);
res = res + resTmp;
testNo = testNo +1;

% big distance
q_0 = 0;
q_1 = 5;
[resTmp]=SimulateTrajectory(t_0,q_0,q_1,v_0,v_1,v_v,a_max, freq,...
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
[resTmp]=SimulateTrajectory(t_0,q_0,q_1,v_0,v_1,v_v,a_max, freq,...
    '[BVT] non-zero border velocities',plotEnabled);
res = res + resTmp;
testNo = testNo +1;

v_1 = 1;
[resTmp]=SimulateTrajectory(t_0,q_0,q_1,v_0,v_1,v_v,a_max, freq,...
    '[BVT] non-zero border velocities',plotEnabled);
res = res + resTmp;
testNo = testNo +1;

v_0 = 0;
[resTmp]=SimulateTrajectory(t_0,q_0,q_1,v_0,v_1,v_v,a_max, freq,...
    '[BVT] non-zero border velocities',plotEnabled);
res = res + resTmp;
testNo = testNo +1;

% negative velocites
v_1 = -1;
[resTmp]=SimulateTrajectory(t_0,q_0,q_1,v_0,v_1,v_v,a_max, freq,...
    '[BVT] negative velocites',plotEnabled);
res = res + resTmp;
testNo = testNo +1;

v_0 = -1;
[resTmp]=SimulateTrajectory(t_0,q_0,q_1,v_0,v_1,v_v,a_max, freq,...
    '[BVT] negative velocites',plotEnabled);
res = res + resTmp;
testNo = testNo +1;

v_1 = 0;
[resTmp]=SimulateTrajectory(t_0,q_0,q_1,v_0,v_1,v_v,a_max, freq,...
    '[BVT] negative velocites',plotEnabled);
res = res + resTmp;
testNo = testNo +1;

% velocities higher than the constant velocity
v_0 = 3;
[resTmp]=SimulateTrajectory(t_0,q_0,q_1,v_0,v_1,v_v,a_max, freq,...
    '[BVT] velocities > v_v',plotEnabled);
res = res + resTmp;
testNo = testNo +1;

v_1 = 3;
[resTmp]=SimulateTrajectory(t_0,q_0,q_1,v_0,v_1,v_v,a_max, freq,...
    '[BVT] velocities > v_v',plotEnabled);
res = res + resTmp;
testNo = testNo +1;

v_0 = 0;
[resTmp]=SimulateTrajectory(t_0,q_0,q_1,v_0,v_1,v_v,a_max, freq,...
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
[resTmp]=SimulateTrajectory(t_0,q_0,q_1,v_0,v_1,v_v,a_max, freq,...
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
[resTmp]=SimulateTrajectory(t_0,q_0,q_1,v_0,v_1,v_v,a_max, freq,...
    '[MAT] negative acceleration',plotEnabled);
res = res + resTmp;
testNo = testNo +1;

% excessively small acceleration
a_max = 0.2;
[resTmp]=SimulateTrajectory(t_0,q_0,q_1,v_0,v_1,v_v,a_max, freq,...
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
[resTmp]=SimulateTrajectory(t_0,q_0,q_1,v_0,v_1,v_v,a_max, freq,...
    '[CVT] negative velocity',plotEnabled);
res = res + resTmp;
testNo = testNo +1;

% zero velocity
v_v = 0;
[resTmp]=SimulateTrajectory(t_0,q_0,q_1,v_0,v_1,v_v,a_max, freq,...
    '[CVT] zreo velocity',plotEnabled);
res = res + resTmp;
testNo = testNo +1;

% excessively small velocity
v_v = 0.2;
a_max = 3;
[resTmp]=SimulateTrajectory(t_0,q_0,q_1,v_0,v_1,v_v,a_max, freq,...
    '[CVT] excessively small velocity',plotEnabled);
res = res + resTmp;
testNo = testNo +1;

end

%% Symulation of the trajectory
function [result]=SimulateTrajectory(t_0,q_0,q_1,v_0,v_1,v_v,a_max, freq, ttl, plotEnabled)
% predefined limits of the joint, it is assumed that this is the last
% possible position
result = 0;
q_max = 4;
q_min = -5;

% displacement
h = abs(q_1 - q_0);
s = 1;
if q_1 - q_0 < 0
    s = -1;
end

% % check for impossible trajectory (boundry vel out of range)
% if (s*v_0 - v_v > 0) || (s*v_1 - v_v > 0)
%     if plotEnabled
%         msgbox('[ERROR] Wrong boundry velocity!!!', ttl);
%     end
%     return
% end

if h==0
    if plotEnabled
        msgbox('Joint already in position', ttl);
    end
    return
end
v_v=v_v*s;

% duration of the acceleration
T_a = abs(v_v-v_0)/a_max;
% duration of the deceleration
T_d = abs(v_v-v_1)/a_max;

a_max=a_max*s;
% duration of thw whole movement iteration
T = s*h/v_v + (v_v/(2*a_max))*(((1- v_0/v_v)^2)+((1- v_1/v_v)^2));
% v_0 = s*v_0;
% v_1 = s*v_1;
unReachVel=false;

if s*a_max*h < (abs(v_0^2-v_1^2)/2)
    %disp('Trajectory not possible!')
    if plotEnabled
        msgbox(['Trajectory data:' newline ...
            '                                            ' newline ...
            't_0 = ' num2str(t_0) newline ...
            'q_0 = ' num2str(q_0) newline ...
            'q_1 = ' num2str(q_1) newline ...
            'v_0 = ' num2str(v_0) newline ...'
            'v_1 = ' num2str(v_1) newline ...
            'v_v = ' num2str(v_v) newline ...
            'a_{max} = ' num2str(a_max) newline,...
            '                                                   ' newline ...
            '[ERROR] Trajectory not possible!'], ttl)
    end
    return
end

if s*h*a_max<(v_v^2-(v_0^2+v_1^2)/2)
    unReachVel=true;
    v_v=s*sqrt(s*h*a_max + (v_0^2 + v_1^2)/2);
    T_a=abs(v_v-v_0)/(s*a_max);
    T_d=abs(v_v-v_1)/(s*a_max);
    T=T_a+T_d;
end
% v_0 = s*v_0;
% v_1 = s*v_1;

if T<0 || isnan(T) || isinf(T)
    if plotEnabled
        msgbox(['Trajectory data:' newline ...
            '                                                   ' newline ...
            't_0 = ' num2str(t_0) newline ...
            'q_0 = ' num2str(q_0) newline ...
            'q_1 = ' num2str(q_1) newline ...
            'v_0 = ' num2str(v_0) newline ...'
            'v_1 = ' num2str(v_1) newline ...
            'v_v = ' num2str(v_v) newline ...
            'a_{max} = ' num2str(a_max) newline,...
            '                                                   ' newline ...
            '[ERROR] Something wrong with the time! Most likely, v_v is non-positive'], ttl)
    end
    return
end

% time of the end of the movement
t_1 = t_0 + T;

% no. iterations
iterations=floor(T/freq+1);

% sampling times
time=zeros(iterations, 1);

j=1;
for i=t_0:freq:(t_0+T)
    time(j)=i;
    j=j+1;
end

% v_v=s*v_v;
%a_max=s*a_max;

% calculating coefficients
[ap_coeff, cvp_coeff, dp_coeff]=SetProfile(q_0, q_1, v_0, v_1, v_v, T_a,...
    T_d, t_0, t_1);

%checking for position jumps
if IsTheTimeTooShort(ap_coeff, cvp_coeff, T_a, t_0)
    str = ['Trajectory data:' newline ...
        '                                                   ' newline ...
        't_0 = ' num2str(t_0) newline ...
        'q_0 = ' num2str(q_0) newline ...
        'q_1 = ' num2str(q_1) newline ...
        'v_0 = ' num2str(v_0) newline ...'
        'v_1 = ' num2str(v_1) newline ...
        'v_v = ' num2str(v_v) newline ...
        'a_{max} = ' num2str(a_max) newline,...
        '                                                   ' newline ...
        '[ERROR] Time too short tor that trajectory'];
    if plotEnabled
        msgbox(str, ttl);
    end
%     return
end

if IsTheTimeTooLong(cvp_coeff, dp_coeff, T_d, T,t_0)
    str = ['Trajectory data:' newline ...
        '                                                   ' newline ...
        't_0 = ' num2str(t_0) newline ...
        'q_0 = ' num2str(q_0) newline ...
        'q_1 = ' num2str(q_1) newline ...
        'v_0 = ' num2str(v_0) newline ...'
        'v_1 = ' num2str(v_1) newline ...
        'v_v = ' num2str(v_v) newline ...
        'a_{max} = ' num2str(a_max) newline,...
        '                                                   ' newline ...
        '[ERROR] Time too long tor that trajectory'];
    if plotEnabled
        msgbox(str, ttl);
    end
    return
end

% check for possible position limit breach
[res, time_res] = IsPositionLimitBreached(q_0, q_1, q_max, q_min,...
    ap_coeff, cvp_coeff,  dp_coeff,...
    t_0, t_1, T_a, T_d);
if res
    str = ['Trajectory data:' newline ...
        '                                                   ' newline ...
        't_0 = ' num2str(t_0) newline ...
        'q_0 = ' num2str(q_0) newline ...
        'q_1 = ' num2str(q_1) newline ...
        'v_0 = ' num2str(v_0) newline ...'
        'v_1 = ' num2str(v_1) newline ...
        'v_v = ' num2str(v_v) newline ...
        'a_{max} = ' num2str(a_max) newline,...
        '                                                   ' newline ...
        '[ERROR] Position limit breached at time (peaks): '];
    for j=1:length(time_res)
        str = [str newline num2str(j) ': ' num2str(time_res(j))];
    end
    if plotEnabled
        msgbox(str, ttl);
    end
    %         return
end
ap_coeff
cvp_coeff
dp_coeff
% calculating position, velocity and acceleration for each sample
[positions, velocities, accelerations]=Data(time, ap_coeff, cvp_coeff,...
    dp_coeff, T_a, T_d, T, t_0);

str = ['Trajectory data:' newline ...
    't_0 = ' num2str(t_0) newline ...
    'q_0 = ' num2str(q_0) ...
    '      q_1 = ' num2str(q_1) newline ...
    'v_0 = ' num2str(v_0) ...
    '      v_1 = ' num2str(v_1) newline ...
    'v_v = ' num2str(v_v) ...
    '      a_{max} = ' num2str(a_max)];

str2='Results:';

if unReachVel
    str2 = [str2 newline 'Unable to achieve maximum velocity'];
end
result = 1;
txt = 'No position anomalies detected';
for k=2:iterations
    if 0.05*h < abs(positions(k)-positions(k-1))
        txt = '[WARNING] Position jump detected';
        result = 0;
    end
end
str2 = [str2 newline txt];

txt = 'No velocity anomalies detected';
for k=1:iterations
    if abs(velocities(k)) > abs(v_v)
        txt = '[WARNING] Velocity breach detected';
        result = 0;
    end
end
str2 = [str2 newline txt];

txt = 'No acceleration anomalies detected';
for k=1:iterations
    if abs(accelerations(k)) > abs(a_max)
        txt = '[WARNING] Acceleration breach detected';
        result = 0;
    end
end
str2 = [str2 newline txt];

if plotEnabled == false
    return
end

figure('Name',ttl,'NumberTitle','off', ...
    'OuterPosition', [330 80 1200 900]);

subplot(4,1,1)
plot(time, positions)
title('Position')
ylabel('position')
xlabel('time')

subplot(4,1,2)
plot(time, velocities)
title('Velocity')
ylabel('velocity')
xlabel('time')

subplot(4,1,3)
plot(time, accelerations)
title('Acceleration')
ylabel('acceleration')
xlabel('time')

subplot(4,1,4)
axis('off')
text(.05,.05,str,'HorizontalAlignment','left', 'Units', 'normalized',...
    'Fontsize', 12, 'Color', 'magenta')
text(.55,.05,str2,'HorizontalAlignment','left', 'Units', 'normalized',...
    'Fontsize', 14, 'Color', 'magenta')

end

%% Checking wether a position limit might breached
function [result, time_res]=IsPositionLimitBreached(q_0, q_1, q_max,...
    q_min, ap_coeff,...
    cvp_coeff,  dp_coeff,...
    t_0, t_1, T_a, T_d)
time_res=999999;
i=1;
result=false;
time_a = (-1)*ap_coeff(2)/(2*ap_coeff(3));
time_d = (-1)*dp_coeff(2)/(2*dp_coeff(3));

if (q_max < q_0 || q_0 < q_min)
    time_res(i)=t_0;
    i=i+1;
    result=true;
end
if (q_max < q_1 || q_1 < q_min)
    time_res(i)=t_1;
    i=i+1;
    result=true;
end
if t_0 < time_a && time_a < t_0+T_a
    pos_a = Position(time_a, ap_coeff, cvp_coeff, dp_coeff, T_a, T_d, t_1-t_0, t_0);
    if  q_max < pos_a || pos_a < q_min
        time_res(i)=time_a;
        i=i+1;
        result=true;
    end
end
if t_1 > time_d || time_d > t_1-T_d
    pos_d = Position(time_d, ap_coeff, cvp_coeff, dp_coeff, T_a, T_d, t_1-t_0, t_0);
    if  q_max < pos_d || pos_d < q_min
        time_res(i)=time_d;
        result=true;
    end
end

end

%% Checking wether there is a position jump
function [result]=IsTheTimeTooShort(ap_coeff, cvp_coeff, T_a, t_0)
    result = false;
    epsi = 0.001;
    time = T_a+t_0;
    pos_v = cvp_coeff(1) + time*cvp_coeff(2) + (time^2)*cvp_coeff(3);
    pos_a = ap_coeff(1) + time*ap_coeff(2) + (time^2)*ap_coeff(3);
    if pos_v < pos_a-epsi || pos_v > pos_a+epsi
        result = true;
    end
end

%% Checking wether there is a position jump
function [result]=IsTheTimeTooLong(cvp_coeff, dp_coeff, T_d, T,t_0)
    result = false;
    epsi = 0.001;
    time = t_0+T-T_d;
    pos_v = cvp_coeff(1) + time*cvp_coeff(2) + (time^2)*cvp_coeff(3);
    pos_d = dp_coeff(1) + time*dp_coeff(2) + (time^2)*dp_coeff(3);
    if pos_v < pos_d-epsi || pos_v > pos_d+epsi
        result = true;
    end
end

%% set profile using positions, velocities and accelerations
function [ap_coeff, cvp_coeff, dp_coeff]=SetProfile(q_0, q_1, v_0, v_1, v_v,...
    T_a, T_d, t_0, t_1)

% acceleration phase coefficients
ap_coeff=zeros(3,1);
ap_coeff(1) = q_0 - v_0*t_0 + (v_v-v_0)*(t_0^2)/(2*T_a);
ap_coeff(2) = v_0 - (v_v-v_0)*t_0/T_a;
ap_coeff(3) = (v_v-v_0)/(2*T_a);
% constant velocity phase coefficients
cvp_coeff=zeros(3,1);
cvp_coeff(1) = q_0 + (v_0-v_v)*T_a/2 - v_v*t_0;
cvp_coeff(2) = v_v;
cvp_coeff(3) = 0.0;
% deceleration phase coefficients
dp_coeff=zeros(3,1);
dp_coeff(1) = q_1 - v_1*t_1 - (v_v-v_1)*(t_1^2)/(2*T_d);
dp_coeff(2) = v_1 + (v_v-v_1)*t_1/T_d;
dp_coeff(3) = (v_1-v_v)/(2*T_d);

end

%% get position for a given moment
function [position]=Position(time, ap_coeff, cvp_coeff, dp_coeff, T_a, T_d, T, t_0)
position=0;

if time < t_0+T_a
    position = ap_coeff(1) + time*ap_coeff(2) + (time^2)*ap_coeff(3);
elseif time < t_0+T-T_d
    position = cvp_coeff(1) + time*cvp_coeff(2) + (time^2)*cvp_coeff(3);
else
    position = dp_coeff(1) + time*dp_coeff(2) + (time^2)*dp_coeff(3);
end

end


%% get velocity for a given moment
function [velocity]=Velocity(time, ap_coeff, cvp_coeff, dp_coeff, T_a, T_d, T, t_0)
velocity=0;

if time < t_0+T_a
    velocity = ap_coeff(2) + 2.0*time*ap_coeff(3);
elseif time < t_0+T-T_d
    velocity = cvp_coeff(2) + 2.0 *time*cvp_coeff(3);
else
    velocity = dp_coeff(2) + 2.0 *time*dp_coeff(3);
end

end


%% get velocity for a given moment
function [acceleration]=Acceleration(time, ap_coeff, cvp_coeff, dp_coeff,...
    T_a, T_d, T, t_0)
acceleration=0;

if time < t_0+T_a
    acceleration = 2.0*ap_coeff(3);
elseif time < t_0+T-T_d
    acceleration = 2.0*cvp_coeff(3);
else
    acceleration = 2.0*dp_coeff(3);
end

end


%% produces data
function [positions, velocities, accelerations]=Data(time, ap_coeff,...
    cvp_coeff, dp_coeff,...
    T_a, T_d, T, t_0)

iterations=length(time);
positions=zeros(iterations, 1);
velocities=zeros(iterations, 1);
accelerations=zeros(iterations, 1);

for i=1:iterations
    positions(i)=Position(time(i), ap_coeff, cvp_coeff, dp_coeff, T_a,...
        T_d, T, t_0);
    velocities(i)=Velocity(time(i), ap_coeff, cvp_coeff, dp_coeff, T_a,...
        T_d, T, t_0);
    accelerations(i)=Acceleration(time(i), ap_coeff, cvp_coeff,...
        dp_coeff, T_a, T_d, T, t_0);
end

end
