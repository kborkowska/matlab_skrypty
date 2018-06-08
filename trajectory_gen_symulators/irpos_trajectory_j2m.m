%% Script showing differences between joint and motor positions
% setting data for joint trajectories

lower_limits = [-0.45 -2.2689280276 -0.6108652382 ...
                -1.5707963268 -10.0 -2.88];
upper_limits = [2.9670597284 -0.872664626 0.6981317008 ...
                1.6057029118 10.0 2.93];

t_0 = [0 0 0 0 0 0];
q_0 = [-0.10063291139240507 -1.5419428654532268 0.019737556833721442 1.1335183568246088 3.658072916666667 -2.7381185214159984];
q_1 = [-0.40 -1.0 0.0 -1.55 0.0 -2.86];
%q_1 = [-0.1 -1.7 0.6 1.0 4.0 2.0];
%q_0 = [2.0 -1.0 0.0 1.0 4.0 2.0];
v_0 = [0 0 0 0 0 0];
v_1 = [0 0 0 0 0 0];
v_v = [0.3 0.3 0.3 0.3 0.3 0.3];
a_max = [0.2 0.2 0.2 0.2 0.2 0.2];
freq = 0.002;
plotEnabled = false;

% get data for plots
[joints]=getJointDataAccVel(t_0,q_0,q_1,v_0,v_1,v_v,a_max, freq, plotEnabled);
[motors]=getMotorData(joints);

% plot results
plotAllJointData(motors, joints);



%% function that produces motor positions
function [motors] = getMotorData(joints)

motors = zeros(size(joints,1),6);
for i=1:size(joints,1)
    motors(i,:)=j2m(joints(i,:));
end

end

%% function that gathers joint positions
function [joints]=getJointDataAccVel(t_0,q_0,q_1,v_0,v_1,v_v,a_max, freq, plotEnabled)

for i=1:6
    [result, positions, velocities, accelerations]=SimulateTrajectoryAccVel(t_0(i),q_0(i),q_1(i),v_0(i),v_1(i),v_v(i),a_max(i),freq, ['joint no.' num2str(i)], plotEnabled);

    if i==1
        joints = zeros(size(positions,1),6);
        joints(:,i) = positions;
    elseif size(positions,1)>size(joints,1)
        tmp = joints;
        joints = zeros(size(positions,1),6);
        for j=1:i
            for k=1:size(tmp,1)
                joints(k,j)=tmp(k,j);
            end
            for l=k:size(positions,1)
                joints(l,j)=tmp(k,j);
            end
        end
        joints(:,i) = positions;
    elseif size(positions,1)<size(joints,1)
        for j=1:size(positions,1)
            joints(j,i) = positions(j);
        end
        for k=j:size(joints,1)
            joints(k,i) = positions(j);
        end
    else
        joints(:,i) = positions;
    end
end

end

%% function that produces graph for all joints
function []=plotAllJointData(motors, joints)

lower_motor_limit_ = [-470.0 -110.0 -80.0 -70.0 -80.0 -1000.0];
upper_motor_limit_ = [450.0 100.0 100.0 380.0 490.0 3000.0];

for i=1:size(motors, 2)
    plotOneJointData(motors(:,i), joints(:,i),...
                     lower_motor_limit_(i),upper_motor_limit_(i),...
                     ['Results for joint no.' num2str(i-1)]);
end

end

%% function that produces graph for one joint
function []=plotOneJointData(motors, joints, lower_motor_limit_,...
                                          upper_motor_limit_, ttl)

figure('Name',ttl,'NumberTitle','off', ...
    'OuterPosition', [330 80 1200 900]);

subplot(2,1,1)
plot(motors);
title('Motor positions')
ylabel('motor pos.')
xlabel('time')

[x,y]=getOverLimitPointsMotor(motors,lower_motor_limit_,...
                                        upper_motor_limit_);

hold on;
plot(x,y,'*r')

subplot(2,1,2)
plot(joints);
title('Joint positions')
ylabel('joint pos.')
xlabel('time')

end

function [x,y]=getOverLimitPointsMotor(motors,lower_motor_limit_,...
                                                upper_motor_limit_)
x=[];
y=[];
j=1;
for i=1:size(motors,1)
    if motors(i,1) < lower_motor_limit_ || ....
        motors(i,1) > upper_motor_limit_
        x(j) = i;
        y(j) = motors(i,1);
        j=j+1;
    end
end

end