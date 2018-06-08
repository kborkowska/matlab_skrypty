%% Symulation of the trajectory duration
function [result, positions, velocities, accelerations]=SimulateTrajectoryDuration(t_0,q_0,q_1,v_0,v_1,T,a_max, freq, ttl, plotEnabled)
% predefined limits of the joint, it is assumed that this is the last
% possible position
result=0;
q_max = 50;
q_min = -50;

% displacement
h = abs(q_1 - q_0);
s = 1;
if q_1 - q_0 < 0
    s = -1;
end
if (a_max<(2*h-T*(v_0+v_1)+sqrt(4*h^2-4*h*(v_0+v_1)*T+2*(v_0^2+v_1^2)*T^2))/T^2)
    if plotEnabled
        msgbox('[ERROR] Wrong boundry acceleration!!!', ttl);
    end
    return
end
% v_0 = s*v_0;
% v_1 = s*v_1;

% duration of thw whole movement iteration
v_v = 0.5*(v_0+v_1+a_max*T-sqrt((a_max^2)*(T^2)-4*a_max*h+2*a_max*(v_0+v_1)*T-(v_0-v_1)^2));

%a_max=a_max;
% v_0 = s*v_0;
% v_1 = s*v_1;
v_v=s*v_v;
a_max=s*a_max;
h=s*h;
% a_max=s*a_max;

% duration of the acceleration
T_a = (v_v-v_0)/(a_max);
% duration of the deceleration
T_d = (v_v-v_1)/(a_max);


if a_max*h < (abs(v_0^2-v_1^2)/2)
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

% v_0 = s*v_0;
% v_1 = s*v_1;


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
    return
end
ap_coeff
cvp_coeff
dp_coeff

% calculating position, velocity and acceleration for each sample
[positions, velocities, accelerations]=JointTrajectoryData(time, ap_coeff, cvp_coeff,...
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
    if abs(velocities(k)) > (abs(v_v)+0.01)
        txt = '[WARNING] Velocity breach detected';
        result = 0;
    end
end
str2 = [str2 newline txt];

txt = 'No acceleration anomalies detected';
for k=1:iterations
    if (abs(accelerations(k))) > (abs(a_max)+0.01)
%         disp([num2str(abs(a_max)) '-----' num2str(abs(accelerations(k)))])
        txt = '[WARNING] Acceleration breach detected';
        result = 0;
    end
end
str2 = [str2 newline txt];

if plotEnabled ~= true
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

toPlotForLatex(['[DUR]positions; q_0:' num2str(q_0) 'q_1:' num2str(q_1) ...
                'v_0:' num2str(v_0) 'v_1:' num2str(v_1) 'v_max:' num2str(v_v)...
                'a_max:' num2str(a_max)], time, positions);
toPlotForLatex(['[DUR]velocities; q_0:' num2str(q_0) 'q_1:' num2str(q_1) ...
                'v_0:' num2str(v_0) 'v_1:' num2str(v_1) 'v_max:' num2str(v_v)...
                'a_max:' num2str(a_max)], time, velocities);
toPlotForLatex(['[DUR]accelerations; q_0:' num2str(q_0) 'q_1:' num2str(q_1) ...
                'v_0:' num2str(v_0) 'v_1:' num2str(v_1) 'v_max:' num2str(v_v)...
                'a_max:' num2str(a_max)], time, accelerations);
end