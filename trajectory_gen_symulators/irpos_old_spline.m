duration=3;
position1=0.1;
position2=-1;
velocity1=0;
velocity2=-1;
acceleration1=0;
acceleration2=0;
freq=0.002;
iterations=duration/freq+1;
time=zeros(iterations, 1);

coeffPos=SetProfilePosition(position1, position2, duration);
coeffVel=SetProfileVelocity(position1, position2, velocity1,...
                            velocity2, duration);
coeffAcc=SetProfileAcceleration(position1, position2, velocity1,...
                                velocity2, acceleration1, acceleration2,...
                                duration);

j=1;
for i=0:freq:duration
    time(j)=i;
    j=j+1;
end

[positions, velocities, accelerations]=Data(time, coeffPos);

figure('Name','Position','NumberTitle','off');
title('Only position given')

subplot(3,1,1)
plot(time, positions)
title('Position')
ylabel('position')
xlabel('time')

subplot(3,1,2)
plot(time, velocities)
title('Velocity')
ylabel('velocity')
xlabel('time')

subplot(3,1,3)
plot(time, accelerations)
title('acceleration')
ylabel('acceleration')
xlabel('time')

[positions, velocities, accelerations]=Data(time, coeffVel);

figure('Name','Velocity','NumberTitle','off');
title('Position and velocity given')

subplot(3,1,1)
plot(time, positions)
title('Position')
ylabel('position')
xlabel('time')

subplot(3,1,2)
plot(time, velocities)
title('Velocity')
ylabel('velocity')
xlabel('time')

subplot(3,1,3)
plot(time, accelerations)
title('acceleration')
ylabel('acceleration')
xlabel('time')

[positions, velocities, accelerations]=JointTrajectoryData(time, coeffAcc);

figure('Name','Acceleration','NumberTitle','off');
title('Position, velocity and acceleration given')

subplot(3,1,1)
plot(time, positions)
title('Position')
ylabel('position')
xlabel('time')

subplot(3,1,2)
plot(time, velocities)
title('Velocity')
ylabel('velocity')
xlabel('time')

subplot(3,1,3)
plot(time, accelerations)
title('acceleration')
ylabel('acceleration')
xlabel('time')



 %% set profile using only position
 
 function [coeff]=SetProfilePosition(position1, position2,...
                                duration)
    coeff=zeros(6,1);
 
    coeff(1)=position1;
    coeff(2)=(position2 - position1)/duration;

 end
 
%% set profile using positions and velocities
 function [coeff]=SetProfileVelocity(position1, position2,...
                                  velocity1, velocity2,...
                                  duration)
    coeff=zeros(6,1);
 
    coeff(1)=position1;
    coeff(2)=velocity1;
    coeff(3)=(-3.0 * position1 + 3.0 * position2 -...
               2.0 * velocity1 * duration -...
               velocity2 * duration)/duration^2;
    coeff(4)=(2.0 * position1 - 2.0 * position2 +...
              velocity1 * duration + velocity2 * duration)/duration^3;

 end
 
 %% set profile using positions, velocities and accelerations
 function [coeff]=SetProfileAcceleration(position1, position2,...
                                  velocity1, velocity2,...
                                  acceleration1, acceleration2,...
                                  duration)
    coeff=zeros(6,1);
 
    coeff(1)=position1;
    coeff(2)=velocity1;
    coeff(3)=0.5 * acceleration1;
    coeff(4)=(-20.0 * position1 + 20.0 * position2 -...
              3.0 * acceleration1 * duration^2 +...
              acceleration2 * duration^2 - 12.0 * velocity1 * duration -...
              8.0 * velocity2 * duration)/(2.0 * duration^3);
    coeff(5)=(30.0 * position1 - 30.0 * position2 +...
              3.0 * acceleration1 * duration^2 -...
              2.0 * acceleration2 * duration^2 +...
              16.0 * velocity1 * duration +...
              14.0 * velocity2 * duration)/(2.0 * duration^4);
    coeff(6)=(-12.0 * position1 + 12.0 * position2 -...
              acceleration1 * duration^2 + acceleration2 * duration^2 -...
              6.0 * velocity1 * duration -...
              6.0 * velocity2 * duration)/(2.0 * duration^5);
          
 end
 
 

 