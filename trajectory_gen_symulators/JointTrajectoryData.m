%% produces data
function [positions, velocities, accelerations]=JointTrajectoryData(time, ap_coeff,...
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