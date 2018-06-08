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