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