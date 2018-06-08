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