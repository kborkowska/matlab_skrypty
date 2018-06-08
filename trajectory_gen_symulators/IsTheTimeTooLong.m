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