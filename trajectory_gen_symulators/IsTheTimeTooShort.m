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
