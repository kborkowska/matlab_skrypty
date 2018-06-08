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
if t_1 > time_d && time_d > t_1-T_d
    pos_d = Position(time_d, ap_coeff, cvp_coeff, dp_coeff, T_a, T_d, t_1-t_0, t_0);
    if  q_max < pos_d || pos_d < q_min
        time_res(i)=time_d;
        result=true;
    end
end

end