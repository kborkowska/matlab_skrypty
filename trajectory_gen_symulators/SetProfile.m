%% set profile using positions, velocities and accelerations
function [ap_coeff, cvp_coeff, dp_coeff]=SetProfile(q_0, q_1, v_0, v_1, v_v,...
    T_a, T_d, t_0, t_1)

% acceleration phase coefficients
ap_coeff=zeros(3,1);
ap_coeff(1) = q_0 - v_0*t_0 + (v_v-v_0)*(t_0^2)/(2*T_a);
ap_coeff(2) = v_0 - (v_v-v_0)*t_0/T_a;
ap_coeff(3) = (v_v-v_0)/(2*T_a);
% constant velocity phase coefficients
cvp_coeff=zeros(3,1);
cvp_coeff(1) = q_0 + (v_0-v_v)*T_a/2 - v_v*t_0;
cvp_coeff(2) = v_v;
cvp_coeff(3) = 0.0;
% deceleration phase coefficients
dp_coeff=zeros(3,1);
dp_coeff(1) = q_1 - v_1*t_1 - (v_v-v_1)*(t_1^2)/(2*T_d);
dp_coeff(2) = v_1 + (v_v-v_1)*t_1/T_d;
dp_coeff(3) = (v_1-v_v)/(2*T_d);

end