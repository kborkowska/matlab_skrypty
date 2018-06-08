function [max_vel]=max_inc_to_max_vel(max_inc, joint_no)

T = 0.002;
M_PI = 3.14159265358979323846;
enc_res = 4000.0;

if joint_no == 5
    enc_res = 2000.0;
end

max_vel = max_inc*(2.0*M_PI)/(enc_res*T);

end