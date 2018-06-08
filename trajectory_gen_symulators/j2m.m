%% calculates motor position from the one given in joint positions
function [motors] = j2m(joints)

% stuff that desctibes motors
lower_motor_limit_ = [-470.0 -110.0 -80.0 -70.0 -80.0 -1000.0];
upper_motor_limit_ = [450.0 100.0 100.0 380.0 490.0 3000.0];
synchro_motor_position_ = [-15.9 -5.0 -8.527 151.31 432.25 791.0];
%enc_res = [4000.0 4000.0 4000.0 4000.0 4000.0 2000.0];

M_PI = 3.14159265358979323846;
M_PI_2 = M_PI/2;
GEAR = [-158.0 2*M_PI/5.0 2*M_PI/5.0 -128.0 -128.0*0.6 288.8845 ];
THETA = [0.0 220.3374 183.8348 1.570796 0.0, 0.0];

SYNCHRO_JOINT_POSITION(1) = synchro_motor_position_(1) - GEAR(1) * THETA(1);
SYNCHRO_JOINT_POSITION(2) = synchro_motor_position_(2) - GEAR(2) * THETA(2);
SYNCHRO_JOINT_POSITION(3) = synchro_motor_position_(3) - GEAR(3) * THETA(3);
SYNCHRO_JOINT_POSITION(4) = synchro_motor_position_(4) - GEAR(4) * THETA(4);
SYNCHRO_JOINT_POSITION(5) = synchro_motor_position_(5) - GEAR(5) * THETA(5)...
    - synchro_motor_position_(4);
SYNCHRO_JOINT_POSITION(6) = synchro_motor_position_(6) - GEAR(6) * THETA(6);

joint_3_revolution = M_PI;
axis_4_revolution = 2 * M_PI;

sl123 = 77895.25;
mi1 = 60902.55;
ni1 = -29346.68;
mi2 = -44100.00;
ni2 = -51240.00;

motors = zeros(1,6);

%% motor 0
motors(1,1) = GEAR(1) * joints(1,1) + SYNCHRO_JOINT_POSITION(1);

%% motor 1
motors(1,2) = GEAR(2) * sqrt(sl123 + mi1 * cos(joints(1,2))...
    + ni1 * sin(-joints(1,2))) + SYNCHRO_JOINT_POSITION(2);

%% motor 2
motors(1,3) = GEAR(3) * sqrt(sl123 + mi2 * cos(joints(1,3) +...
    joints(1,2) + M_PI_2) + ni2 *...
    sin(-(joints(1,3) + joints(1,2) + M_PI_2)))+ SYNCHRO_JOINT_POSITION(3);

%% motor 3
joints_tmp3 = joints(1,4) + joints(1,3) + joints(1,2) + M_PI_2;
if joints_tmp3 < -M_PI_2
    joints_tmp3 = joints_tmp3 + joint_3_revolution;
end
motors(1,4) = GEAR(4) * (joints_tmp3 + THETA(4))...
    + SYNCHRO_JOINT_POSITION(4);

%% motor 4

motors(1,5) = GEAR(5) * joints(1,5)...
    + SYNCHRO_JOINT_POSITION(5) + motors(1,4);

while motors(1,5) < lower_motor_limit_(5)
    motors(1,5) = motors(1,5) + axis_4_revolution;
end
while motors(1,5) > upper_motor_limit_(5)
    motors(1,5) = motors(1,5) - axis_4_revolution;
end

%% motor 5
motors(1,6) = GEAR(6) * joints(1,6) + SYNCHRO_JOINT_POSITION(6);

end

