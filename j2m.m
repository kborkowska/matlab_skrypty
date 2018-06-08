%% calculates motor position from the one given in joint positions
joints = [0 0 0 0 0 0];

% stuff that desctibes motors
lower_limits = [-470.0, -110.0, -80.0, -70.0, -80.0, -1000.0];
upper_limits = [450.0, 100.0, 100.0, 380.0, 490.0, 3000.0];
synchro_motor_position = [-15.9, -5.0, -8.527, 151.31, 432.25, 791.0];
enc_res = [4000.0, 4000.0, 4000.0, 4000.0, 4000.0, 2000.0];

M_PI = 3.14159;
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

%% function calculatin motor positions
function [motors] = 

motors = [0 0 0 0 0 0];

%% motor 1
motors(1) = GEAR(1) * joints(1) + SYNCHRO_JOINT_POSITION(1);

%% motor 2
motors(2) = GEAR(2) * sqrt(sl123 + mi1 * cos(joints(2))...
            + ni1 * sin(-joints(2))) + SYNCHRO_JOINT_POSITION(2);
        
%% motor 3
motors(3) = GEAR(3) * sqrt(sl123 + mi2 * cos(joints(3) +...
                            joints(2) + M_PI) + ni2 *...
  sin(-(joints(3) + joints(2) + M_PI)))+ SYNCHRO_JOINT_POSITION(3);

%% motor 4
joints_tmp3 = joints(4) + joints(3) + joints(2) + M_PI;
if joints_tmp3 < -M_PI
  joints_tmp3 = joints_tmp3 + joint_3_revolution;
end
motors(4) = GEAR(4) * (joints_tmp3 + THETA(4))...
                        + SYNCHRO_JOINT_POSITION(4);
                    
%% motor 5

motors(5) = GEAR(5) * joints(5)...
                + SYNCHRO_JOINT_POSITION(5) + motors(4);

while motors(5) < lower_motor_limit_(5)
  motors(5) = motors(5) + axis_4_revolution;
end
while motors(5) > upper_motor_limit_(5)
  motors(5) = motors(5) - axis_4_revolution;
end

%% motor 6
motors(6) = GEAR(6) * joints(6) + SYNCHRO_JOINT_POSITION(6);



