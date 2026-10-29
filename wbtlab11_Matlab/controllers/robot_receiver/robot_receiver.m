% =========================================================================
% OPCION A — robot_receiver.m  (corregido lib.pointer)
% =========================================================================
function robot_receiver
TIME_STEP = 64;

right_motor = wb_robot_get_device('motor_1');
left_motor  = wb_robot_get_device('motor_2');
receiver    = wb_robot_get_device('receiver');

wb_motor_set_position(left_motor,  inf);
wb_motor_set_position(right_motor, inf);
wb_motor_set_velocity(left_motor,  0.0);
wb_motor_set_velocity(right_motor, 0.0);
wb_receiver_enable(receiver, TIME_STEP);

rpm2rads = @(x) x * (2*pi / 60);

while wb_robot_step(TIME_STEP) ~= -1

    v_left  = 0;
    v_right = 0;

    while wb_receiver_get_queue_length(receiver) > 0
        % wb_receiver_get_data_as_double es la funcion correcta en
        % Webots R2025a + MATLAB: devuelve directamente un array double
        vals    = wb_receiver_get_data_as_double(receiver);
        v_left  = vals(1);
        v_right = vals(2);
        wb_receiver_next_packet(receiver);
    end

    wb_motor_set_velocity(left_motor,  -rpm2rads(v_left));
    wb_motor_set_velocity(right_motor, -rpm2rads(v_right));
end
