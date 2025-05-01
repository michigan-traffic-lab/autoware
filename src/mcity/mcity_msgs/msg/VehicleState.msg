float64 timestamp
 
float64 x
float64 y
float64 z

float64 speed_x
float64 speed_y
float64 speed_z

float64 acc_x
float64 acc_y
float64 acc_z

float64 heading
float64 yaw_rate

bool 	by_wire_enabled

float64 throttle_cmd
float64 throttle_input
float64 throttle_state
bool    throttle_enabled
bool    throttle_override
bool    throttle_driver
bool    throttle_timeout

float64 brake_cmd
float64 brake_input
float64 brake_state
float64 brake_torq_cmd
float64 brake_torq_input
float64 brake_torq_state
bool    brake_boo_output
bool    brake_enabled
bool    brake_override
bool    brake_driver
bool    brake_timeout

float64 steer_cmd
float64 steer_state
float64 steer_torque
bool    steer_enabled
bool    steer_override
bool    steer_timeout

int16   gear_pos

float64 engine_rpm

float64 wheel_v_front_left
float64 wheel_v_front_right
float64 wheel_v_rear_left
float64 wheel_v_rear_right