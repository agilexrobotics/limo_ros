Header header

int8 MOTOR_ID_FRONT_RIGHT = 0
int8 MOTOR_ID_FRONT_LEFT = 1
int8 MOTOR_ID_REAR_RIGHT = 2
int8 MOTOR_ID_REAR_LEFT = 3

# motion state
float64 linear_velocity
float64 angular_velocity
float64 lateral_velocity
float64 steering_angle
float64 x_linear_vel
float64 y_linear_vel
float64 motion_radius
uint8 current_motion_mode

# system base state
uint8 vehicle_state
uint8 control_mode
uint16 error_code
float64 battery_voltage

# 