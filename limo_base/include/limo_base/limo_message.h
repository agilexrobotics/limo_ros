/**
 * @Kit       : Qt-Creator: Desktop
 * @Author    : Wang Zhe
 * @Date      : 2021-09-06  14:44:48
 * @FileName  : limo_message.h
 * @Mail      : zhe.wang@agilex.ai
 * Copyright  : AgileX Robotics (2021)
 **/

#ifndef LIMO_MESSAGE_H
#define LIMO_MESSAGE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include <stdint.h>

#define LIMO_SYNC1	    0x55
#define LIMO_SYNC2	    0x0E

// limo protocol data format
typedef struct {
  uint8_t state{0};
  uint8_t type;
  uint16_t can_id;
  uint8_t count;
  uint8_t data[8];
  uint16_t length;
  uint8_t RxCK;  // Rx check
  uint8_t hight;
  uint8_t low;
} LIMO_t_RAW_t;

enum {
  LIMO_WAIT_SYNC1 = 0,
  LIMO_WAIT_SYNC2,
  LIMO_WAIT_ID_HIGH,
  LIMO_WAIT_ID_LOW,
  LIMO_WAIT_DATA,
  LIMO_COUNT,
  LIMO_CHECK,
};

/***************** Control messages *****************/

// 0x111
typedef struct {
  float linear_velocity;
  float angular_velocity;
  float lateral_velocity;
  float steering_angle;
} MotionCommandMessage;

// 0x121
typedef enum {
  CONST_OFF = 0x00,
  CONST_ON = 0x01,
  BREATH = 0x02,
  CUSTOM = 0x03
} LightMode;

typedef struct {
  LightMode mode;
  uint8_t custom_value;
} LightOperation;

typedef struct {
  bool cmd_ctrl_allowed;
  LightOperation front_light;
  LightOperation rear_light;
} LightCommandMessage;

//0x211
typedef enum{
  MODE_FOUR_DIFF=0x00,
  MODE_ACKERMANN=0x01,
  MODE_UNKNOWN=0xff,
} MotionMode;

// 0x131
typedef struct {
  bool enable_braking;
} BrakingCommandMessage;

// 0x141
typedef struct {
  uint8_t motion_mode;
} MotionModeMessage;

/**************** Feedback messages *****************/

// 0x211
typedef enum {
  VehicleStateNormal = 0x00,
  VehicleStateEStop = 0x01,
  VehicleStateException = 0x02
} VehicleState;

typedef enum {
  //   CONTROL_MODE_STANDBY = 0x00,
  CONTROL_MODE_RC = 0x00,
  CONTROL_MODE_CAN = 0x01,
  CONTROL_MODE_UART = 0x02
} ControlMode;

#define SYSTEM_ERROR_MOTOR_DRIVER_MASK ((uint16_t)0x0100)
#define SYSTEM_ERROR_HL_COMM_MASK ((uint16_t)0x0200)
#define SYSTEM_ERROR_BATTERY_FAULT_MASK ((uint16_t)0x0001)
#define SYSTEM_ERROR_BATTERY_WARNING_MASK ((uint16_t)0x0002)
#define SYSTEM_ERROR_RC_SIGNAL_LOSS_MASK ((uint16_t)0x0004)
#define SYSTEM_ERROR_MOTOR1_COMM_MASK ((uint16_t)0x0008)
#define SYSTEM_ERROR_MOTOR2_COMM_MASK ((uint16_t)0x0010)
#define SYSTEM_ERROR_MOTOR3_COMM_MASK ((uint16_t)0x0020)
#define SYSTEM_ERROR_MOTOR4_COMM_MASK ((uint16_t)0x0040)
#define SYSTEM_ERROR_STEER_ENCODER_MASK ((uint16_t)0x0080)

// 0x211
typedef struct {
  VehicleState vehicle_state;
  ControlMode control_mode;
  float battery_voltage;
  uint16_t error_code;
  MotionMode motion_mode;
} SystemStateMessage;

// 0x221
typedef struct {
  float linear_velocity;
  float angular_velocity;  // only valid for differential drivering
  float lateral_velocity;
  float steering_angle;  // only valid for ackermann steering
} MotionStateMessage;

// 0x231
typedef LightCommandMessage LightStateMessage;

// 0x241
typedef enum {
  RC_SWITCH_UP = 0,
  RC_SWITCH_MIDDLE,
  RC_SWITCH_DOWN
} RcSwitchState;

typedef struct {
  RcSwitchState swa;
  RcSwitchState swb;
  RcSwitchState swc;
  RcSwitchState swd;
  int8_t stick_right_v;
  int8_t stick_right_h;
  int8_t stick_left_v;
  int8_t stick_left_h;
  int8_t var_a;
} RcStateMessage;

// 0x251 - 0x258
typedef struct {
  uint8_t motor_id;
  int16_t rpm;
  float current;
  int32_t pulse_count;
} ActuatorHSStateMessage;

// 0x261 - 0x264
#define DRIVER_STATE_INPUT_VOLTAGE_LOW_MASK ((uint8_t)0x01)
#define DRIVER_STATE_MOTOR_OVERHEAT_MASK ((uint8_t)0x02)
#define DRIVER_STATE_DRIVER_OVERLOAD_MASK ((uint8_t)0x04)
#define DRIVER_STATE_DRIVER_OVERHEAT_MASK ((uint8_t)0x08)
#define DRIVER_STATE_SENSOR_FAULT_MASK ((uint8_t)0x10)
#define DRIVER_STATE_DRIVER_FAULT_MASK ((uint8_t)0x20)
#define DRIVER_STATE_DRIVER_ENABLED_MASK ((uint8_t)0x40)
#define DRIVER_STATE_DRIVER_RESET_MASK ((uint8_t)0x80)

// 0x291
typedef struct {
  uint8_t motion_mode;
  uint8_t mode_changing;
} MotionModeFeedbackMessage;

typedef struct {
  uint8_t motor_id;
  float driver_voltage;
  float driver_temperature;
  int8_t motor_temperature;
  uint8_t driver_state;
} ActuatorLSStateMessage;

/***************** Sensor messages ******************/

// 0x311
typedef struct {
  float left_wheel;
  float right_wheel;
} OdometryMessage;

// 0x321
typedef struct {
  float accel_x;
  float accel_y;
  float accel_z;
} ImuAccelMessage;

// 0x322
typedef struct {
  float gyro_x;
  float gyro_y;
  float gyro_z;
} ImuGyroMessage;

// 0x323
typedef struct {
  float yaw;
  float pitch;
  float roll;
} ImuEulerMessage;

// 0x331
typedef struct {
  uint8_t trigger_state;
} SafetyBumperMessage;

// 0x340 + num
typedef struct {
  uint8_t sensor_id;
  uint8_t distance[8];
} UltrasonicMessage;

// 0x350 + num
typedef struct {
  uint8_t sensor_id;
  float relative_distance;
  float relative_angle;
  bool is_normal;
  int8_t channels[3];
} UwbMessage;

// 0x361
typedef struct {
  uint8_t battery_soc;
  uint8_t battery_soh;
  float voltage;
  float current;
  float temperature;
} BmsBasicMessage;

// 0x362
#define BMS_PROT1_CHARGING_CURRENT_NONZERO_MASK ((uint8_t)0x01)
#define BMS_PROT1_CHARGING_OVERCURRENT_SET_MASK ((uint8_t)0x02)
#define BMS_PROT1_DISCHARGING_CURRENT_NONZERO_MASK ((uint8_t)0x10)
#define BMS_PROT1_DISCHARGING_OVERCURRENT_SET_MASK ((uint8_t)0x20)
#define BMS_PROT1_DISCHARGING_SHORTCIRCUIT_SET_MASK ((uint8_t)0x40)

#define BMS_PROT2_CORE_OPENCIRCUIT_SET_MASK ((uint8_t)0x01)
#define BMS_PROT2_TEMP_SENSOR_OPENCIRCUIT_SET_MASK ((uint8_t)0x02)
#define BMS_PROT2_CORE_OVERVOLTAGE_SET_MASK ((uint8_t)0x10)
#define BMS_PROT2_CORE_UNDERVOLTAGE_SET_MASK ((uint8_t)0x20)
#define BMS_PROT2_TOTAL_OVERVOLTAGE_SET_MASK ((uint8_t)0x40)
#define BMS_PROT2_TOTAL_UNDERVOLTAGE_SET_MASK ((uint8_t)0x80)

#define BMS_PROT3_CHARGING_OVERTEMP_SET_MASK ((uint8_t)0x04)
#define BMS_PROT3_DISCHARGING_OVERTEMP_SET_MASK ((uint8_t)0x08)
#define BMS_PROT3_CHARGING_UNDERTEMP_SET_MASK ((uint8_t)0x10)
#define BMS_PROT3_DISCHARGING_UNDERTEMP_SET_MASK ((uint8_t)0x20)
#define BMS_PROT3_CHARGING_TEMPDIFF_SET_MASK ((uint8_t)0x40)
#define BMS_PROT3_DISCHARGING_TEMPDIFF_SET_MASK ((uint8_t)0x80)

#define BMS_PROT4_CHARGING_MOS_STATE_SET_MASK ((uint8_t)0x01)
#define BMS_PROT4_DISCHARGING_MOS_STATE_SET_MASK ((uint8_t)0x02)
#define BMS_PROT4_CHARGING_MOS_FAILURE_SET_MASK ((uint8_t)0x04)
#define BMS_PROT4_DISCHARGING_MOS_FAILURE_SET_MASK ((uint8_t)0x08)
#define BMS_PROT4_WEAK_SIGNAL_SWITCH_OPEN_SET_MASK ((uint8_t)0x10)

typedef struct {
  uint8_t protection_code1;
  uint8_t protection_code2;
  uint8_t protection_code3;
  uint8_t protection_code4;
  uint8_t battery_max_teperature;
  uint8_t battery_min_teperature;
} BmsExtendedMessage;

/************  Query/config messages ****************/

// 0x411
typedef struct {
  bool request;
} VersionRequestMessage;

// 0x41a
typedef struct {
  uint16_t controller_hw_version;
  uint16_t motor_driver_hw_version;
  uint16_t controller_sw_version;
  uint16_t motor_driver_sw_version;
} VersionResponseMessage;

// 0x421
typedef struct {
  ControlMode mode;
} ControlModeConfigMessage;

// 0x431
typedef struct {
  bool set_as_neutral;
} SteerNeutralRequestMessage;

// 0x43a
typedef struct {
  bool neutral_set_successful;
} SteerNeutralResponseMessage;

// 0x441
typedef enum {
  CLEAR_ALL_FAULT = 0x00,
  CLEAR_MOTOR1_FAULT = 0x01,
  CLEAR_MOTOR2_FAULT = 0x02,
  CLEAR_MOTOR3_FAULT = 0x03,
  CLEAR__MOTOR4_FAULT = 0x04
} FaultClearCode;

typedef struct {
  uint8_t error_clear_byte;
} StateResetConfigMessage;

//////////////////////////////////////////////////////

typedef enum {
  AgxMsgUnkonwn = 0x00,
  // command
  AgxMsgMotionCommand,
  AgxMsgLightCommand,
  AgxMsgBrakingCommand,
  AgxMsgSetMotionMode,
  // state feedback
  AgxMsgSystemState,
  AgxMsgMotionState,
  AgxMsgLightState,
  AgxMsgRcState,
  AgxMsgActuatorHSState,
  AgxMsgActuatorLSState,
  AgxMsgMotionModeState,
  // sensor
  AgxMsgOdometry,
  AgxMsgImuAccel,
  AgxMsgImuGyro,
  AgxMsgImuEuler,
  AgxMsgSafetyBumper,
  AgxMsgUltrasonic,
  AgxMsgUwb,
  AgxMsgBmsBasic,
  AgxMsgBmsExtended,
  // query/config
  AgxMsgVersionRequest,
  AgxMsgVersionResponse,
  AgxMsgControlModeConfig,
  AgxMsgSteerNeutralRequest,
  AgxMsgSteerNeutralResponse,
  AgxMsgStateResetConfig
} MsgType;

typedef struct {
  MsgType type;
  union {
    // command
    MotionCommandMessage motion_command_msg;
    LightCommandMessage light_command_msg;
    BrakingCommandMessage braking_command_msg;
    MotionModeMessage motion_mode_msg;
    // state feedback
    SystemStateMessage system_state_msg;
    MotionStateMessage motion_state_msg;
    LightStateMessage light_state_msg;
    RcStateMessage rc_state_msg;
    ActuatorHSStateMessage actuator_hs_state_msg;
    ActuatorLSStateMessage actuator_ls_state_msg;
    MotionModeFeedbackMessage motion_mode_feedback_msg;
    // sensor
    OdometryMessage odometry_msg;
    ImuAccelMessage imu_accel_msg;
    ImuGyroMessage imu_gyro_msg;
    ImuEulerMessage imu_euler_msg;
    SafetyBumperMessage safety_bumper_msg;
    UltrasonicMessage ultrasonic_msg;
    UwbMessage uwb_msg;
    BmsBasicMessage bms_basic_msg;
    BmsExtendedMessage bms_extended_msg;
    // query/config
    VersionRequestMessage version_request_msg;
    VersionResponseMessage version_response_msg;
    ControlModeConfigMessage control_mode_config_msg;
    SteerNeutralRequestMessage steer_neutral_request_msg;
    SteerNeutralResponseMessage steer_neutral_response_msg;
    StateResetConfigMessage state_reset_config_msg;
  } body;
} AgxMessage;


struct LimoState {
  // system state
  SystemStateMessage system_state;
  MotionStateMessage motion_state;
  LightStateMessage light_state;

  RcStateMessage rc_state;

  ActuatorHSStateMessage actuator_hs_state[8];
  ActuatorLSStateMessage actuator_ls_state[8];
  MotionModeFeedbackMessage current_motion_mode;

  // sensor data
  OdometryMessage odometry;

  // imu
  ImuAccelMessage imu_accel_;
  ImuGyroMessage imu_gyro_;
  ImuEulerMessage imu_euler_;
};
#ifdef __cplusplus
}
#endif

#endif /* LIMO_MESSAGE_H */
