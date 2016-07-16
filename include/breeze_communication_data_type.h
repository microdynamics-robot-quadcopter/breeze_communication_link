#ifndef BREEZE_COMMUNICATION_DATA_TYPE
#define BREEZE_COMMUNICATION_DATA_TYPE

typedef struct DataTypeAcceleration {
    float acc_x;
    float acc_y;
    float acc_z;
} DataTypeAcceleration;

typedef struct DataTypeCoordinate {
    float axis_x;
    float axis_y;
    float axis_z;
} DataTypeCoordinate;

typedef struct DataTypeAttitude {
    float att_r;
    float att_p;
    float att_y;
} DataTypeAttitude;

typedef struct DataTypeSpacePose {
    DataTypeCoordinate coo;
    DataTypeAttitude   att;
} DataTypeSpacePose;

typedef struct DataTypeIMU {
    DataTypeAcceleration acc;
    DataTypeAttitude     att;
} DataTypeIMU;

typedef struct DataTypeMotor {
    float motor_a;
    float motor_b;
    float motor_c;
    float motor_d;
} DataTypeMotor;

typedef struct DataTypeThrust {
    float thrust;
} DataTypeThrust;

typedef struct DataTypeHeight {
    float alt;
    float hei;
} DataTypeHeight;

typedef struct DataTypePID {
    float p;
    float i;
    float d;
} DataTypePID;

typedef struct DataTypeSystemInfo {
    float battery_capacity;
    float link_quality;
} DataTypeSystemInfo;

class CommunicationDataType
{
public:
    CommunicationDataType(void);
public:
    DataTypeCoordinate global_coord_speed_target_;
    DataTypeCoordinate global_coord_speed_actual_;
    DataTypeCoordinate global_coordinate_actual_;
    DataTypeCoordinate robot_coord_speed_target_;
    DataTypeCoordinate robot_coord_speed_actual_;
    DataTypeCoordinate robot_coordinate_actual_;
    DataTypeMotor      motor_speed_target_;
    DataTypeMotor      motor_speed_actual_;
    DataTypeMotor      motor_mileage_actual_;
    DataTypeIMU        robot_imu_target_;
    DataTypeIMU        robot_imu_actual_;
    DataTypeHeight     robot_height_target_;
    DataTypeHeight     robot_height_actual_;
    DataTypeThrust     motor_thrust_target_;
    DataTypeThrust     motor_thrust_actual_;
    DataTypeSystemInfo robot_system_info_actual_;
    DataTypeSpacePose  robot_space_pose_target_;
    DataTypeSpacePose  robot_space_pose_actual_;
};

#endif // BREEZE_COMMUNICATION_DATA_TYPE
