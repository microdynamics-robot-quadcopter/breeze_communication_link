#include <breeze_communication_data_type.h>

CommunicationDataType::CommunicationDataType(void)
{
    global_coord_speed_target_.axis_x = 0.0;
    global_coord_speed_target_.axis_y = 0.0;
    global_coord_speed_target_.axis_z = 0.0;

    global_coord_speed_actual_.axis_x = 0.0;
    global_coord_speed_actual_.axis_y = 0.0;
    global_coord_speed_actual_.axis_z = 0.0;

    global_coordinate_actual_.axis_x = 0.0;
    global_coordinate_actual_.axis_y = 0.0;
    global_coordinate_actual_.axis_z = 0.0;

    robot_coord_speed_target_.axis_x = 0.0;
    robot_coord_speed_target_.axis_y = 0.0;
    robot_coord_speed_target_.axis_z = 0.0;

    robot_coord_speed_actual_.axis_x = 0.0;
    robot_coord_speed_actual_.axis_y = 0.0;
    robot_coord_speed_actual_.axis_z = 0.0;

    robot_coordinate_actual_.axis_x = 0.0;
    robot_coordinate_actual_.axis_y = 0.0;
    robot_coordinate_actual_.axis_z = 0.0;

    motor_speed_target_.motor_a = 0.0;
    motor_speed_target_.motor_b = 0.0;
    motor_speed_target_.motor_c = 0.0;
    motor_speed_target_.motor_d = 0.0;

    motor_speed_actual_.motor_a = 0.0;
    motor_speed_actual_.motor_b = 0.0;
    motor_speed_actual_.motor_c = 0.0;
    motor_speed_actual_.motor_d = 0.0;

    motor_mileage_actual_.motor_a = 0.0;
    motor_mileage_actual_.motor_b = 0.0;
    motor_mileage_actual_.motor_c = 0.0;
    motor_mileage_actual_.motor_d = 0.0;

    robot_imu_target_.acc.acc_x = 0.0;
    robot_imu_target_.acc.acc_y = 0.0;
    robot_imu_target_.acc.acc_z = 0.0;
    robot_imu_target_.att.att_r = 0.0;
    robot_imu_target_.att.att_p = 0.0;
    robot_imu_target_.att.att_y = 0.0;

    robot_imu_actual_.acc.acc_x = 0.0;
    robot_imu_actual_.acc.acc_y = 0.0;
    robot_imu_actual_.acc.acc_z = 0.0;
    robot_imu_actual_.att.att_r = 0.0;
    robot_imu_actual_.att.att_p = 0.0;
    robot_imu_actual_.att.att_y = 0.0;

    robot_height_target_.alt = 0.0;
    robot_height_target_.hei = 0.0;

    robot_height_actual_.alt = 0.0;
    robot_height_actual_.hei = 0.0;

    motor_thrust_target_.thrust = 0.0;
    motor_thrust_actual_.thrust = 0.0;

    robot_system_info_actual_.battery_capacity = 0.0;
    robot_system_info_actual_.link_quality     = 0.0;

    robot_space_pose_target_.coo.axis_x = 0.0;
    robot_space_pose_target_.coo.axis_y = 0.0;
    robot_space_pose_target_.coo.axis_z = 0.0;
    robot_space_pose_target_.att.att_r  = 0.0;
    robot_space_pose_target_.att.att_p  = 0.0;
    robot_space_pose_target_.att.att_y  = 0.0;

    robot_space_pose_actual_.coo.axis_x = 0.0;
    robot_space_pose_actual_.coo.axis_y = 0.0;
    robot_space_pose_actual_.coo.axis_z = 0.0;
    robot_space_pose_actual_.att.att_r  = 0.0;
    robot_space_pose_actual_.att.att_p  = 0.0;
    robot_space_pose_actual_.att.att_y  = 0.0;
}
