#include <stdio.h>
#include <string.h>
#include <breeze_communication_link.h>

CommunicationLink::CommunicationLink(unsigned char owner_id,
                                     unsigned char other_id,
                                     CommunicationDataType *comm_data_type)
{
    comm_mode_      = MODE_MASTER;
    port_num_       = 1;
    owner_id_       = owner_id;
    other_id_       = other_id;
    link_ack_en_    = FALSE;
    comm_data_type_ = comm_data_type;

    if (comm_mode_ == MODE_SLAVE) {
        link_ack_en_ = TRUE;
    }

    shake_hands_state_             = FALSE;
    comm_receive_state_            = WAITING_FF_A;
    comm_command_state_            = SHAKE_HANDS;
    comm_message_recv_.sender_id   = 0;
    comm_message_recv_.receiver_id = 0;
    comm_message_recv_.length      = 0;
    comm_message_send_.sender_id   = 0;
    comm_message_send_.receiver_id = 0;
    comm_message_send_.length      = 0;
    recv_package_count_            = 0;
    package_update_freq_           = 0;
    send_package_count_            = 0;
    send_buffer_[0]                = 0;
    send_buffer_length_            = 0;
}

unsigned char CommunicationLink::sendCommandFromMaster(
    CommunicationCommandState command_state)
{
    unsigned char  analysis_state = TRUE;
    unsigned char *single_command;

    if (comm_mode_ != MODE_MASTER) {
        return FALSE;
    }

    switch (command_state) {
        case READ_GLOBAL_COORDINATE: {
            READ_COMMAND_STATE;
        }
        case READ_GLOBAL_COORD_SPEED: {
            READ_COMMAND_STATE;
        }
        case READ_ROBOT_COORDINATE: {
            READ_COMMAND_STATE;
        }
        case READ_ROBOT_COORD_SPEED: {
            READ_COMMAND_STATE;
        }
        case READ_ROBOT_IMU: {
            READ_COMMAND_STATE;
        }
        case READ_MOTOR_SPEED: {
            READ_COMMAND_STATE;
        }
        case READ_MOTOR_MILEAGE: {
            READ_COMMAND_STATE;
        }
        case READ_ROBOT_HEIGHT: {
            READ_COMMAND_STATE;
        }
        case READ_MOTOR_THRUST: {
            READ_COMMAND_STATE;
        }
        case READ_ROBOT_SPACE_POSE: {
            READ_COMMAND_STATE;
        }
        case READ_ROBOT_SYSTEM_INFO: {
            READ_COMMAND_STATE;
        }
        case SHAKE_HANDS: {
            shake_hands_state_ = TRUE;
            WRITE_COMMAND_STATE(comm_data_type_->global_coordinate_actual_);
        }
        case WRITE_GLOBAL_COORD_SPEED: {
            WRITE_COMMAND_STATE(comm_data_type_->global_coord_speed_target_);
        }
        case WRITE_ROBOT_COORD_SPEED: {
            WRITE_COMMAND_STATE(comm_data_type_->robot_coord_speed_target_);
        }
        case WRITE_MOTOR_SPEED: {
            WRITE_COMMAND_STATE(comm_data_type_->motor_speed_target_);
        }
        case WRITE_ROBOT_IMU: {
            WRITE_COMMAND_STATE(comm_data_type_->robot_imu_target_);
        }
        case WRITE_ROBOT_HEIGHT: {
            WRITE_COMMAND_STATE(comm_data_type_->robot_height_target_);
        }
        case WRITE_MOTOR_THRUST: {
            WRITE_COMMAND_STATE(comm_data_type_->motor_thrust_target_);
        }
        case WRITE_ROBOT_SPACE_POSE: {
            WRITE_COMMAND_STATE(comm_data_type_->robot_space_pose_target_);
        }
        default: {
            analysis_state = FALSE;
            break;
        }

        return analysis_state;
    }
}

unsigned char CommunicationLink::analyseReceiveByte(unsigned char recv_byte)
{
    // unsigned char
}

unsigned char CommunicationLink::analyseReceiveStates(unsigned char recv_data)
{
    switch (comm_receive_state_) {
        case WAITING_FF_A: {
            if (recv_data == 0xff) {
                comm_receive_state_ = WAITING_FF_B;
                recv_checksum_      = 0;
                recv_message_length_ = 0;
                recv_byte_count_ = 0;
                recv_checksum_ += recv_data;
            }
            break;
        }
        case WAITING_FF_B: {
            if (recv_data == 0xff) {
                comm_receive_state_ = GET_SENDER_ID;
                recv_checksum_ += recv_data;
            }
            else {
                comm_receive_state_ = WAITING_FF_A;
            }
            break;
        }
        case GET_SENDER_ID: {
            comm_message_recv_.sender_id = recv_data;
            if (comm_message_recv_.sender_id == other_id_) {
                recv_checksum_ += recv_data;
                comm_receive_state_ = GET_RECEIVER_ID;
            }
            else {
                printf("Error, the sender id is not exist!\n");
                comm_receive_state_ = WAITING_FF_A;
            }
            break;
        }
        case GET_RECEIVER_ID: {
            comm_message_recv_.receiver_id = recv_data;
            if (comm_message_recv_.receiver_id == owner_id_) {
                recv_checksum_ += recv_data;
                comm_receive_state_ = RECEIVE_LENGTH_H;
            }
            else {
                printf("Error, the receiver id is not exist!\n");
                comm_receive_state_ = WAITING_FF_A;
            }
            break;
        }
        case RECEIVE_LENGTH_H: {
            recv_checksum_ += recv_data;
            recv_message_length_ |= recv_data << 8;
            comm_receive_state_ = RECEIVE_LENGTH_L;
            break;
        }
        case RECEIVE_LENGTH_L: {
            recv_checksum_ += recv_data;
            recv_message_length_ |= recv_data;
            comm_message_recv_.length = recv_message_length_;
            comm_receive_state_ = RECEIVE_PACKAGE;
            break;
        }
        case RECEIVE_PACKAGE: {
            recv_checksum_ += recv_data;
            comm_message_recv_.data[recv_byte_count_++] = recv_data;
            if (recv_byte_count_ >= recv_message_length_) {
                comm_receive_state_ = RECEIVE_CHECKSUM;
                recv_checksum_ = recv_checksum_ % 255;
            }
            break;
        }
        case RECEIVE_CHECKSUM: {
            if (recv_data == (unsigned char)recv_checksum_) {
                recv_checksum_ = 0;
                comm_receive_state_ = WAITING_FF_A;
                printf("Receive a package!\n");
                return TRUE;
            }
            else {
                printf("Checksum is error!\n");
                comm_receive_state_ = WAITING_FF_A;
            }
            break;
        }
        default: {
            comm_receive_state_ = WAITING_FF_A;
        }
    }

    return FALSE;
}

unsigned char CommunicationLink::analyseReceivePackage(void)
{
    unsigned char  analysis_state = FALSE;
    unsigned char *single_command;

    comm_command_state_ = (CommunicationCommandState)comm_message_recv_.data[0];

    if (comm_mode_ == MODE_SLAVE) {
        if (!shake_hands_state_ && comm_command_state_ != SHAKE_HANDS) {
            sendData(SHAKE_HANDS, (unsigned char *)single_command, 0);
            return TRUE;
        }
    }

    switch (comm_command_state_) {
        case SHAKE_HANDS: {
            analysis_state = analyseWriteCommand(
                comm_command_state_,
                (unsigned char *)&comm_data_type_->global_coordinate_actual_,
                sizeof(comm_data_type_->global_coordinate_actual_));
            break;
        }
        case READ_GLOBAL_COORDINATE: {
            analysis_state = analyseReadCommand(
                comm_command_state_,
                (unsigned char *)&comm_data_type_->global_coordinate_actual_,
                sizeof(comm_data_type_->global_coordinate_actual_));
            break;
        }
        case READ_GLOBAL_COORD_SPEED: {
            analysis_state = analyseReadCommand(
                comm_command_state_,
                (unsigned char *)&comm_data_type_->global_coord_speed_actual_,
                sizeof(comm_data_type_->global_coord_speed_actual_));
            break;
        }
        case READ_ROBOT_COORDINATE: {
            analysis_state = analyseReadCommand(
                comm_command_state_,
                (unsigned char *)&comm_data_type_->robot_coordinate_actual_,
                sizeof(comm_data_type_->robot_coordinate_actual_));
            break;
        }
        case READ_ROBOT_COORD_SPEED: {
            analysis_state = analyseReadCommand(
                comm_command_state_,
                (unsigned char *)&comm_data_type_->robot_coord_speed_actual_,
                sizeof(comm_data_type_->robot_coord_speed_actual_));
            break;
        }
        case READ_ROBOT_IMU: {
            analysis_state = analyseReadCommand(
                comm_command_state_,
                (unsigned char *)&comm_data_type_->robot_imu_actual_,
                sizeof(comm_data_type_->robot_imu_actual_));
            break;
        }
        case READ_MOTOR_SPEED: {
            analysis_state = analyseReadCommand(
                comm_command_state_,
                (unsigned char *)&comm_data_type_->motor_speed_actual_,
                sizeof(comm_data_type_->motor_speed_actual_));
            break;
        }
        case READ_MOTOR_MILEAGE: {
            analysis_state = analyseReadCommand(
                comm_command_state_,
                (unsigned char *)&comm_data_type_->motor_mileage_actual_,
                sizeof(comm_data_type_->motor_mileage_actual_));
            break;
        }
        case READ_ROBOT_HEIGHT: {
            analysis_state = analyseReadCommand(
                comm_command_state_,
                (unsigned char *)&comm_data_type_->robot_height_actual_,
                sizeof(comm_data_type_->robot_height_actual_));
            break;
        }
        case READ_MOTOR_THRUST: {
            analysis_state = analyseReadCommand(
                comm_command_state_,
                (unsigned char *)&comm_data_type_->motor_thrust_actual_,
                sizeof(comm_data_type_->motor_thrust_actual_));
            break;
        }
        case READ_ROBOT_SPACE_POSE: {
            analysis_state = analyseReadCommand(
                comm_command_state_,
                (unsigned char *)&comm_data_type_->robot_space_pose_actual_,
                sizeof(comm_data_type_->robot_space_pose_actual_));
            break;
        }
        case READ_ROBOT_SYSTEM_INFO: {
            analysis_state = analyseReadCommand(
                comm_command_state_,
                (unsigned char *)&comm_data_type_->robot_system_info_actual_,
                sizeof(comm_data_type_->robot_system_info_actual_));
            break;
        }
        case WRITE_GLOBAL_COORD_SPEED: {
            analysis_state = analyseWriteCommand(
                comm_command_state_,
                (unsigned char *)&comm_data_type_->global_coord_speed_target_,
                sizeof(comm_data_type_->global_coord_speed_target_));
            break;
        }
        case WRITE_ROBOT_COORD_SPEED: {
            analysis_state = analyseWriteCommand(
                comm_command_state_,
                (unsigned char *)&comm_data_type_->robot_coord_speed_target_,
                sizeof(comm_data_type_->robot_coord_speed_target_));
            break;
        }
        case WRITE_MOTOR_SPEED: {
            analysis_state = analyseWriteCommand(
                comm_command_state_,
                (unsigned char *)&comm_data_type_->motor_speed_target_,
                sizeof(comm_data_type_->motor_speed_target_));
            break;
        }
        case WRITE_ROBOT_IMU: {
            analysis_state = analyseWriteCommand(
                comm_command_state_,
                (unsigned char *)&comm_data_type_->robot_imu_target_,
                sizeof(comm_data_type_->robot_imu_target_));
            break;
        }
        case WRITE_ROBOT_HEIGHT: {
            analysis_state = analyseWriteCommand(
                comm_command_state_,
                (unsigned char *)&comm_data_type_->robot_height_target_,
                sizeof(comm_data_type_->robot_height_target_));
            break;
        }
        case WRITE_MOTOR_THRUST: {
            analysis_state = analyseWriteCommand(
                comm_command_state_,
                (unsigned char *)&comm_data_type_->motor_thrust_target_,
                sizeof(comm_data_type_->motor_thrust_target_));
            break;
        }
        case WRITE_ROBOT_SPACE_POSE: {
            analysis_state = analyseWriteCommand(
                comm_command_state_,
                (unsigned char *)&comm_data_type_->robot_space_pose_target_,
                sizeof(comm_data_type_->robot_space_pose_target_));
            break;
        }
        default: {
            analysis_state = FALSE;
            break;
        }
    }

    comm_message_recv_.sender_id   = 0;
    comm_message_recv_.receiver_id = 0;
    comm_message_recv_.length      = 0;
    comm_message_recv_.data[0]     = 0;

    return analysis_state;
}

unsigned char CommunicationLink::analyseReadCommand(
    CommunicationCommandState comm_command_state,
    unsigned char *comm_data_type,
    unsigned short comm_data_type_length)
{
    if (comm_mode_ == MODE_MASTER) {
        if ((comm_message_recv_.length - 1) != comm_data_type_length) {
            printf("Error, the master can not read message from slave!");
            return FALSE;
        }
        memcpy(comm_data_type, &comm_message_recv_.data[1],
               comm_data_type_length);
        recv_package_state_[(unsigned char)comm_command_state] = TRUE;
    }
    else if (comm_mode_ == MODE_SLAVE) {
        sendData(comm_command_state, comm_data_type, comm_data_type_length);
        recv_package_state_[(unsigned char)comm_command_state] = TRUE;
    }

    return TRUE;
}

unsigned char CommunicationLink::analyseWriteCommand(
    CommunicationCommandState comm_command_state,
    unsigned char *comm_data_type,
    unsigned short comm_data_type_length)
{
    unsigned char *ack;

    if (comm_mode_ = MODE_MASTER) {
        if (comm_command_state == SHAKE_HANDS) {
            shake_hands_state_ = TRUE;
            printf("The slave is waiting master send data!\n");
        }
        else {
            printf("Receive a ack!\n");
        }
        recv_package_state_[(unsigned char)comm_command_state] = TRUE;
    }
    else if (comm_mode_ = MODE_SLAVE) {
        if ((comm_message_recv_.length - 1) != comm_data_type_length) {
            printf("Error, the slave can not read message from master!\n");
            return FALSE;
        }
        memcpy(comm_data_type, &comm_message_recv_.data[1],
                comm_data_type_length);
        if (comm_command_state == SHAKE_HANDS) {
            shake_hands_state_ = TRUE;
        }
        else {
            sendData(comm_command_state, (unsigned char *)ack, 0);
        }
        recv_package_state_[(unsigned char)comm_command_state] = TRUE;
    }

    return TRUE;
}

void CommunicationLink::sendData(CommunicationCommandState command_state,
                                 unsigned char *comm_data_type,
                                 unsigned short comm_data_type_length)
{
    comm_message_send_.sender_id   = owner_id_;
    comm_message_send_.receiver_id = other_id_;
    comm_message_send_.length      = comm_data_type_length + 1;
    comm_message_send_.data[0]     = (unsigned char)command_state;

    if (comm_data_type_length > 0) {
        memcpy(&comm_message_send_.data[1], comm_data_type,
                comm_data_type_length);
    }

    sendMessage();
}

void CommunicationLink::sendMessage(void)
{
    unsigned short int i;
    unsigned short int checksum = 0;

    send_buffer_[0] = 0xff;
    checksum += 0xff;

    send_buffer_[1] = 0xff;
    checksum += 0xff;

    send_buffer_[2] = comm_message_send_.sender_id;
    checksum += send_buffer_[2];

    send_buffer_[3] = comm_message_send_.receiver_id;
    checksum += send_buffer_[3];

    send_buffer_[4] = (unsigned char)(comm_message_send_.length >> 8);
    checksum += send_buffer_[4];

    send_buffer_[5] = (unsigned char)(comm_message_send_.length);
    checksum += send_buffer_[5];

    for (i = 0; i < comm_message_send_.length; i++) {
        send_buffer_[6 + i] = comm_message_send_.data[i];
        checksum += send_buffer_[6 + i];
    }

    checksum = checksum % 255;
    send_buffer_[6 + i] = checksum;
    send_buffer_length_ = 7 + i;
    send_package_count_++;
}

unsigned char CommunicationLink::getReceiveState(
    CommunicationCommandState command_state)
{
    return recv_package_state_[command_state];
}

unsigned char *CommunicationLink::getSerializeData(void)
{
    return send_buffer_;
}

unsigned short int CommunicationLink::getSerializedLength(void)
{
    return send_buffer_length_;
}

void CommunicationLink::setOwnerID(unsigned char owner_id)
{
    owner_id_ = owner_id;
}

void CommunicationLink::setOtherID(unsigned char other_id)
{
    other_id_ = other_id;
}

void CommunicationLink::setPortNum(unsigned char port_num)
{
    port_num_ = port_num;
}

void CommunicationLink::enableAck(void)
{
    if (!link_ack_en_) {
        link_ack_en_ = TRUE;
    }
}

void CommunicationLink::disableAck(void)
{
    link_ack_en_ = FALSE;
}

int main(void)
{
    return 0;
}
