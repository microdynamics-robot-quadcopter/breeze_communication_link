#ifndef BREEZE_COMMUNICATION_LINK
#define BREEZE_COMMUNICATION_LINK

#include "breeze_communication_data_type.h"

#define MESSAGE_BUFFER_SIZE 100
#define TRUE                1
#define FALSE               0

#define READ_COMMAND_STATE                                         \
do {                                                               \
    recv_package_state_[(unsigned char)command_state] = FALSE;     \
    sendData(command_state, single_command, 0);                    \
    break;                                                         \
} while(FALSE)

#define WRITE_COMMAND_STATE(type)                                  \
do {                                                               \
    recv_package_state_[(unsigned char)command_state] = FALSE;     \
    sendData(command_state, (unsigned char *)&type, sizeof(type)); \
    break;                                                         \
} while(FALSE)

typedef enum CommunicationMode {
    MODE_MASTER,
    MODE_SLAVE
} CommunicationMode;

typedef enum CommunicationReceiveState {
    WAITING_FF_A,
    WAITING_FF_B,
    GET_SENDER_ID,
    GET_RECEIVER_ID,
    RECEIVE_LENGTH_H,
    RECEIVE_LENGTH_L,
    RECEIVE_PACKAGE,
    RECEIVE_CHECKSUM
} CommunicationReceiveState;

typedef enum CommunicationCommandState {
    SHAKE_HANDS,
    READ_GLOBAL_COORDINATE,
    READ_GLOBAL_COORD_SPEED,
    READ_ROBOT_COORDINATE,
    READ_ROBOT_COORD_SPEED,
    READ_ROBOT_IMU,
    READ_MOTOR_SPEED,
    READ_MOTOR_MILEAGE,
    READ_ROBOT_HEIGHT,
    READ_MOTOR_THRUST,
    READ_ROBOT_SPACE_POSE,
    READ_ROBOT_SYSTEM_INFO,
    WRITE_GLOBAL_COORD_SPEED,
    WRITE_ROBOT_COORD_SPEED,
    WRITE_MOTOR_SPEED,
    WRITE_ROBOT_IMU,
    WRITE_ROBOT_HEIGHT,
    WRITE_MOTOR_THRUST,
    WRITE_ROBOT_SPACE_POSE,
    LAST_COMMAND
} CommunicationCommandState;

typedef struct CommunicationMessage {
    unsigned char      sender_id;
    unsigned char      receiver_id;
    unsigned short int length;
    unsigned char      data[MESSAGE_BUFFER_SIZE];
} CommunicationMessage;

class CommunicationLink
{
public:
    CommunicationLink(unsigned char owner_id = 0x11,
                      unsigned char other_id = 0x01,
                      CommunicationDataType *comm_data_type = 0);
    unsigned char sendCommandFromMaster(CommunicationCommandState command_state);
    unsigned char getReceiveState(CommunicationCommandState command_state);
    unsigned char *getSerializeData(void);
    unsigned short int getSerializedLength(void);
    unsigned char analyseReceiveByte(unsigned char recv_byte);
    void setOwnerID(unsigned char owner_id);
    void setOtherID(unsigned char other_id);
    void setPortNum(unsigned char port_num);
    void enableAck(void);
    void disableAck(void);
private:
    unsigned char analyseReceiveStates(unsigned char recv_data);
    unsigned char analyseReceivePackage(void);
    unsigned char analyseReadCommand(CommunicationCommandState comm_command_state,
                                     unsigned char *comm_data_type,
                                     unsigned short int comm_data_type_length);
    unsigned char analyseWriteCommand(CommunicationCommandState comm_command_state,
                                      unsigned char *comm_data_type,
                                      unsigned short int comm_data_type_length);
    void sendData(CommunicationCommandState command_state,
                  unsigned char *comm_data_type,
                  unsigned short int comm_data_type_length);
    void sendMessage(void);
private:
    unsigned char              shake_hands_state_;
    unsigned char              recv_package_state_[LAST_COMMAND];
    unsigned char              port_num_;
    unsigned char              owner_id_;
    unsigned char              other_id_;
    unsigned char              link_ack_en_;
    CommunicationDataType     *comm_data_type_;
    CommunicationReceiveState  comm_receive_state_;
    CommunicationCommandState  comm_command_state_;
    CommunicationMode          comm_mode_;
    CommunicationMessage       comm_message_recv_;
    CommunicationMessage       comm_message_send_;
    float                      recv_package_count_;
    float                      send_package_count_;
    float                      package_update_freq_;
    unsigned char              send_buffer_[MESSAGE_BUFFER_SIZE + 20];
    unsigned short int         send_buffer_length_;
    unsigned short int         recv_checksum_;
    unsigned short int         recv_message_length_;
    unsigned short int         recv_byte_count_;
};

#endif // BREEZE_COMMUNICATION_LINK
