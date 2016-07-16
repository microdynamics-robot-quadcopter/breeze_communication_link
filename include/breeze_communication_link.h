#ifndef BREEZE_COMMUNICATION_LINK
#define BREEZE_COMMUNICATION_LINK

#include "breeze_communication_data_type.h"

#define MESSAGE_BUFFER_SIZE 100

typedef enum CommunicationModel {
    MODEL_MASTER, MODEL_SLAVE
} CommunicationModel;

typedef enum CommunicationState {
    WAITING_FF_A,
    WAITING_FF_B,
    GET_SENDER_ID,
    GET_RECEIVER_ID,
    RECEIVE_LENGTH_H,
    RECEIVE_LENGTH_L,
    RECEIVE_PACKAGE,
    RECEIVE_CHECKSUM
} CommunicationState;

typedef enum CommunicationCommand {
    READ_MOTOR_SPEED,
    READ_ROBOT_SPEED,
} CommunicationCommand;

typedef struct CommunicationMessage {
    unsigned char      sender_id;
    unsigned char      receiver_id;
    unsigned short int length;
    unsigned char      data[MESSAGE_BUFFER_SIZE];
} CommunicationMessage;

#endif // BREEZE_COMMUNICATION_LINK
