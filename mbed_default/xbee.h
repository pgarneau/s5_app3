#include "mbed.h"

#ifndef XBEE_H
#define XBEE_H

#define START 0x7E

#define ADDR_64BIT_SIZE 8
#define ADDR_16BIT_SIZE 2
#define AT_CMD_ID_SIZE  2

// Présent dans tous les frames
#define START_IDX      0
#define LENGTH_MSB_IDX 1
#define LENGTH_LSB_IDX 2
#define API_ID_IDX     3
#define FRAME_MIN_SIZE 4

// Les API ID supportés2
#define API_ID_AT_CMD          0x08
#define API_ID_AT_CMD_QUEUE    0x09
#define API_ID_TRANSMIT        0x10
#define API_ID_REMOTE_AT_RQST  0x17
#define API_ID_AT_CMD_RSP      0x88
#define API_ID_MODEM_STATUS    0x8A
#define API_ID_TRANSMIT_STATUS 0x8B
#define API_ID_RECEIVED_PACKET 0x90
#define API_ID_REMOTE_CMD_RSP  0x97

// Présent dans la majorité des frames
#define FRAME_ID_IDX  4

// Spécifique pour l'AT Command
#define AT_CMD_ID_IDX 5
#define AT_PARAM_IDX  7
#define AT_MIN_SIZE   4

// Spécifique pour l'AT Command Queue
#define AT_QUEUE_CMD_1_IDX   5
#define AT_QUEUE_CMD_2_IDX   6
#define AT_QUEUE_PARAM_IDX   7
#define AT_QUEUE_MIN_SIZE    4

// Spécifique pour la Transmit Command
#define TRANSMIT_64BIT_MSB_IDX 5
#define TRANSMIT_64BIT_LSB_IDX 12
#define TRANSMIT_16BIT_MSB_IDX 13
#define TRANSMIT_16BIT_LSB_IDX 14
#define TRANSMIT_BROADCAST_IDX 15
#define TRANSMIT_OPT_IDX       16
#define TRANSMIT_DATA_IDX      17
#define TRANSMIT_MIN_SIZE      14

// Les options de la Transmit Command
#define TRANSMIT_DEFAULT_BROADCAST 0x00
#define TRANSMIT_DEFAULT_OPT       0x00

// Spécifique pour la Remote AT Request Command
#define REMOTE_AT_RQST_64BIT_MSB_IDX 5
#define REMOTE_AT_RQST_64BIT_LSB_IDX 12
#define REMOTE_AT_RQST_16BIT_MSB_IDX 13
#define REMOTE_AT_RQST_16BIT_LSB_IDX 14
#define REMOTE_AT_RQST_OPT_IDX       15
#define REMOTE_AT_RQST_AT_CMD1_IDX   16
#define REMOTE_AT_RQST_AT_CMD2_IDX   17
#define REMOTE_AT_RQST_AT_PARAM_IDX  18
#define REMOTE_AT_RQST_MIN_SIZE      15

// Les options pour la Remote AT Request Command
#define REMOTE_AT_RQST_DEFAULT_OPT      0x00
#define REMOTE_AT_RQST_OPT_APPLY_CHANGE 0x02

// Spécifique pour la AT Command Response
#define AT_CMD_RSP_AT_CMD1_IDX 5
#define AT_CMD_RSP_AT_CMD2_IDX 6
#define AT_CMD_RSP_STATUS_IDX  7
#define AT_CMD_RSP_DATA_IDX    8
#define AT_CMD_RSP_MIN_SIZE    5

// Les status pour la AT Command Response
#define AT_CMD_RSP_STATUS_OK            0x00
#define AT_CMD_RSP_STATUS_ERROR         0x01
#define AT_CMD_RSP_STATUS_INVALID_CMD   0x02
#define AT_CMD_RSP_STATUS_INVALID_PARAM 0x03
#define AT_CMD_RSP_STATUS_TX_FAILURE    0x04

// Spécifique pour la Modem Status Command
#define MODEM_STATUS_STATUS_IDX         4

// Les status pour la Modem Status Command
#define MODEM_STATUS_HARDWARE_RST        0x00
#define MODEM_STATUS_JOINED_NETWORK      0x02
#define MODEM_STATUS_DISASSOCIATED       0x03
#define MODEM_STATUS_COORDINATOR_STARTED 0x06

// Spécifique pour la Transit Status Command
#define TRANSMIT_STATUS_16BIT_MSB_IDX        5
#define TRANSMIT_STATUS_16BIT_LSB_IDX        6
#define TRANSMIT_STATUS_RETRY_COUNT_IDX      7
#define TRANSMIT_STATUS_DELIVERY_STATUS_IDX  8
#define TRANSMIT_STATUS_DISCOVERY_STATUS_IDX 9

#define TRANSMIT_STATUS_OK 0x00

// Spécifique pour la Received Packet Command
#define RECEIVED_PACKET_64BIT_MSB_IDX 4
#define RECEIVED_PACKET_64BIT_LSB_IDX 11
#define RECEIVED_PACKET_16BIT_MSB_IDX 12
#define RECEIVED_PACKET_16BIT_LSB_IDX 13
#define RECEIVED_PACKET_OPT_IDX       14
#define RECEIVED_PACKET_DATA_IDX      15
#define RECEIVED_PACKET_MIN_SIZE      12

// Spécifique pour la Remote AT Command Response
#define REMOTE_CMD_RSP_64BIT_MSB_IDX 5
#define REMOTE_CMD_RSP_64BIT_LSB_IDX 12
#define REMOTE_CMD_RSP_16BIT_MSB_IDX 13
#define REMOTE_CMD_RSP_16BIT_LSB_IDX 14
#define REMOTE_CMD_RSP_AT_IDX        15
#define REMOTE_CMD_RSP_STATUS_IDX    17
#define REMOTE_CMD_RSP_DATA_IDX      18
#define REMOTE_CMD_RSP_MIN_SIZE      14

// Les status pour la Remote AT Command Response
#define REMOTE_AT_CMD_RSP_STATUS_OK            0x00
#define REMOTE_AT_CMD_RSP_STATUS_ERROR         0x01
#define REMOTE_AT_CMD_RSP_STATUS_INVALID_CMD   0x02
#define REMOTE_AT_CMD_RSP_STATUS_INVALID_PARAM 0x03
#define REMOTE_AT_CMD_RSP_STATUS_TX_FAILURE    0x04

inline uint16_t GetFrameLength(char * buffer){
    return (((uint16_t)buffer[LENGTH_MSB_IDX]) << 8) + buffer[LENGTH_LSB_IDX];
}

bool ValidateCheckSum(char * buffer){
    uint16_t length = GetFrameLength(buffer);
    
    char sum = 0;
    for (int i = 0; i <= length; ++i){
        sum += buffer[i + API_ID_IDX];
    }
    
    return sum == 0xff;
}

inline uint16_t GetAtParamLength(char * buffer){
    return GetFrameLength(buffer) - AT_MIN_SIZE;
}

inline uint16_t GetAtQueueParamLength(char * buffer){
    return GetFrameLength(buffer) - AT_QUEUE_MIN_SIZE;
}

inline uint16_t GetTransmitDataLength(char * buffer){
    return GetFrameLength(buffer) - TRANSMIT_MIN_SIZE;
}

inline uint16_t GetRemoteAtRequestParamLength(char * buffer){
    return GetFrameLength(buffer) - REMOTE_AT_RQST_MIN_SIZE;
}

inline uint16_t GetAtResponseParamLength(char * buffer){
    return GetFrameLength(buffer) - AT_CMD_RSP_MIN_SIZE;
}

inline uint16_t GetReceivedPacketDataLength(char * buffer){
    return GetFrameLength(buffer) - RECEIVED_PACKET_MIN_SIZE;
}

inline long long int Get64Addr(char * buffer, int start){
    long long int addr = 0;
    memcpy(&addr, &buffer[start], ADDR_64BIT_SIZE);
    
    return addr;
}

inline short Get16Addr(char * buffer, int start){
    short addr = 0;
    memcpy(&addr, &buffer[start], ADDR_16BIT_SIZE);
    
    return addr;
}

#endif