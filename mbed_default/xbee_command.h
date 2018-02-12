#ifndef MBED_XBEE_COMMAND_H
#define MBED_XBEE_COMMAND_H

// API Frame Names      API ID
#define AT_CMD          0x08
#define AT_CMD_QUEUE    0x09
#define TX_REQUEST      0x10
#define RMT_AT_RQST     0x17
#define CREATE_SRC_R    0x21
#define AT_CMD_RSPNS    0x88
#define MODEM_STATUS    0x8A
#define ZBEE_TX_STATUS  0x8B
#define RMT_CMD_RSPNS   0x97

// API Frame length
#define AT_MSB         0x00
#define AT_LSB         0x04

#define TX_RQST_MSB    0x00
#define TX_RQST_LSB    0x17

#define RMT_AT_MSB     0x00
#define RMT_AT_LSB     0x10

// 
#define BROADCAST_RAD 0x00

//AT command
#define AT_CMD_ID     0x4944
#define AT_CMD_SC     0x5343
#define AT_CMD_WR     0x5752
#define AT_CMD_AC     0x4143

//
#define START         0x7E
#define CMD_AT        0x08
#define CMD_TX_RQST   0X10
#define RMT_CMD_RQST  0X17
#define PLUS          0x2B
#define FRAME_ID01    0x01

uint16_t PAN_ID                = 0xA2D77;
uint16_t FREQ_LECTURE_CAPTEURS = 0; 

// Fonctions communication entre LPC1768 et XBee

uint8_t build_AT_cmd_frame(uint8_t frame_id, uint16_t p_AT_cmd, uint8_t* p_value, uint8_t p_value_length, uint8_t* p_at_cmd_frame);
uint8_t build_tx_request_frame(uint8_t* p_mac_address, uint16_t p_pan_id, uint8_t* p_tx_request_frame, uint8_t *data, uint8_t data_length);
bool    send_frame_2xbee(uint8_t* frame, uint8_t frame_length);
bool    send_char_2Xbee(uint8_t caratere);
uint8_t receive_Xbee(uint8_t* message_recu);
bool    xbee_init(uint16_t pan_id);
uint8_t build_remote_cmd_request_frame(uint8_t p_frame_id,  uint16_t p_pan_id, uint16_t p_AT_cmd, uint8_t* p_remote_cmd_request_frame);
bool message_received(uint8_t* message_recu);
#endif