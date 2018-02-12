#include "mbed.h"
#include "rtos.h"
#include "xbee.h"

#define nullptr 0

DigitalOut led1(LED1);

Serial pc(USBTX, USBRX);

DigitalIn * btn;
I2C * acc;
Serial *xbee;
DigitalOut *xbeeRst;



int ReadButton(char*);
int ReadAccelerometer(char*);

const uint8_t DEVICE_COUNT = 2;

int (* readFunctions[DEVICE_COUNT]) (char *) = {&ReadButton, &ReadAccelerometer};

const int HARDWARE_RESET_SIGNAL = 0x10;
const int JOINED_NETWORK_SIGNAL = 0x20;
const int TICKER_SIGNAL = 0x40;
const int RESPONSE_SIGNAL = 0x80;

Thread * XBeeConsumer;
Thread * XBeeProducer;
Ticker timer;

int responseStatus;

/*******************************************************/
/**********************UTILITIES************************/
/*******************************************************/

PinName GetPinName(const int p){
    switch(p){
        case 5: return p5;
        case 6: return p6;
        case 7: return p7;
        case 8: return p8;
        case 9: return p9;
        case 10: return p10;
        case 11: return p11;
        case 12: return p12;
        case 13: return p13;
        case 14: return p14;
        case 15: return p15;
        case 16: return p16;
        case 17: return p17;
        case 18: return p18;
        case 19: return p19;
        case 20: return p20;
        case 21: return p21;
        case 22: return p22;
        case 23: return p23;
        case 24: return p24;
        case 25: return p25;
        case 26: return p26;
        case 27: return p27;
        case 28: return p28;
        case 29: return p29;
        case 30: return p30;
    }
    pc.printf("Numero de pin invalid");
    return NC;
}

/*******************************************************/
/***********************CONFIG**************************/
/*******************************************************/
int panID;
int pauseTime;
int btnPin;
int accSdaPin;
int accSclPin;
int xbeeTxPin;
int xbeeRxPin;
int xbeeRstPin;

char key[10];

//LocalFileSystem local("local");

void ReadConfig(){

  panID = 666999;
  pauseTime = 10;
  accSclPin = 10;
    accSdaPin = 9;
  btnPin = 11;
    xbeeTxPin= 13;
    xbeeRxPin= 14;
  xbeeRstPin=8;
    
    
    
}

/*******************************************************/
/**********************XBEE SEND************************/
/*******************************************************/

char frameID = 0;

inline char GetFrameID(){
    ++frameID;
    if (frameID == 0){
        frameID = 1;
    }
    return frameID;
}

const char coordinator64bit[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
const char coordinator16bit[2] = {0xff, 0xfe};

void SetCheckSum(char * buffer){
    uint16_t length = GetFrameLength(buffer);
    
    char sum = 0;
    
    int max = length + 3;
    
    for(int i = 3; i < max; ++i){
        sum += buffer[i];
    }
    
    buffer[max] = 0xff - sum;
}

void XBeeSend(char * buffer, int count){
    pc.printf("message sent :: \r\n");
    for ( int i = 0; i < count; ++i ){
        xbee->putc(buffer[i]);
        wait_us(25);
                pc.printf("%02x ", buffer[i]);
    }
    pc.printf("\r\n");
}

void XBeeSentTransmitCommand(char * data, int dataLength){
    char buffer[128];
    buffer[START_IDX] = START;
    buffer[LENGTH_MSB_IDX] = (dataLength + TRANSMIT_MIN_SIZE) >> 8;
    buffer[LENGTH_LSB_IDX] = (dataLength + TRANSMIT_MIN_SIZE) & 0xff;
    buffer[API_ID_IDX] = API_ID_TRANSMIT;
    buffer[FRAME_ID_IDX] = GetFrameID();
    memcpy(&buffer[TRANSMIT_64BIT_MSB_IDX], coordinator64bit, ADDR_64BIT_SIZE);
    memcpy(&buffer[TRANSMIT_16BIT_MSB_IDX], coordinator16bit, ADDR_16BIT_SIZE);
    buffer[TRANSMIT_BROADCAST_IDX] = TRANSMIT_DEFAULT_BROADCAST;
    buffer[TRANSMIT_OPT_IDX] = TRANSMIT_DEFAULT_OPT;
    memcpy(&buffer[TRANSMIT_DATA_IDX], data, dataLength);
    
    SetCheckSum(buffer);
    
    while(true) {
        XBeeSend(buffer, dataLength + TRANSMIT_MIN_SIZE + FRAME_MIN_SIZE);
                pc.printf("This Transit error occured : %02x\r\n"),
        Thread::signal_wait(RESPONSE_SIGNAL);
        
        switch (responseStatus){
        case TRANSMIT_STATUS_OK:
            return;
        default:
            pc.printf("This Transit error occured : %02x\r\n", responseStatus);
            break;
        }
    }
}

void XBeeSendATCommand(bool queue, char * type, char * data, int dataLength){
    char buffer[128];
    buffer[START_IDX] = START;
    buffer[LENGTH_MSB_IDX] = (dataLength + AT_MIN_SIZE) >> 8;
    buffer[LENGTH_LSB_IDX] = (dataLength + AT_MIN_SIZE) & 0xff;
    buffer[API_ID_IDX] = queue ? API_ID_AT_CMD_QUEUE : API_ID_AT_CMD;
    buffer[FRAME_ID_IDX] = GetFrameID();
    memcpy(&buffer[AT_CMD_ID_IDX], type, AT_CMD_ID_SIZE);
    memcpy(&buffer[AT_PARAM_IDX], data, dataLength);
    
    SetCheckSum(buffer);
    
    SetCheckSum(buffer);
    
    if (!ValidateCheckSum(buffer)){
        pc.printf("CheckSum problem\r\n");
    }
        else{
                pc.printf("CheckSum GOOOD\r\n");
        }
    
    while(true){
        XBeeSend(buffer, dataLength + AT_MIN_SIZE + FRAME_MIN_SIZE);
   
        
        switch (responseStatus){
        case AT_CMD_RSP_STATUS_OK:
            return;
        case AT_CMD_RSP_STATUS_ERROR:
        case AT_CMD_RSP_STATUS_INVALID_CMD:
        case AT_CMD_RSP_STATUS_INVALID_PARAM:
        case AT_CMD_RSP_STATUS_TX_FAILURE:
        default:
            pc.printf("This AT error occured : %02x\r\n", responseStatus);
            break;
        }
    }
}

 void XBeeSendATID(){
    char idBuf[8];
    for (int i = 0; i < 8; ++i){
        idBuf[i] = (panID >> (56 - 8 * i)) & 0xff;
    }
        pc.printf(" pan id id bufeered = %02X \r\n", idBuf);// ST      
    XBeeSendATCommand(true, "ID", idBuf, 8);
}

 void XBeeSendATWR(){
    XBeeSendATCommand(true, "WR", nullptr, 0);
}

 void XBeeSendATAC(){
    XBeeSendATCommand(true, "AC", nullptr, 0);
}


/*******************************************************/
/************************READ***************************/
/*******************************************************/

int ReadButton(char* buffer){

    
    return 4;
}

int ReadAccelerometer(char* buffer){
    const char deviceAddr = 0x1d<<1;
    char temp[1];
    temp[0] = 0x01; // Adresse de OUT_X_MSB
    
    buffer[0] = 'A';
    buffer[1] = 'C';
    buffer[2] = 'C';
    // Lecture des données X, Y et Z
    acc->write(deviceAddr, temp, 1, true);
    acc->read(deviceAddr, &buffer[3], 6);
    
    SetCheckSum(buffer);
    
    //Number of bytes to send
    return 9;
}

void ReadDevices(){
    int count;
    char buffer[64];
        
    for ( int i = 0; i < DEVICE_COUNT; ++i){
        count = readFunctions[i](buffer);
        XBeeSentTransmitCommand(buffer, count);
    }
}

bool Xbee_read(){

while(xbee->readable() == 0){
        //pc.printf(" ");
    }
        pc.printf("caractere recu ::  \r\n");
    while(xbee->readable() == 1){

        //pc.printf("%d %d \r\n", serialXBEE.readable(), pc.readable());
        led1 = !led1;  
       uint8_t caratere_recu = (xbee->getc())  ;
						if(caratere_recu == 0x7e){
							 pc.printf("\r\n caractere recu ::  \r\n");
						}
        pc.printf("%02X ", caratere_recu);// ST      
        wait_ms(50);
    }
    
    pc.printf("\r\n\r\n");
}


int main() {
		// Lecture de la configuration.
		ReadConfig();
		pc.printf("config read\r\n");
		//Créer les interfaces de communication des capteurs avec les données de la config.   
		DigitalIn mainBtn(GetPinName(btnPin));
		I2C mainAcc(GetPinName(accSdaPin), GetPinName(accSclPin));
		Serial mainXbee(GetPinName(xbeeTxPin), GetPinName(xbeeRxPin));
		DigitalOut mainXbeeRst(GetPinName(xbeeRstPin));

		//Rendre les interfaces de communication globaux.
		btn = &mainBtn;
		acc = &mainAcc;
		xbee = &mainXbee;
		xbeeRst = &mainXbeeRst;

		xbeeRst->write(0);
		wait(0.4);
		xbeeRst->write(1);
		pc.printf("reset done, waiting for answer \r\n \r\n");
		Xbee_read();
		pc.printf("HW reset received,  setting the pan_id\r\n");
		char buffer[11] = {START ,0x00, 0x07 ,0x09 ,0x3D ,0x49 ,0x44 ,0x66 ,0x69 ,0x99 ,0xC4};
		XBeeSend(buffer,sizeof(buffer));

		pc.printf("waiting answer from pan id set");
		Xbee_read();

		XBeeSendATWR();
		pc.printf("waiting answer from XBeeSendATWR");
		Xbee_read();
		XBeeSendATAC();
		pc.printf("waiting answer from XBeeSendATAC");
		Xbee_read();

		char buffer_send[22] = {0x7E ,0x00 ,0x11 ,0x10 ,0x01 ,0x00 ,0x00 ,0x00 ,0x00 ,0x00 ,0x00 ,0x00 ,0x00 ,0xFF ,0xFE ,0x00 ,0x00 ,0x61 ,0x73 ,0x64 ,0xB9};
		XBeeSend(buffer,sizeof(buffer_send));

		Xbee_read();
}
