#include "mbed.h"
#include "rtos.h"
#include "xbee.h"
#include "EthernetInterface.h"

#define nullptr 0

DigitalOut led1(LED1);

Serial pc(USBTX, USBRX);

Serial *xbee;
DigitalOut *xbeeRst;

const int HARDWARE_RESET_SIGNAL =      0x10;
const int COORDINATOR_STARTED_SIGNAL = 0x20;
const int TICKER_SIGNAL =              0x40;
const int RESPONSE_SIGNAL =            0x80;

Thread * XBeeConsumer;
Thread * XBeeProducer;
Thread * EventConsumer;
Thread * EthernetConsumer;
Ticker timer;

int responseStatus;

char * BTN_ID = "BTN";
char * ACC_ID = "ACC";

struct ButtonEvent{
    char id[3];
    bool state;
};

struct AccelerometerEvent {
    char id[3];
    char x[2], y[2], z[2];
};

MemoryPool<ButtonEvent, 32> btnPool;
MemoryPool<AccelerometerEvent, 32> accPool;
Queue<void, 64> event;

struct Rooter{
    char addr64[8];
    char addr16[2];
    
    bool operator==(const Rooter & rhs){
        bool same = true;
        
        for (int i = 0; i < 8; ++i){
            if (addr64[i] != rhs.addr64[i]){
                return false;
            }
        }
        
        for (int i = 0; i < 2; ++i){
            if (addr16[i] != rhs.addr16[i]){
                return false;
            }
        }
        
        return same;
    }
};

#define ROOTER_MAX 2
Rooter rooters[ROOTER_MAX];
int rooterCount = 0;

Mutex rooterMutex;

TCPSocketConnection socket;

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
int xbeeTxPin;
int xbeeRxPin;
int xbeeRstPin;
char server[16];

char key[10];

LocalFileSystem local("local");

void ReadConfig(){
  /*  memset(server, 0x00, 16);
    FILE * f = fopen("/local/coord.cfg", "r");
    fscanf(f,"%s %x", key, &panID);
   pc.printf("Lecture de la config %s : %04x\r\n", key, panID);
    fscanf(f,"%s %d %d %d", key, &xbeeTxPin, &xbeeRxPin, &xbeeRstPin);
    pc.printf("Lecture de la config %s : %d %d %d\r\n", key, xbeeTxPin, xbeeRxPin, xbeeRstPin);
    fscanf(f,"%s %s", key, server);
    pc.printf("Lecture de la config %s : %s\r\n", key, server);
    fclose(f);*/
      panID = 666999;
  xbeeTxPin = 13;
  xbeeRxPin =14 ;
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
      pc.printf("SENT caractere :\r\n");
    for ( int i = 0; i < count; ++i ){
        xbee->putc(buffer[i]);
        wait_us(25);
           pc.printf("%02x ", buffer[i]);
    }
 pc.printf("\r\n");
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
    
    while(true){
        XBeeSend(buffer, dataLength + AT_MIN_SIZE + FRAME_MIN_SIZE);
    
       // Thread::signal_wait(RESPONSE_SIGNAL);
        
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



void XbeeSendRemoteAtCommand(char * addr64, char * addr16, char opt, char * type, char * data, int dataLength){
    char buffer[128];
    buffer[START_IDX] = START;
    buffer[LENGTH_MSB_IDX] = (dataLength + REMOTE_AT_RQST_MIN_SIZE) >> 8;
    buffer[LENGTH_LSB_IDX] = (dataLength + REMOTE_AT_RQST_MIN_SIZE) & 0xff;
    buffer[API_ID_IDX] = API_ID_REMOTE_AT_RQST;
    buffer[FRAME_ID_IDX] = GetFrameID();
    memcpy(&buffer[REMOTE_AT_RQST_64BIT_MSB_IDX], addr64, ADDR_64BIT_SIZE);
    memcpy(&buffer[REMOTE_AT_RQST_16BIT_MSB_IDX], addr16, ADDR_16BIT_SIZE);
    buffer[REMOTE_AT_RQST_OPT_IDX] = opt;
    memcpy(&buffer[REMOTE_AT_RQST_AT_CMD1_IDX], type, AT_CMD_ID_SIZE);
    memcpy(&buffer[REMOTE_AT_RQST_AT_PARAM_IDX], data, dataLength);
    
    SetCheckSum(buffer);
    
    while(true){
        XBeeSend(buffer, dataLength + REMOTE_AT_RQST_MIN_SIZE + FRAME_MIN_SIZE);
        Thread::signal_wait(RESPONSE_SIGNAL);
        switch (responseStatus){
        case REMOTE_AT_CMD_RSP_STATUS_OK:
            return;
        default:
            pc.printf("This AT error occured : %02x\r\n", responseStatus);
            break;
        }
    }
}





/*******************************************************/
/************************MAIN***************************/
/*******************************************************/

inline void XBeeSendATID(){
    char idBuf[8];
    for (int i = 0; i < 8; ++i){
        idBuf[i] = (panID >> (56 - 8 * i)) & 0xff;
    }
    XBeeSendATCommand(true, "ID", idBuf, 8);
}

inline void XBeeSendATWR(){
    XBeeSendATCommand(true, "WR", nullptr, 0);
}

inline void XBeeSendATAC(){
    XBeeSendATCommand(true, "AC", nullptr, 0);
}
bool Xbee_read(){

while(xbee->readable() == 0){
        //pc.printf("wait for xbee message\r\n");
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
        pc.printf("config read for coordonator\r\n");
        //Créer les interfaces de communication des capteurs avec les données de la config.   

        Serial mainXbee(GetPinName(xbeeTxPin), GetPinName(xbeeRxPin));
        DigitalOut mainXbeeRst(GetPinName(xbeeRstPin));

        //Rendre les interfaces de communication globaux.
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
         
         pc.printf("\r\n ***************************** \r\n ATTENTE MESSAGE DU ROOTEUR\r\n ***************************** \r\n");
         Xbee_read();
         
       pc.printf(" read done, shit is done \r\n ");
          
        
}