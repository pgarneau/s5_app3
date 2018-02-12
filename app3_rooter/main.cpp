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

inline void SendRemoteD0Command(char* addr64, char* addr16, bool on){
    char data[1] = {on ? 0x05 : 0x04};
    XbeeSendRemoteAtCommand(addr64, addr16, 0x02, "D0", data, 1);
}

/*******************************************************/
/**********************XBEE READ************************/
/*******************************************************/

void HandleAtRemoteCommandResponse(char * cmd){
    responseStatus = cmd[REMOTE_CMD_RSP_STATUS_IDX];
    XBeeProducer->signal_set(RESPONSE_SIGNAL);
}

void HandleBtnPacket(char* cmd){
    ButtonEvent* evt = btnPool.alloc();
    memcpy(evt->id, BTN_ID, 3);
    evt->state = cmd[RECEIVED_PACKET_DATA_IDX + 3] == 0x01;
    event.put((void*)evt);
}

void HandleAccPacket(char* cmd){
    AccelerometerEvent* evt = accPool.alloc();
    memcpy(evt->id, ACC_ID, 3);
    memcpy(evt->x, &cmd[RECEIVED_PACKET_DATA_IDX + 3], 2);
    memcpy(evt->y, &cmd[RECEIVED_PACKET_DATA_IDX + 5], 2);
    memcpy(evt->z, &cmd[RECEIVED_PACKET_DATA_IDX + 7], 2);
    event.put((void*)evt);
}

void HandleXbeeReceivedPacket(char * cmd){
    if (rooterCount < ROOTER_MAX){
        Rooter r;
        memcpy(r.addr64, &cmd[RECEIVED_PACKET_64BIT_MSB_IDX], 8);
        memcpy(r.addr16, &cmd[RECEIVED_PACKET_16BIT_MSB_IDX], 2);
        
        bool found = false;
        for (int i = 0; i < rooterCount; ++i){
            if (rooters[i] == r){
                found = true;
                break;
            }
        }
        if (!found){
            rooterMutex.lock();
            rooters[rooterCount] = r;
            ++rooterCount;
            rooterMutex.unlock();
        }
    }
    
    if (cmd[RECEIVED_PACKET_DATA_IDX] == BTN_ID[0] && 
        cmd[RECEIVED_PACKET_DATA_IDX + 1] == BTN_ID[1] && 
        cmd[RECEIVED_PACKET_DATA_IDX + 2] == BTN_ID[2]){
        HandleBtnPacket(cmd);
    }
    
    if (cmd[RECEIVED_PACKET_DATA_IDX] == ACC_ID[0] && 
        cmd[RECEIVED_PACKET_DATA_IDX + 1] == ACC_ID[1] && 
        cmd[RECEIVED_PACKET_DATA_IDX + 2] == ACC_ID[2]){
        HandleAccPacket(cmd);
    }
}

void HandleXbeeModemStatus(char * cmd){
    switch(cmd[MODEM_STATUS_STATUS_IDX]){
    case MODEM_STATUS_HARDWARE_RST:
        XBeeProducer->signal_set(HARDWARE_RESET_SIGNAL);
        break;
    case MODEM_STATUS_COORDINATOR_STARTED:
        XBeeProducer->signal_set(COORDINATOR_STARTED_SIGNAL);
        break;
    default:
        pc.printf("Unhandled modem status received : %02x\r\n", cmd[MODEM_STATUS_STATUS_IDX]);
        break;
    }
}

void HandleXBeeATCommandResponse(char * cmd){
    responseStatus = cmd[AT_CMD_RSP_STATUS_IDX];
    XBeeProducer->signal_set(RESPONSE_SIGNAL);
}

void HandleXbeeReceivedCommand(char * cmd){
    switch(cmd[API_ID_IDX]){
    case API_ID_AT_CMD_RSP:
        HandleXBeeATCommandResponse(cmd);
        break;
    case API_ID_MODEM_STATUS:
        HandleXbeeModemStatus(cmd);
        break;
    case API_ID_RECEIVED_PACKET:
        HandleXbeeReceivedPacket(cmd);
        break;
    case API_ID_REMOTE_CMD_RSP:
        HandleAtRemoteCommandResponse(cmd);
        break;
    default:
        pc.printf("Unhandle XBee Command received : %02x\r\n", cmd[API_ID_IDX]);
        break;
    }
}

/*******************************************************/
/************************EVENT**************************/
/*******************************************************/

void HandleBtnEvent(ButtonEvent* data){
     char out[24];
     out[23] = 0x00;
     sprintf(out, "Event BTN: %s", data->state ? "Pressed" : "Released");
     
     pc.printf("Sending to Server : %s\r\n", out);
     socket.send_all(out, data->state ? 20 : 21);
}

#define NEGATIVE_PADDING 0xfffff000;
int AccDataToInt(char* data)
{
    int x = ((int)data[0])<<4;
    x |= data[1]>>4;
    if ((data[0] & 0x80) != 0) {
        x |= NEGATIVE_PADDING;
    }
    return x;
}

void HandleAccEvent(AccelerometerEvent* data){
    char out[40];
    out[39] = 0;
    int x = AccDataToInt(data->x);
    int y = AccDataToInt(data->y);
    int z = AccDataToInt(data->z);
    
    float x_g = (float)x/1024.0f;
    float y_g = (float)y/1024.0f;
    float z_g = (float)z/1024.0f;
    sprintf(out, "Event ACC: x=%01.02fg, y=%01.02fg, z=%01.02fg", x_g, y_g, z_g);  
    
    pc.printf("Sending to Server : %s\r\n", out);
    socket.send_all(out, 40);
}

/*******************************************************/
/************************INIT***************************/
/*******************************************************/

bool InitXBee(){
    xbeeRst->write(0);
    wait(0.4);
    xbeeRst->write(1);
    
    Thread::signal_wait(HARDWARE_RESET_SIGNAL);
    
    XBeeSendATID();
    XBeeSendATWR();
    XBeeSendATAC();
    
    Thread::signal_wait(COORDINATOR_STARTED_SIGNAL);
     
    pc.printf("XBee configured\r\n");
    
    return true;
}

bool InitEthernet(){
    EthernetInterface eth;
    // No DHCP
    eth.init("192.168.2.3", "255.255.255.0", server); 
    // DHCP
    //eth.init();
    eth.connect();
    printf("\nClient IP Address is %s\r\n", eth.getIPAddress());
    
    // Connect to Server
    while (socket.connect(server, 7) < 0) {
        printf("Unable to connect to (%s) on port (%d)\r\n", server, 7);
        wait(1);
    }
    
    printf("Connected to Server at %s\r\n", server);
    
    return true;
}


/*******************************************************/
/************************MAIN***************************/
/*******************************************************/

inline char XbeeReadChar(){
    while(!xbee->readable()){
    }
    return xbee->getc();
}

void ConsumerMain(){
    char buffer[128];
    while(true){
        buffer[START_IDX] = XbeeReadChar();
        if (buffer[START_IDX] != START){
            pc.printf("Wrong start byte received : %02x\r\n", buffer[START_IDX]);
            continue;
        }
        buffer[LENGTH_MSB_IDX] = XbeeReadChar();
        buffer[LENGTH_LSB_IDX] = XbeeReadChar();
        int length = GetFrameLength(buffer);
        
        for (int i = 0; i <= length; ++i){
            buffer[i + API_ID_IDX] = XbeeReadChar();
        }
        if (!ValidateCheckSum(buffer)){
            pc.printf("Bad CheckSum\r\n");
            
            for (int i = 0; i < length + FRAME_MIN_SIZE; ++i){
                pc.printf("%02x ", buffer[i]);
            }
            pc.printf("\r\n");
            continue;
        }
        
        HandleXbeeReceivedCommand(buffer);
    }
}

void ToggleRemoteRooters(bool on){
    rooterMutex.lock();
    for(int i = 0; i < rooterCount; ++i){
        SendRemoteD0Command(rooters[i].addr64, rooters[i].addr16, on);
    }
    rooterMutex.unlock();
}

bool ProducerInit(){
    if (!InitEthernet()){
        pc.printf("Connection problem with the Ethernet\r\n");
        return false;
    }
    
    if (!InitXBee()){
        pc.printf("Connection problem with the XBee\r\n");
        return false;
    }
    
    return true;
}

void Tick(){
    XBeeProducer->signal_set(TICKER_SIGNAL);
}

bool initDone = false;

void ProducerMain(const void*){
    if (!ProducerInit()){
        pc.printf("Initialization problem\r\n");
        return;
    } else {
        initDone = true;
    }
    
    timer.attach(&Tick, 1);
    bool on = true;
    while(true){
        Thread::signal_wait(TICKER_SIGNAL);
        ToggleRemoteRooters(on);
        on = !on;
    }
}

void EventConsumerMain(const void*){
    while(true){
        void* ptr = event.get().value.p;
        
        char * id = (char*) ptr;
        // Compare l'ID pour trouver le type d'évènement
        if (id[0] == BTN_ID[0] && id[1] == BTN_ID[1] && id[2] == BTN_ID[2]){
            ButtonEvent* BtnEvt = (ButtonEvent*)ptr;
            HandleBtnEvent(BtnEvt);
            btnPool.free(BtnEvt);
        } else if (id[0] == ACC_ID[0] && id[1] == ACC_ID[1] && id[2] == ACC_ID[2]){
            AccelerometerEvent* AccEvt = (AccelerometerEvent*)ptr;
            HandleAccEvent(AccEvt);
            accPool.free(AccEvt);
        } else {
            pc.printf("Unknown event : %c%c%c\r\n", id[0], id[1], id[2]);
        }
    }
}

void EthernetConsumerMain(const void *){
    // Attend que l'initialisation soit terminée avant de commencer
    while (!initDone){
        Thread::yield();
    }
    // Receive message from server
    char buf[256];
    while(true){
        int n = socket.receive(buf, 256);
        buf[n] = 0x00;
        printf("Received from Server : %s\r\n", buf);
    }
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
        pc.printf("config read for rooter\r\n");
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
         pc.printf("\r\n ***************************** \r\n MESSAGE ENVOYER AU ROOTER\r\n ***************************** \r\n");

          
        char buffer_send[22] = {0x7E ,0x00 ,0x11 ,0x10 ,0x01 ,0x00 ,0x00 ,0x00 ,0x00 ,0x00 ,0x00 ,0x00 ,0x00 ,0xFF ,0xFE ,0x00 ,0x00 ,0x61 ,0x73 ,0x64 ,0xB9};
        XBeeSend(buffer_send,sizeof(buffer_send));
                 
       pc.printf(" sending done, now wairting fot xbee answer \r\n ");
        Xbee_read();
         pc.printf("done");
    }