#include "mbed.h"

DigitalOut pin8(p8);
Serial pc(USBTX, USBRX, 9600);
//DigitalOut pin13(p13);
Serial xbee(p13, p14, 9600);

int main() {
	pin8 = 0;
	Thread::wait(410);
	pin8 = 1;
	
    while (true) {
		if (pc.readable()) {
			int pc_result = pc.getc();
			printf("%d\n\r", pc_result);
			xbee.putc(pc_result);
		}
		
		if (xbee.readable()) {
			int xbee_result = xbee.getc();
			printf("%d\n\r", xbee_result);
			pc.putc(xbee_result);
		}
    }
}

