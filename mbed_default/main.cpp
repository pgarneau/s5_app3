#include "mbed.h"

DigitalOut pin8(p8);
Serial pc(USBTX, USBRX, 9600);
Serial xbee(p13, p14, 9600);

int main() {
	pin8 = 0;
	wait(0.4);
	pin8 = 1;
	
    while (true) {
		if (pc.readable()) {
			int pc_result = pc.getc();
			xbee.putc(pc_result);
		}
		
		if (xbee.readable()) {
			int xbee_result = xbee.getc();
			pc.putc(xbee_result);
		}
    }
}

