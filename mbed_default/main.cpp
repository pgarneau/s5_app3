#include "mbed.h"

DigitalOut pin8(p8);

Serial serial(p13, p14, 9600);





// main() runs in its own thread in the OS
int main() {
	pin8 = 0;
	wait(0.4);
	pin8 = 1;
	
    while (true) {
		printf("allo");
    }
}

