#include <tcs.h>
#include <stdio.h>

int main() {

	uint16_t rgbc[] = {0x0000, 0x0000, 0x0000, 0x0000};
	tcsGetStandaloneRgbc(rgbc);
	for(int i =0; i < 4; i++) {
		printf("%d\n", rgbc[i]);
	}
	return 0;
}