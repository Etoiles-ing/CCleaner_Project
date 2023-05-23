#include "tcs.h"
#include "i2c.h"

// Send a data byte to the command specified in reg
static void tcsWrite(uint8_t reg, uint8_t data) {
	uint8_t buffer[] = {reg, data};

	// Send buffer through I2C
	I2C_Master_Transmit(TCS_I2C_WRITE, buffer, 2);
}

// Read dataNb bytes to the data array (make sure it's big enough) at the command specified with reg
static void tcsRead(uint8_t reg, uint8_t *data, uint8_t dataNb) {
	// Send address through I2C
	I2C_Master_Transmit(TCS_I2C_WRITE, &reg, 1);

	// Send read and receive data
	I2C_Master_Receive(TCS_I2C_READ, data, dataNb);
}

// Get a measure from a powered-down TCS. Returns it back to sleep afterwards.
// You must pass a pointer to a 16-bit uint array long enough for 4 values.
// The returned array follows the following format: [red, green, blue, clear]
void tcsGetStandaloneRgbc(uint16_t *rgbc) {
	// Power-on the TCS
	tcsWrite(TCS_ENABLE, TCS_ENABLE_PON);

	// Wait at least 2.4ms
	// TODO: replace with something better, like a timer
	volatile uint32_t n = 0;
	while (n < 100000) {
		++n;
	}

	// Enable 4x gain
	tcsWrite(TCS_CONTROL, TCS_CONTROL_AGAIN_0);

	// Activate the ADC to start a RGBC measure
	tcsWrite(TCS_ENABLE, TCS_ENABLE_AEN | TCS_ENABLE_PON);

	// Wait until a measure is ready
	uint8_t status = 0x00;
	do {
		tcsRead(TCS_STATUS, &status, 1);
	} while ((status & TCS_STATUS_AVALID) == 0);

	// Get RGBC data
	uint8_t rgbc_buffer[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
	tcsRead(TCS_CDATA, rgbc_buffer, 8);

	// Format 8-bit rgbc_buffer data into the 16-bit array
	rgbc[0] = ((uint16_t)(rgbc_buffer[3]) << 8) | rgbc_buffer[2];
	rgbc[1] = ((uint16_t)(rgbc_buffer[5]) << 8) | rgbc_buffer[4];
	rgbc[2] = ((uint16_t)(rgbc_buffer[7]) << 8) | rgbc_buffer[6];
	rgbc[3] = ((uint16_t)(rgbc_buffer[1]) << 8) | rgbc_buffer[0];

	// Power-off the TCS
	tcsWrite(TCS_ENABLE, 0x00);
}
