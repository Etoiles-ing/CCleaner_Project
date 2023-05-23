#ifndef I2C_H_
#define I2C_H_

void I2C_Master_Transmit(uint8_t addr, uint8_t *buffer, uint8_t size);
void I2C_Master_Receive(uint8_t addr, uint8_t *buffer, uint8_t size);

#endif /* I2C_H_ */
