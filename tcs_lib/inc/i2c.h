#ifndef I2C_H_
#define I2C_H_

/**
  * @brief  Send between 1 and 9 data bytes on the I2C-1 bus
  * @param  addr address of the target component on the I2C bus
  * @param  buffer data bytes buffer. 
  *         The first one sent will be the register offset on your target component
  * @param  size number of bytes in array. Must be between 1 and 9
  * @retval none
  * */
void I2C_Master_Transmit(uint8_t addr, uint8_t *buffer, uint8_t size);

/**
  * @brief  Receive between 1 and 8 data bytes from the I2C-1 bus
  * @param  addr address of the target component on the I2C bus
  * @param  buffer data bytes buffer to be filled up (make sure it's big enough)
  * @param  size number of bytes to receive. Must be between 1 and 8
  * @retval none
  * */
void I2C_Master_Receive(uint8_t addr, uint8_t *buffer, uint8_t size);

#endif /* I2C_H_ */
