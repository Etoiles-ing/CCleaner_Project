#ifndef INC_TCS_H_
#define INC_TCS_H_

#include <stdint.h>

/****************** I2C-related constants for the color sensor *******************/
#define I2C_WRITE (0x0U)
#define I2C_READ (0x1U)
#define TCS_I2C_ADDR (0x29U)
#define TCS_I2C_READ ((TCS_I2C_ADDR << 1) | I2C_READ)   // 0x53
#define TCS_I2C_WRITE ((TCS_I2C_ADDR << 1) | I2C_WRITE) // 0x52

/******************  Bit definition for TCS_COMMAND register  *******************/
#define TCS_COMMAND_CMD_Pos           (7U)
#define TCS_COMMAND_CMD_Msk           (0x1U << TCS_COMMAND_CMD_Pos)
#define TCS_COMMAND_CMD               TCS_COMMAND_CMD_Msk
#define TCS_COMMAND_TYPE_Pos          (5U)
#define TCS_COMMAND_TYPE_Msk          (0x3U << TCS_COMMAND_TYPE_Pos)
#define TCS_COMMAND_TYPE              TCS_COMMAND_TYPE_Msk
#define TCS_COMMAND_TYPE_0            (0x1U << TCS_COMMAND_TYPE_Pos)
#define TCS_COMMAND_TYPE_1            (0x2U << TCS_COMMAND_TYPE_Pos)
#define TCS_COMMAND_ADDR_Pos          (0U)
#define TCS_COMMAND_ADDR_Msk          (0x1FU << TCS_COMMAND_ADDR_Pos)
#define TCS_COMMAND_ADDR              TCS_COMMAND_ADDR_Msk
#define TCS_COMMAND_ADDR_0            (0x1U << TCS_COMMAND_ADDR_Pos)
#define TCS_COMMAND_ADDR_1            (0x2U << TCS_COMMAND_ADDR_Pos)
#define TCS_COMMAND_ADDR_2            (0x4U << TCS_COMMAND_ADDR_Pos)
#define TCS_COMMAND_ADDR_3            (0x8U << TCS_COMMAND_ADDR_Pos)
#define TCS_COMMAND_ADDR_4            (0x10U << TCS_COMMAND_ADDR_Pos)

#define TCS_COMMAND_TYPE_AUTO_INCR    (TCS_COMMAND_TYPE_0)
#define TCS_COMMAND_TYPE_SF           (TCS_COMMAND_TYPE_1 | TCS_COMMAND_TYPE_0)
#define TCS_COMMAND_SF_INTR_CLEAR     (TCS_COMMAND_CMD | TCS_COMMAND_SF | 0b00110)

/******************  TCS registers definition *******************/
#define TCS_ENABLE_Addr               (0x00U)
#define TCS_ENABLE                    (TCS_COMMAND_CMD | TCS_ENABLE_Addr)
#define TCS_ATIME_Addr                (0x01U)
#define TCS_ATIME                     (TCS_COMMAND_CMD | TCS_ATIME_Addr)
#define TCS_WTIME_Addr                (0x03U)
#define TCS_WTIME                     (TCS_COMMAND_CMD | TCS_WTIME_Addr)
#define TCS_AILTL_Addr                (0x04U)
#define TCS_AILTH_Addr                (0x05U)
#define TCS_AILT                      (TCS_COMMAND_CMD | TCS_COMMAND_TYPE_AUTO_INCR | TCS_AILTL_Addr)
#define TCS_AIHTL_Addr                (0x06U)
#define TCS_AIHTH_Addr                (0x07U)
#define TCS_AIHT                      (TCS_COMMAND_CMD | TCS_COMMAND_TYPE_AUTO_INCR | TCS_AIHTL_Addr)
#define TCS_PERS_Addr                 (0x0CU)
#define TCS_PERS                      (TCS_COMMAND_CMD | TCS_PERS_Addr)
#define TCS_CONFIG_Addr               (0x0DU)
#define TCS_CONFIG                    (TCS_COMMAND_CMD | TCS_CONFIG_Addr)
#define TCS_CONTROL_Addr              (0x0FU)
#define TCS_CONTROL                   (TCS_COMMAND_CMD | TCS_CONTROL_Addr)
#define TCS_ID_Addr                   (0x12U)
#define TCS_ID                        (TCS_COMMAND_CMD | TCS_ID_Addr)
#define TCS_STATUS_Addr               (0x13U)
#define TCS_STATUS                    (TCS_COMMAND_CMD | TCS_STATUS_Addr)
#define TCS_CDATAL_Addr               (0x14U)
#define TCS_CDATAH_Addr               (0x15U)
#define TCS_CDATA                     (TCS_COMMAND_CMD | TCS_COMMAND_TYPE_AUTO_INCR | TCS_CDATAL_Addr)
#define TCS_RDATAL_Addr               (0x16U)
#define TCS_RDATAH_Addr               (0x17U)
#define TCS_RDATA                     (TCS_COMMAND_CMD | TCS_COMMAND_TYPE_AUTO_INCR | TCS_RDATAL_Addr)
#define TCS_GDATAL_Addr               (0x18U)
#define TCS_GDATAH_Addr               (0x19U)
#define TCS_GDATA                     (TCS_COMMAND_CMD | TCS_COMMAND_TYPE_AUTO_INCR | TCS_GDATAL_Addr)
#define TCS_BDATAL_Addr               (0x1AU)
#define TCS_BDATAH_Addr               (0x1BU)
#define TCS_BDATA                     (TCS_COMMAND_CMD | TCS_COMMAND_TYPE_AUTO_INCR | TCS_BDATAL_Addr)

/******************  Bit definition for TCS_ENABLE register  *******************/
#define TCS_ENABLE_AIEN_Pos           (4U)
#define TCS_ENABLE_AIEN_Msk           (0x1U << TCS_ENABLE_AIEN_Pos)
#define TCS_ENABLE_AIEN               TCS_ENABLE_AIEN_Msk
#define TCS_ENABLE_WEN_Pos            (3U)
#define TCS_ENABLE_WEN_Msk            (0x1U << TCS_ENABLE_WEN_Pos)
#define TCS_ENABLE_WEN                TCS_ENABLE_WEN_Msk
#define TCS_ENABLE_AEN_Pos            (1U)
#define TCS_ENABLE_AEN_Msk            (0x1U << TCS_ENABLE_AEN_Pos)
#define TCS_ENABLE_AEN                TCS_ENABLE_AEN_Msk
#define TCS_ENABLE_PON_Pos            (0U)
#define TCS_ENABLE_PON_Msk            (0x1U << TCS_ENABLE_PON_Pos)
#define TCS_ENABLE_PON                TCS_ENABLE_PON_Msk

/******************  Bit definition for TCS_CONFIG register  *******************/
#define TCS_CONFIG_WLONG_Pos          (1U)
#define TCS_CONFIG_WLONG_Msk          (0x1U << TCS_CONFIG_WLONG_Pos)
#define TCS_CONFIG_WLONG              TCS_CONFIG_WLONG_Msk

/******************  Bit definition for TCS_CONTROL register  *******************/
#define TCS_CONTROL_AGAIN_Pos         (0U)
#define TCS_CONTROL_AGAIN_Msk         (0xFU << TCS_CONTROL_AGAIN_Pos)
#define TCS_CONTROL_AGAIN             TCS_CONTROL_AGAIN_Msk
#define TCS_CONTROL_AGAIN_0           (0x1U << TCS_CONTROL_AGAIN_Pos)
#define TCS_CONTROL_AGAIN_1           (0x2U << TCS_CONTROL_AGAIN_Pos)
#define TCS_CONTROL_AGAIN_2           (0x4U << TCS_CONTROL_AGAIN_Pos)
#define TCS_CONTROL_AGAIN_3           (0x8U << TCS_CONTROL_AGAIN_Pos)

/******************  Bit definition for TCS_STATUS register  *******************/
#define TCS_STATUS_AINT_Pos           (4U)
#define TCS_STATUS_AINT_Msk           (0x1U << TCS_STATUS_AINT_Pos)
#define TCS_STATUS_AINT               TCS_STATUS_AINT_Msk
#define TCS_STATUS_AVALID_Pos         (0U)
#define TCS_STATUS_AVALID_Msk         (0x1U << TCS_STATUS_AVALID_Pos)
#define TCS_STATUS_AVALID             TCS_STATUS_AVALID_Msk

/**
  * @brief  Get a measure from a powered-down TCS. Returns it back to sleep afterwards.
  * @param  rgbc pointer to a 16-bit uint array long enough for 4 values.
  *         The returned array follows the following format: [red, green, blue, clear]
  * @retval none
  * */
void tcsGetStandaloneRgbc(uint16_t *rgbc);

#endif /* INC_TCS_H_ */
