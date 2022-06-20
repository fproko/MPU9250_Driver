/**
 * \file
 * @brief     mpu9250.h
 * @details   Header para las funciones de ............
 * @author    Fernando Andres Prokopiuk
 * @date      06/06/2022
 * @version   1.0
 */

#ifndef MPU9250_HEADER_H_
#define MPU9250_HEADER_H_

/*****************************************************************************
 * Includes
 ****************************************************************************/

#include "stdint.h"
#include "stdio.h"

/*****************************************************************************
 * Macros
 ****************************************************************************/

#define CONFIG 0x1A

#define GYRO_CONFIG 0x1B
#define GYRO_FS_SEL_250DPS 0x00
#define GYRO_FS_SEL_500DPS 0x08
#define GYRO_FS_SEL_1000DPS 0x10
#define GYRO_FS_SEL_2000DPS 0x18

#define ACCEL_CONFIG 0x1C
#define ACCEL_FS_SEL_2G 0x00
#define ACCEL_FS_SEL_4G 0x08
#define ACCEL_FS_SEL_8G 0x10
#define ACCEL_FS_SEL_16G 0x18

#define FIFO_EN 0x23
#define FIFO_TEMP 0x80
#define FIFO_GYRO 0x70
#define FIFO_ACCEL 0x08

#define FIFO_COUNT 0x72
#define FIFO_R_W 0x74

#define INT_PIN_CFG 0x37

#define INT_ENABLE 0x38
#define INT_STATUS 0x3A
#define ACCEL_OUT 0x3B
#define TEMP_OUT 0x41
#define GYRO_OUT 0x43

#define WHO_AM_I 0x75

#define USER_CTRL 0x6A
#define I2C_IF_DIS 0x10

#define FIFO_MODE_ROLL_OVER 0x00

#define RX_BUFF_SIZE 21

#define DUMMY_BYTE 0xA5

#define READ_MSB 0x80

/*****************************************************************************
 * Typedef
 ****************************************************************************/

typedef enum accel_range
{
   ACCEL_RANGE_2G,
   ACCEL_RANGE_4G,
   ACCEL_RANGE_8G,
   ACCEL_RANGE_16G
} MPU9250_accel_range_t;

typedef enum gyro_range
{
   GYRO_RANGE_250DPS,
   GYRO_RANGE_500DPS,
   GYRO_RANGE_1000DPS,
   GYRO_RANGE_2000DPS
} MPU9250_gyro_range_t;

typedef struct control
{
   uint8_t rx_idx;
   uint8_t rx_buffer[RX_BUFF_SIZE];
} MPU9250_control_t;

typedef enum chip_select
{
   CS_ENABLE = 0,
   CS_DISABLE
} CS_state_t;

typedef enum interrupt_state
{
   I_ENABLE = 0,
   I_DISABLE
} int_state_t;

/**
 * typedefs de punteros a función para simplificar la lectura.
 */
typedef void (*cs_func_t)(CS_state_t);
typedef uint8_t (*spi_receive_func_t)(void);
typedef void (*spi_transmit_func_t)(uint8_t);

/**
 * Estructura con punteros a funciones para separar la capa más baja del driver.
 */
typedef struct
{
   /* Se convierte en void (*chip_select)(CS_state_t); */
   /* y finalmente en void (*CS_MSP430_port)(CS_state_t);*/
   cs_func_t chip_select;
   spi_receive_func_t spi_receive;
   spi_transmit_func_t spi_transmit;
} MPU9250_t;

/*****************************************************************************
 * Public functions declaration
 ****************************************************************************/

/* Prototipos de funciones que va a utilizar la aplicación */

uint8_t mpu9250_init(MPU9250_t);
void mpu9250_received_data(void);

#endif /* MPU9250_HEADER_H_ */
