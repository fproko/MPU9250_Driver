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

/* Mapa de registros de MPU9250 para Giróscopo y Acelerómetro */
#define SMPLRT_DIV 0x19

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

#define ACCEL_CONFIG2 0x1D
#define DLPF_184 0x01
#define DLPF_92 0x02
#define DLPF_41 0x03
#define DLPF_20 0x04
#define DLPF_10 0x05
#define DLPF_5 0x06

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

#define EXT_SENS_DATA_00 0x49

#define WHO_AM_I 0x75

#define USER_CTRL 0x6A
#define I2C_IF_DIS 0x10
#define I2C_MST_EN 0x20

#define I2C_MST_CTRL 0x24
#define I2C_MST_CLK 0x0D

#define FIFO_MODE_ROLL_OVER 0x00

#define RX_BUFF_SIZE 21

#define DUMMY_BYTE 0xA5

#define READ_MSB 0x80

#define PWR_MGMT_1 0x6B
#define CLKSEL_PLL 0x01
#define H_RESET 0x80

#define PWR_MGMT_2 0x6C
#define SENSORS_EN 0x00

/* Mapa de registros de Magnetómetro AK8963 */
#define AK8963_HXL 0x03

#define AK8963_I2C_ADDR 0x0C

#define AK8963_CNTL1 0X0A
#define AK8963_PWR_DOWN 0X00
#define AK8963_FUSE_ROM_ACCESS 0x0F
#define AK8963_CNT_MEAS_1 0x12
#define AK8963_CNT_MEAS_2 0x16

#define AK8963_CNTL2 0X0B
#define AK8963_SRST 0x01

#define AK8963_ASA 0x10

#define AK8963_WIA 0x00

#define I2C_SLV0_ADDR 0x25

#define I2C_SLV0_REG 0x26

#define I2C_SLV0_CTRL 0x27
#define I2C_SLV0_EN 0x80

#define I2C_SLV0_DO 0x63

#define I2C_MST_STATUS 0x36

#define SRD 0x00

#define AXES_NUMBER 3
#define AXE_X 0
#define AXE_Y 1
#define AXE_Z 2

/*****************************************************************************
 * Typedef
 ****************************************************************************/

typedef struct control
{
    uint8_t rx_buff[RX_BUFF_SIZE];
    int16_t acc_data[AXES_NUMBER];
    int16_t gyro_data[AXES_NUMBER];
    int16_t mag_data[AXES_NUMBER];
    uint8_t mag_adjust[AXES_NUMBER];
} MPU9250_control_t;

typedef enum chip_select
{
    CS_ENABLE = 0,
    CS_DISABLE
} CS_state_t;

/**
 * typedefs de punteros a función para simplificar la lectura.
 */
typedef void (*cs_func_t)(CS_state_t);
typedef uint8_t (*spi_receive_func_t)(void);
typedef void (*spi_transmit_func_t)(uint8_t);
typedef void (*delay_ms_func_t)(uint16_t);

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
    delay_ms_func_t delay_ms;
} MPU9250_t;

/*****************************************************************************
 * Public functions declaration
 ****************************************************************************/

/* Prototipos de funciones que va a utilizar la aplicación */

uint8_t mpu9250_who_am_i(void);
uint8_t ak8963_who_am_i(void);
void mpu9250_set_accel_range(uint8_t range);
void mpu9250_set_gyro_range(uint8_t range);
void mpu9250_set_DLPF_BW(uint8_t bandwidth);
uint8_t mpu9250_init(MPU9250_t);
void mpu9250_get_data(void);
int16_t mpu9250_get_accel_x(void);
int16_t mpu9250_get_accel_y(void);
int16_t mpu9250_get_accel_z(void);
int16_t mpu9250_get_gyro_x(void);
int16_t mpu9250_get_gyro_y(void);
int16_t mpu9250_get_gyro_z(void);
int16_t mpu9250_get_mag_x(void);
int16_t mpu9250_get_mag_y(void);
int16_t mpu9250_get_mag_z(void);
#endif /* MPU9250_HEADER_H_ */
