/**
 * \file
 * @brief     mpu9250.c
 * @details   Implementaciones de la funciones de ............
 * @author    Fernando Andres Prokopiuk
 * @date      06/06/2022
 * @version   1.0
 */

/*****************************************************************************
 * Includes
 ****************************************************************************/
#include "./inc/mpu9250.h"

/*****************************************************************************
 * Private types/enumerations/variables
 ****************************************************************************/

/**
 * Estructura interna al driver que no es visible al usuario.
 * Se utiliza para cargar las funciones definidas especificamente como port para
 * el hardware especifico.
 */
static MPU9250_t MPU9250_s;

/**
 * Estructura interna al driver que no es visible al usuario.
 * Se utiliza para control del chip.
 */
static MPU9250_control_t MPU9250_control_s;

/*****************************************************************************
 * Public types/enumerations/variables
 ****************************************************************************/

/*****************************************************************************
 * Private functions declaration
 ****************************************************************************/

static uint8_t mpu9250_read_reg(uint8_t reg);
static void mpu9250_read_regs(uint8_t reg, uint8_t qty);
static void mpu9250_write_reg(uint8_t reg, uint8_t data);

/*****************************************************************************
 * Private functions definition
 ****************************************************************************/

/**
 * @brief Lee el registro recibido como parámetro.
 *
 * @details Se envia primero que dirección de registro queremos leer
 * y luego se envia un dummy byte porque las operaciones de recepción
 * y transmision operan de forma concurrente.
 *
 * @param reg Registro a leer.
 * @return uint8_t Registro leido.
 *
 */
static uint8_t mpu9250_read_reg(uint8_t reg)
{
  /* Procedimiento de lectura debe setear el MSB. */
  reg |= READ_MSB;

  /* Habilitación de chip. */
  MPU9250_s.chip_select(CS_ENABLE);

  /* Se envia dirección de registro. */
  MPU9250_s.spi_transmit(reg);

  MPU9250_s.spi_transmit(DUMMY_BYTE);

  /* Dehabilitación de chip. */
  MPU9250_s.chip_select(CS_DISABLE);

  return MPU9250_control_s.rx_buffer[MPU9250_control_s.rx_idx - 1];
}

/**
 * @brief Lee varios registros a partir de la dirección base recibida como parámetro.
 *
 * @param reg Dirección base.
 * @param qty Cantidad de registros a leer.
 */
static void mpu9250_read_regs(uint8_t reg, uint8_t qty)
{
  /* Procedimiento de lectura debe setear el MSB. */
  reg |= READ_MSB;

  /* Habilitación de chip. */
  MPU9250_s.chip_select(CS_ENABLE);

  /* Se envia dirección de registro. */
  MPU9250_s.spi_transmit(reg);

  /* Se envia dirección de registro. */
  while (qty > 0)
  {
    MPU9250_s.spi_transmit(DUMMY_BYTE);
    qty--;
  }

  /* Dehabilitación de chip. */
  MPU9250_s.chip_select(CS_DISABLE);
}

/**
 * @brief Escribe en el registro recibido como parámetro.
 *
 * @param reg Regsitro a escribir.
 * @param data Dato a escribir.
 */
static void mpu9250_write_reg(uint8_t reg, uint8_t data)
{
  /* Habilitación de chip. */
  MPU9250_s.chip_select(CS_ENABLE);

  MPU9250_s.spi_transmit(reg);

  MPU9250_s.spi_transmit(data);

  /* Dehabilitación de chip. */
  MPU9250_s.chip_select(CS_DISABLE);
}

/*****************************************************************************
 * Public functions definition
 ****************************************************************************/

/**
 * @brief Función de inicializacion del driver MPU9250.
 *
 * @details
 * Se copian los punteros a funciones pasados por argumentos a una estructura interna
 * del driver, la cual no puede ser accedida por el resto del programa.
 *
 * @param      config  Estructura de configuracion para el driver que proviene del main.
 * @return     None.
 * @warning    En esta version no hay assert alguno sobre si los punteros tienen un valor
 * distinto de NULL.
 */
uint8_t mpu9250_init(MPU9250_t config)
{
  uint8_t tmp;

  MPU9250_s.chip_select = config.chip_select;
  MPU9250_s.spi_receive = config.spi_receive;
  MPU9250_s.spi_transmit = config.spi_transmit;

  // When using SPI, the I2C interface should be disabled by setting the I2C_IF_DIS[4] configuration bit
  // from register USER_CTRL.

  tmp = mpu9250_read_reg(WHO_AM_I);

  return 0;
}

/**
 * @brief Función para guardar el dato recibido en el buffer. Es llamada por la ISR del módulo SPI.
 *
 */
void mpu9250_received_data(void)
{
  MPU9250_control_s.rx_buffer[MPU9250_control_s.rx_idx] = MPU9250_s.spi_receive();

  MPU9250_control_s.rx_idx++;

  if (MPU9250_control_s.rx_idx == RX_BUFF_SIZE)
  {
    /* Memoria llena, se hace roll over */
    MPU9250_control_s.rx_idx = 0;
  }
}
