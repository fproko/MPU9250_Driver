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
static MPU9250_control_t MPU9250_ctrl_s;

/*****************************************************************************
 * Public types/enumerations/variables
 ****************************************************************************/

/*****************************************************************************
 * Private functions declaration
 ****************************************************************************/

static uint8_t mpu9250_read_reg(uint8_t reg_addr);
static void mpu9250_read_regs(uint8_t reg_addr, uint8_t qty, uint8_t *pBuffer);
static void mpu9250_write_reg(uint8_t reg_addr, uint8_t data);
static void ak8963_write_reg(uint8_t reg_addr, uint8_t data);
static void ak8963_read_regs(uint8_t reg_addr, uint8_t qty, uint8_t *pBuffer);

/*****************************************************************************
 * Private functions definition
 ****************************************************************************/

/**
 * @brief Lee el registro recibido como parámetro.
 *
 * @param reg_addr Registro a leer.
 * @return rx_aux Registro leido.
 *
 */
static uint8_t mpu9250_read_reg(uint8_t reg_addr)
{
    uint8_t rx_aux;

    /* Habilitación de chip */
    MPU9250_s.chip_select(CS_ENABLE);

    /* Se envia dirección de registro a leer */
    /* Procedimiento de lectura debe setear el MSB */
    MPU9250_s.spi_transmit(reg_addr | READ_MSB);

    /* Se copia del RXBUFF el dato recibido */
    rx_aux = MPU9250_s.spi_receive();

    /* Deshabilitación de chip */
    MPU9250_s.chip_select(CS_DISABLE);

    return rx_aux;
}

/**
 * @brief Lee varios registros a partir de la dirección base recibida como parámetro.
 *
 * @param reg_addr Dirección base.
 * @param qty Cantidad de registros a leer.
 * @param pBuffer Puntero a buffer donde se guardan los datos leidos.
 */
static void mpu9250_read_regs(uint8_t reg_addr, uint8_t qty, uint8_t *pBuffer)
{
    uint8_t i = 0;

    /* Habilitación de chip */
    MPU9250_s.chip_select(CS_ENABLE);

    /* Se envia dirección de registro */
    MPU9250_s.spi_transmit(reg_addr | READ_MSB);

    while (i < qty)
    {
        /* Se guarda dato leido en buffer */
        pBuffer[i++] = MPU9250_s.spi_receive();
    }

    /* Deshabilitación de chip */
    MPU9250_s.chip_select(CS_DISABLE);
}

/**
 * @brief Escribe en el registro recibido como parámetro.
 *
 * @param reg_addr Regsitro a escribir.
 * @param data Dato a escribir.
 */
static void mpu9250_write_reg(uint8_t reg_addr, uint8_t data)
{
    /* Habilitación de chip */
    MPU9250_s.chip_select(CS_ENABLE);

    /* Se envia registro */
    MPU9250_s.spi_transmit(reg_addr);

    /* Se envian datos */
    MPU9250_s.spi_transmit(data);

    /* Deshabilitación de chip */
    MPU9250_s.chip_select(CS_DISABLE);
}

/**
 * @brief Escribe los datos recibidos como parámetro en el registro de AK8963 recibido como parámetro.
 *
 * @param reg_addr Dirección de registro a escribir.
 * @param data Datos a escribir.
 */
static void ak8963_write_reg(uint8_t reg_addr, uint8_t data)
{
    /* Se setea slave 0 al AK8963 y se setea para escritura */
    mpu9250_write_reg(I2C_SLV0_ADDR, AK8963_I2C_ADDR);

    /* Se setea registro desde donde comienza la transmisión de datos */
    mpu9250_write_reg(I2C_SLV0_REG, reg_addr);

    /* Se guarda la data a escribir */
    mpu9250_write_reg(I2C_SLV0_DO, data);

    /* Se habilita I2C y se envia 1 byte */
    mpu9250_write_reg(I2C_SLV0_CTRL, I2C_SLV0_EN | (uint8_t)1);
}

/**
 * @brief Lee la catidad de registros solicitados de AK8963.
 *
 * @param reg_addr Direccíon base de registros a leer del AK8963.
 * @param qty Cantidad de bytes a leer.
 * @param pBuffer Puntero a buffer donde se guardan los datos leidos.
 */
static void ak8963_read_regs(uint8_t reg_addr, uint8_t qty, uint8_t *pBuffer)
{
    /* Se setea slave 0 al AK8963 y se setea para lectura */
    mpu9250_write_reg(I2C_SLV0_ADDR, AK8963_I2C_ADDR | READ_MSB);

    /* Se setea registro desde donde comienza la lectura de datos */
    mpu9250_write_reg(I2C_SLV0_REG, reg_addr);

    /* Se habilita I2C y se solicitan la cantidad de bytes (I2C_SLV0_LENG) */
    mpu9250_write_reg(I2C_SLV0_CTRL, I2C_SLV0_EN | qty);

    /* Se espera a que los registros se completen */
    MPU9250_s.delay_ms(100);

    /* Se leen los bytes de el registro de sensor externo del MPU9250 */
    mpu9250_read_regs(EXT_SENS_DATA_00, qty, pBuffer);
}

/*****************************************************************************
 * Public functions definition
 ****************************************************************************/

/**
 * @brief Función que lee y retorna valor de registro WHO_AM_I.
 *
 * @return uint8_t Valor leido.
 */
uint8_t mpu9250_who_am_i(void)
{
    /* Se lee y retorna valor de registro WHO_AM_I. Se espera un valor 0x71. */
    return mpu9250_read_reg(WHO_AM_I);
}

/**
 * @brief Función que lee y retorna valor de registro WAI.
 *
 * @return uint8_t Valor leido.
 */
uint8_t ak8963_who_am_i(void)
{
    uint8_t who;

    /* Se lee y retorna valor de registro AK8963_WIA. Se espera un valor 0x48. */
    ak8963_read_regs(AK8963_WIA, 1, &who);

    return who;
}

/**
 * @brief Función que setea el rango de aceleración recibido como parámetro.
 *
 * @param range Rango de aceleración deseado.
 */
void mpu9250_set_accel_range(uint8_t range)
{
    mpu9250_write_reg(ACCEL_CONFIG, range);
}

/**
 * @brief Función que setea el rango de aceleración recibido como parámetro.
 *
 * @param range Rango de giro deseado.
 */
void mpu9250_set_gyro_range(uint8_t range)
{
    mpu9250_write_reg(GYRO_CONFIG, range);
}

/**
 * @brief Función que setea el ancho de banda del filtro DLPF.
 *
 * @param range Ancho de banda deseado.
 */
void mpu9250_set_DLPF_BW(uint8_t bandwidth)
{
    mpu9250_write_reg(ACCEL_CONFIG2, bandwidth);
    mpu9250_write_reg(CONFIG, bandwidth);
}

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
    MPU9250_s.chip_select = config.chip_select;
    MPU9250_s.spi_receive = config.spi_receive;
    MPU9250_s.spi_transmit = config.spi_transmit;
    MPU9250_s.delay_ms = config.delay_ms;

    /* Se setea fuente de CLK para el girómetro */
    mpu9250_write_reg(PWR_MGMT_1, CLKSEL_PLL);

    /* Se habilita I2C modo master */
    mpu9250_write_reg(USER_CTRL, I2C_MST_EN);

    /* Se setea la velocidad del bus I2C en 400 kHz */
    mpu9250_write_reg(I2C_MST_CTRL, I2C_MST_CLK);

    /* Se cambia AK8963 a modo apagado */
    ak8963_write_reg(AK8963_CNTL1, AK8963_PWR_DOWN);

    /* Se resetea MPU9250 */
    mpu9250_write_reg(PWR_MGMT_1, H_RESET);

    /* Se espera que se encienda nuevamente */
    MPU9250_s.delay_ms(10);

    /* Se resetea AK8963 */
    ak8963_write_reg(AK8963_CNTL2, AK8963_SRST);

    /* Se setea fuente de CLK para el girómetro */
    mpu9250_write_reg(PWR_MGMT_1, CLKSEL_PLL);

    /* Se chequea el valor del registro WHO_AM_I. Se espera un valor de 0x71 */
    if (mpu9250_who_am_i() != 0x71)
    {
        return 1;
    }

    /* Se habilita el acelerómetro y el giro */
    mpu9250_write_reg(PWR_MGMT_2, SENSORS_EN);

    /* Se setea rango de escala de aceleración */
    mpu9250_set_accel_range(ACCEL_FS_SEL_16G);

    /* Se setea rango de escala de gyro */
    mpu9250_set_gyro_range(GYRO_FS_SEL_250DPS);

    /* Se setea el ancho de banda del filtro DLPF en 184 Hz*/
    mpu9250_set_DLPF_BW(DLPF_184);

    /* Se setea el Sample Rate Divider en 0 */
    mpu9250_write_reg(SMPLRT_DIV, SRD);

    /* Se habilita I2C modo master */
    mpu9250_write_reg(USER_CTRL, I2C_MST_EN);

    /* Se setea la velocidad del bus I2C en 400 kHz */
    mpu9250_write_reg(I2C_MST_CTRL, I2C_MST_CLK);

    /* Se chequea el valor del registro WHO_AM_I del AK8963 */
    if (ak8963_who_am_i() != 0x48)
    {
        return 1;
    }

    /* Se cambia AK8963 a modo apagado */
    ak8963_write_reg(AK8963_CNTL1, AK8963_PWR_DOWN);

    /* Se espera que AK8963 cambie de modo */
    MPU9250_s.delay_ms(100);

    /* Se setea AK8963 en modo de acceso FUSE ROM*/
    ak8963_write_reg(AK8963_CNTL1, AK8963_FUSE_ROM_ACCESS);

    /* Se espera que AK8963 cambie de modo */
    MPU9250_s.delay_ms(100);

    /* Se leen los registros ASA del AK8963 y se calculan los factores de escala del magnetómetro */
    ak8963_read_regs(AK8963_ASA, 3, MPU9250_ctrl_s.mag_adjust);

    /* Se cambia AK8963 a modo apagado */
    ak8963_write_reg(AK8963_CNTL1, AK8963_PWR_DOWN);

    /* Se espera que AK8963 cambie de modo */
    MPU9250_s.delay_ms(100);

    /* Se setea AK8963 en 16 bit de resolución de salida y modo de medición continuo 2 */
    ak8963_write_reg(AK8963_CNTL1, AK8963_CNT_MEAS_2);

    /* Se espera que AK8963 cambie de modo */
    MPU9250_s.delay_ms(100);

    /* Se setea fuente de CLK para el girómetro */
    mpu9250_write_reg(PWR_MGMT_1, CLKSEL_PLL);

    /* Se optiene 7 bytes de datos del AK8963 al sample rate especificado */
    ak8963_read_regs(AK8963_HXL, 7, MPU9250_ctrl_s.rx_buff);

    /* Se inicializó correctamente */
    return 0;
}

/**
 * @brief Se lee el sensor y se guardan los datos en estructura de control
 *
 */
void mpu9250_get_data(void)
{
    /* Se obtienen 21 bytes datos del MPU9250 */
    mpu9250_read_regs(ACCEL_OUT, 21, MPU9250_ctrl_s.rx_buff);

    /* Se combinan partes altas y bajas de los registros en 1 solo registro de 16 bits */
    MPU9250_ctrl_s.acc_data[AXE_X] = (((int16_t)MPU9250_ctrl_s.rx_buff[0]) << 8) | MPU9250_ctrl_s.rx_buff[1];
    MPU9250_ctrl_s.acc_data[AXE_Y] = (((int16_t)MPU9250_ctrl_s.rx_buff[2]) << 8) | MPU9250_ctrl_s.rx_buff[3];
    MPU9250_ctrl_s.acc_data[AXE_Z] = (((int16_t)MPU9250_ctrl_s.rx_buff[4]) << 8) | MPU9250_ctrl_s.rx_buff[5];

    MPU9250_ctrl_s.gyro_data[AXE_X] = (((int16_t)MPU9250_ctrl_s.rx_buff[8]) << 8) | MPU9250_ctrl_s.rx_buff[9];
    MPU9250_ctrl_s.gyro_data[AXE_Y] = (((int16_t)MPU9250_ctrl_s.rx_buff[10]) << 8) | MPU9250_ctrl_s.rx_buff[11];
    MPU9250_ctrl_s.gyro_data[AXE_Z] = (((int16_t)MPU9250_ctrl_s.rx_buff[12]) << 8) | MPU9250_ctrl_s.rx_buff[13];

    int16_t magx = (((int16_t)MPU9250_ctrl_s.rx_buff[15]) << 8) | MPU9250_ctrl_s.rx_buff[14];
    int16_t magy = (((int16_t)MPU9250_ctrl_s.rx_buff[17]) << 8) | MPU9250_ctrl_s.rx_buff[16];
    int16_t magz = (((int16_t)MPU9250_ctrl_s.rx_buff[19]) << 8) | MPU9250_ctrl_s.rx_buff[18];

    MPU9250_ctrl_s.mag_data[AXE_X] = (int16_t)((float)magx * ((float)(MPU9250_ctrl_s.mag_adjust[AXE_X] - 128) / 256.0f + 1.0f));
    MPU9250_ctrl_s.mag_data[AXE_Y] = (int16_t)((float)magy * ((float)(MPU9250_ctrl_s.mag_adjust[AXE_Y] - 128) / 256.0f + 1.0f));
    MPU9250_ctrl_s.mag_data[AXE_Z] = (int16_t)((float)magz * ((float)(MPU9250_ctrl_s.mag_adjust[AXE_Z] - 128) / 256.0f + 1.0f));
}

/**
 * @brief Función que obtiene la medición de aceleración en el eje x.
 *
 * @return int16_t Dato de aceleración en eje x RAW.
 */
int16_t mpu9250_get_accel_x(void)
{
    return MPU9250_ctrl_s.acc_data[AXE_X];
}

/**
 * @brief Función que obtiene la medición de aceleración en el eje y.
 *
 * @return int16_t Dato de aceleración en eje y RAW.
 */
int16_t mpu9250_get_accel_y(void)
{
    return MPU9250_ctrl_s.acc_data[AXE_Y];
}

/**
 * @brief Función que obtiene la medición de aceleración en el eje z.
 *
 * @return int16_t Dato de aceleración en eje y RAW.
 */
int16_t mpu9250_get_accel_z(void)
{
    return MPU9250_ctrl_s.acc_data[AXE_Z];
}

/**
 * @brief Función que obtiene la medición del giróscopo en el eje x.
 *
 * @return int16_t Dato del giróscopo en eje x RAW.
 */
int16_t mpu9250_get_gyro_x(void)
{
    return MPU9250_ctrl_s.gyro_data[AXE_X];
}

/**
 * @brief Función que obtiene la medición del giróscopo en el eje y.
 *
 * @return int16_t Dato del giróscopo en eje y RAW.
 */
int16_t mpu9250_get_gyro_y(void)
{
    return MPU9250_ctrl_s.gyro_data[AXE_Y];
}

/**
 * @brief Función que obtiene la medición del giróscopo en el eje z.
 *
 * @return int16_t Dato del giróscopo en eje z RAW.
 */
int16_t mpu9250_get_gyro_z(void)
{
    return MPU9250_ctrl_s.gyro_data[AXE_Z];
}

/**
 * @brief Función que obtiene la medición del magentómetro en el eje x.
 *
 * @return int16_t Dato del magnetómetro en eje x RAW.
 */
int16_t mpu9250_get_mag_x(void)
{
    return MPU9250_ctrl_s.mag_data[AXE_X];
}

/**
 * @brief Función que obtiene la medición del magentómetro en el eje y.
 *
 * @return int16_t Dato del magnetómetro en eje y RAW.
 */
int16_t mpu9250_get_mag_y(void)
{
    return MPU9250_ctrl_s.mag_data[AXE_Y];
}

/**
 * @brief Función que obtiene la medición del magentómetro en el eje z.
 *
 * @return int16_t Dato del magnetómetro en eje z RAW.
 */
int16_t mpu9250_get_mag_z(void)
{
    return MPU9250_ctrl_s.mag_data[AXE_Z];
}
