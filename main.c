/**
 * \file
 * @brief     main.c
 * @details   Main file
 * @author    Fernando Andres Prokopiuk
 * @date      06/06/2022
 * @version   1.0
 */

/*****************************************************************************
 * Includes
 ****************************************************************************/

/* Biblioteca de driver de Texas Instrument */
#include "driverlib.h"

/* Header del port para el micro */
//#ifdef MSP430
#include "./inc/mpu9250_MSP430_port.h"
//#endif

/* Header del driver device */
#include "./inc/mpu9250.h"

/* SPI clock */
#define SPI_CLK 1000000 /* 1 MHz */

#define UART_send_str(x) UART_send_str_MSP430_port(x)
#define delay_ms(x) delay_ms_MSP430_port(x)

char pbuf[128]; // used by sprintf()

int MyPuts(int8_t *str);

#define printf(...)                 \
    ;                               \
    {                               \
        sprintf(pbuf, __VA_ARGS__); \
        MyPuts(pbuf);               \
    }

/*****************************************************************************
 * Functions
 ****************************************************************************/
/**
 * @brief Función que inicializa el hardware específico para este proyecto
 *
 */
void hw_init(void)
{
    /* Stop WDT */
    WDT_A_hold(WDT_A_BASE);

    /* Configuración de SPI */
    /* CS 2.3 */
    GPIO_setOutputHighOnPin(GPIO_PORT_P2, GPIO_PIN3);
    GPIO_setAsOutputPin(GPIO_PORT_P2, GPIO_PIN3);

    /* UCB0SIMO 3.0 | UCB0CLK 3.2 */
    GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P3, GPIO_PIN0 + GPIO_PIN2);
    /* UCB0SOMI 3.1 */
    GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P3, GPIO_PIN1);

    /* Estructura de parametros de configuración */
    USCI_B_SPI_initMasterParam SPI_param = {0};
    SPI_param.selectClockSource = USCI_B_SPI_CLOCKSOURCE_SMCLK;
    SPI_param.clockSourceFrequency = UCS_getSMCLK();
    SPI_param.desiredSpiClock = SPI_CLK;
    SPI_param.msbFirst = USCI_B_SPI_MSB_FIRST;
    SPI_param.clockPhase = USCI_B_SPI_PHASE_DATA_CHANGED_ONFIRST_CAPTURED_ON_NEXT;
    SPI_param.clockPolarity = USCI_B_SPI_CLOCKPOLARITY_INACTIVITY_HIGH;

    /* Se inicializa SPI y se verifica bandera de error */
    if (STATUS_FAIL == USCI_B_SPI_initMaster(SPI, &SPI_param))
    {
        return;
    }

    /* Se habilita módulo SPI */
    USCI_B_SPI_enable(SPI);

    /* Configuración de UART */
    /* USCI_A1 TXD */
    GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P4, GPIO_PIN4);
    /* USCI_A1 RXD */
    GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P4, GPIO_PIN5);
    /* Estructura de parametros de configuración */
    USCI_A_UART_initParam UART_param = {0};
    /* Baudrate = 115200, clock frec = 1.048MHz, UCBRx = 9, UCBRFx = 0, UCBRSx = 1,
     UCOS16 = 0 */
    UART_param.selectClockSource = USCI_A_UART_CLOCKSOURCE_SMCLK;
    UART_param.clockPrescalar = 9;
    UART_param.firstModReg = 0;
    UART_param.secondModReg = 1;
    UART_param.parity = USCI_A_UART_NO_PARITY;
    UART_param.msborLsbFirst = USCI_A_UART_LSB_FIRST;
    UART_param.numberofStopBits = USCI_A_UART_ONE_STOP_BIT;
    UART_param.uartMode = USCI_A_UART_MODE;
    UART_param.overSampling = USCI_A_UART_LOW_FREQUENCY_BAUDRATE_GENERATION;

    /* Se inicializa UART y se verifica bandera de error */
    if (STATUS_FAIL == USCI_A_UART_init(UART, &UART_param))
    {
        return;
    }

    /* Habilitación de UART */
    USCI_A_UART_enable(UART);

    /* Habilitación de interrupciones */
    __enable_interrupt();
}

/**
 * @brief Función principal de la aplicación.
 *
 */
void main(void)
{
    /* Se inicializa el hardware específico para este proyecto */
    hw_init();
    UART_send_str("Hardware inicializado\r\n");

    /* Se instancia estructura de driver definida en mpu9250.h */
    MPU9250_t MPU9250_config_s;

/* Se carga en la estructura de driver las distintas funciones
 * que son específicas de la arquitectura */
#ifdef MSP430
    UART_send_str("Cargando estructura de driver\r\n");
    MPU9250_config_s.chip_select = CS_MSP430_port;
    MPU9250_config_s.spi_receive = SPI_receive_MSP430_port;
    MPU9250_config_s.spi_transmit = SPI_transmit_MSP430_port;
    MPU9250_config_s.delay_ms = delay_ms_MSP430_port;
#endif

    uint8_t status;

    /* Se inicializa el driver y se pasa la estructura de driver */
    UART_send_str("Inicializando MPU9250...\r\n");
    status = mpu9250_init(MPU9250_config_s);

    if (status == 1)
    {
        UART_send_str("Error de inicialización de MPU9250\r\n");
        UART_send_str("Se detiene el programa!\r\n");
        while (1)
            ;
    }

    UART_send_str("MPU9250 incializado\r\n");

    while (1)
    {
        /* Se lee el sensor y se guardan los datos en estructura de control */
        mpu9250_get_data();

        /* Se imprimen los datos crudos */
        printf("Acelerometro:   (%d, %d, %d)   \r\n",
               mpu9250_get_accel_x(),
               mpu9250_get_accel_y(),
               mpu9250_get_accel_z());

        printf("Giroscopo:      (%d, %d, %d)   \r\n",
               mpu9250_get_gyro_x(),
               mpu9250_get_gyro_y(),
               mpu9250_get_gyro_z());

        printf("Magnetometro:   (%d, %d, %d)   \r\n",
               mpu9250_get_mag_x(),
               mpu9250_get_mag_y(),
               mpu9250_get_mag_z());

        delay_ms(200);
    }
}

/**
 * @brief Función para colocar salida de printf por UART.
 *
 * @param str
 * @return int
 */
int MyPuts(int8_t *str)
{
    register char i = 0;
    while (str[i])
    {
        while (!(UCA1IFG & UCTXIFG))
            ;
        UCA1TXBUF = str[i++];
    }
    // return number of chars sent
    return (i);
}
