/**
 * \file
 * @brief     mpu9250_MSP430_port.c
 * @details   Implementaciones de la funciones de ............
 * @author    Fernando Andres Prokopiuk
 * @date      06/06/2022
 * @version   1.0
 */

/*****************************************************************************
 * Includes
 ****************************************************************************/

#include "./inc/mpu9250_MSP430_port.h"

/*****************************************************************************
 * Private types/enumerations/variables
 ****************************************************************************/

/*****************************************************************************
 * Public types/enumerations/variables
 ****************************************************************************/

/*****************************************************************************
 * Private functions declaration
 ****************************************************************************/

/*****************************************************************************
 * Private functions definition
 ****************************************************************************/

/*****************************************************************************
 * Public functions definition
 ****************************************************************************/

/**
 *  @brief Función para habilitar el chip MPU9250.
 *
 *  @details Esta función es específica para el hardware utilizado.
 *
 *  @param   state  Determina la accion a ser tomada con el pin CS.
 */
void CS_MSP430_port(CS_state_t state)
{
    switch (state)
    {
    case CS_ENABLE:
        GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN3);
        break;

    case CS_DISABLE:
        GPIO_setOutputHighOnPin(GPIO_PORT_P2, GPIO_PIN3);
        break;

    default:;
    }
}

/**
 *  @brief Funcion para recibir un dato mediante SPI.
 *
 *  @details Esta funcion es especifica para el hardware utilizado.
 *
 *  @return Datos leidos.
 */
uint8_t SPI_receive_MSP430_port(void)
{
    /* Se transmite dummy byte para proover el clock */
    SPI_transmit_MSP430_port(DUMMY_BYTE);

    /* Se espera dato en RXBUF */
    while (!SPI_get_interrupt_status(SPI, SPI_RECEIVE_INTERRUPT))
        ;

    /* Se retorna dato recibido */
    return SPI_receive(SPI);
}

/**
 *  @brief Funcion para enviar un dato mediante SPI.
 *
 *  @details Esta función es especifica para el hardware utilizado.
 *
 *  @param tx_data Datos a enviar.
 *  @return  None.
 */
void SPI_transmit_MSP430_port(uint8_t tx_data)
{
    /* Se espera a que se libere TXBUF */
    while (!SPI_get_interrupt_status(SPI, SPI_TRANSMIT_INTERRUPT))
        ;

    /* Se envian datos */
    SPI_transmit(SPI, tx_data);
}

/**
 * @brief Función para transmitir un string por UART
 *
 * @param[in] str String a enviar.
 */
void UART_send_str_MSP430_port(uint8_t *str)
{
    while (*str != '\0')
    {
        USCI_A_UART_transmitData(UART, (uint8_t)*str);
        str++;
    }
}

/**
 * @brief Función para generar un delay.
 *
 * @param[in] ms Duración del delay en milisegundos.
 */
void delay_ms_MSP430_port(uint16_t ms)
{
    while (ms)
    {
        __delay_cycles(1052);
        ms--;
    }
}
