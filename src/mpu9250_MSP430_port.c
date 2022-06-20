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
 *  @return  None.
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
 *  @brief Funcion para habilitar/deshabilitar interrupción para SPI.
 *
 *  @details Esta funcion es especifica para el hardware utilizado.
 *
 */
void SPI_Interrupt_state_MSP430_port(int_state_t state, int_t mask)
{
    USCI_B_SPI_clearInterrupt(SPI, mask);

    switch (state)
    {
    case I_ENABLE:
        USCI_B_SPI_enableInterrupt(SPI, mask);
        break;

    case I_DISABLE:
        USCI_B_SPI_disableInterrupt(SPI, mask);
        break;

    default:
        break;
    }
}

/**
 *  @brief Funcion para deshabilitar interrupción de RX para SPI.
 *
 *  @details Esta funcion es especifica para el hardware utilizado.
 *
 */
void SPI_Disable_RX_Interrupt_MSP430_port(void)
{
    USCI_B_SPI_disableInterrupt(SPI, USCI_B_SPI_RECEIVE_INTERRUPT);

    USCI_B_SPI_clearInterrupt(SPI, USCI_B_SPI_RECEIVE_INTERRUPT);
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
    /* Buffer vacio? */
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
void UART_send_str_MSP430_port(const uint8_t *str)
{
    while (*str != '\0')
    {
        USCI_A_UART_transmitData(UART, (uint8_t)*str);
        str++;
    }
}

/*****************************************************************************
 * USCI_B0 interrupt vector service routine.
 ****************************************************************************/
#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector = USCI_B0_VECTOR
__interrupt
#elif defined(__GNUC__)
__attribute__((interrupt(USCI_B0_VECTOR)))
#endif

    void
    USCI_B0_ISR(void)
{

    switch (__even_in_range(UCB0IV, 4))
    {
    /* Vector 2: RXIFG */
    case 2:
        mpu9250_received_data();
        break;
    default:
        break;
    }
}
