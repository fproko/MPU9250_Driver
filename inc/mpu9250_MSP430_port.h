/**
 * \file
 * @brief     mpu9250_MSP430_port.h
 * @details   Header para las funciones de ............
 * @author    Fernando Andres Prokopiuk
 * @date      06/06/2022
 * @version   1.0
 */

#ifndef MPU9250_MSP430_PORT_HEADER_H_
#define MPU9250_MSP430_PORT_HEADER_H_

/*****************************************************************************
 * Includes
 ****************************************************************************/

#include "mpu9250.h"

/* Biblioteca de driver de Texas Instrument. */
#include "driverlib.h"

#define UART USCI_A1_BASE
#define SPI USCI_B0_BASE

#define SPI_get_interrupt_status(x, y) USCI_B_SPI_getInterruptStatus(x, y)
#define SPI_TRANSMIT_INTERRUPT USCI_B_SPI_TRANSMIT_INTERRUPT
#define SPI_transmit(x, y) USCI_B_SPI_transmitData(x, y)
#define SPI_receive(x) USCI_B_SPI_receiveData(x)

typedef enum interrupt_type
{
    IRX = USCI_B_SPI_RECEIVE_INTERRUPT,
    ITX = USCI_B_SPI_TRANSMIT_INTERRUPT
} interrupt_state_t;

/*****************************************************************************
 * Private types/enumerations/variables
 ****************************************************************************/

/*****************************************************************************
 * Public types/enumerations/variables
 ****************************************************************************/

/*****************************************************************************
 * Public functions declaration
 ****************************************************************************/
void CS_MSP430_port(CS_state_t state);
uint8_t SPI_receive_MSP430_port(void);
void SPI_transmit_MSP430_port(uint8_t tx_data);
void UART_send_str_MSP430_port(const uint8_t *str);

#endif /* MPU9250_MSP430_PORT_HEADER_H_ */
