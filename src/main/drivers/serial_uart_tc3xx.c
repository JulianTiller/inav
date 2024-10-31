/*
 * This file is part of Cleanflight.
 *
 * Cleanflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Cleanflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Cleanflight.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdbool.h>
#include <stdint.h>

#include "platform.h"

#include "drivers/time.h"
#include "drivers/io.h"
#include "rcc.h"
#include "drivers/nvic.h"
#include "dma.h"

#include "serial.h"
#include "serial_uart.h"
#include "serial_uart_impl.h"

#define UART_RX_BUFFER_SIZE UART1_RX_BUFFER_SIZE
#define UART_TX_BUFFER_SIZE UART1_TX_BUFFER_SIZE

typedef struct uartDevice_s {
    USART_TypeDef* dev;
    uartPort_t port;
    ioTag_t rx;
    ioTag_t tx;
    uint8_t rxBuffer[UART_RX_BUFFER_SIZE + sizeof(Ifx_Fifo) + 8];
    uint8_t txBuffer[UART_TX_BUFFER_SIZE + sizeof(Ifx_Fifo) + 8];
    uint32_t txPriority;
    uint32_t rxPriority;
    uint8_t oversampling;
    uint8_t medianFilter;
    uint8_t samplePointPosition;
} uartDevice_t;

//static uartPort_t uartPort[MAX_UARTS];
#ifdef USE_UART1
static uartDevice_t uart1 =
{
    .dev = USART1,
    .rx = IO_TAG(UART1_RX_PIN),
    .tx = IO_TAG(UART1_TX_PIN),
    .txPriority = IFX_INTPRIO_ASCLIN_USART1_TX,
    .rxPriority = IFX_INTPRIO_ASCLIN_USART1_RX,
    .oversampling = 15,
    .medianFilter = 3,
    .samplePointPosition = 9,
};
#endif

#ifdef USE_UART2
static uartDevice_t uart2 =
{
	.dev = USART2,
	.rx = IO_TAG(UART2_RX_PIN),
	.tx = IO_TAG(UART2_TX_PIN),
	.txPriority = IFX_INTPRIO_ASCLIN_USART2_TX,
	.rxPriority = IFX_INTPRIO_ASCLIN_USART2_RX,
	.oversampling = 15,
	.medianFilter = 3,
	.samplePointPosition = 9,
};
#endif

#ifdef USE_UART3
static uartDevice_t uart3 =
{
		.dev = USART3,
		.rx = IO_TAG(UART3_RX_PIN),
		.tx = IO_TAG(UART3_TX_PIN),
		.txPriority = IFX_INTPRIO_ASCLIN_USART3_TX,
		.rxPriority = IFX_INTPRIO_ASCLIN_USART3_RX,
		.oversampling = 15,
		.medianFilter = 3,
		.samplePointPosition = 9,
};
#endif

#ifdef USE_UART4
static uartDevice_t uart4 =
{
		.dev = UART4,
		.rx = IO_TAG(UART4_RX_PIN),
		.tx = IO_TAG(UART4_TX_PIN),
		.txPriority = IFX_INTPRIO_ASCLIN_USART4_TX,
		.rxPriority = IFX_INTPRIO_ASCLIN_USART4_RX,
		.oversampling = 15,
		.medianFilter = 3,
		.samplePointPosition = 9,
};
#endif

#ifdef USE_UART5
static uartDevice_t uart5 =
{
    .DMAChannel = DMA_CHANNEL_4,
#ifdef USE_UART5_RX_DMA
    .rxDMAStream = DMA1_Stream0,
#endif
#ifdef USE_UART5_TX_DMA
    .txDMAStream = DMA1_Stream7,
#endif
    .dev = UART5,
    .rx = IO_TAG(UART5_RX_PIN),
    .tx = IO_TAG(UART5_TX_PIN),
    .af = GPIO_AF8_UART5,
#ifdef UART5_AHB1_PERIPHERALS
    .rcc_ahb1 = UART5_AHB1_PERIPHERALS,
#endif
    .rcc_apb1 = RCC_APB1(UART5),
    .txIrq = DMA1_ST7_HANDLER,
    .rxIrq = UART5_IRQn,
    .txPriority = NVIC_PRIO_SERIALUART5_TXDMA,
    .rxPriority = NVIC_PRIO_SERIALUART5
};
#endif

#ifdef USE_UART6
static uartDevice_t uart6 =
{
    .DMAChannel = DMA_CHANNEL_5,
#ifdef USE_UART6_RX_DMA
    .rxDMAStream = DMA2_Stream1,
#endif
#ifdef USE_UART6_TX_DMA
    .txDMAStream = DMA2_Stream6,
#endif
    .dev = USART6,
    .rx = IO_TAG(UART6_RX_PIN),
    .tx = IO_TAG(UART6_TX_PIN),
    .af = GPIO_AF8_USART6,
#ifdef UART6_AHB1_PERIPHERALS
    .rcc_ahb1 = UART6_AHB1_PERIPHERALS,
#endif
    .rcc_apb2 = RCC_APB2(USART6),
    .txIrq = DMA2_ST6_HANDLER,
    .rxIrq = USART6_IRQn,
    .txPriority = NVIC_PRIO_SERIALUART6_TXDMA,
    .rxPriority = NVIC_PRIO_SERIALUART6
};
#endif

#ifdef USE_UART7
static uartDevice_t uart7 =
{
    .DMAChannel = DMA_CHANNEL_5,
#ifdef USE_UART7_RX_DMA
    .rxDMAStream = DMA1_Stream3,
#endif
#ifdef USE_UART7_TX_DMA
    .txDMAStream = DMA1_Stream1,
#endif
    .dev = UART7,
    .rx = IO_TAG(UART7_RX_PIN),
    .tx = IO_TAG(UART7_TX_PIN),
    .af = GPIO_AF8_UART7,
#ifdef UART7_AHB1_PERIPHERALS
    .rcc_ahb1 = UART7_AHB1_PERIPHERALS,
#endif
    .rcc_apb1 = RCC_APB1(UART7),
    .txIrq = DMA1_ST1_HANDLER,
    .rxIrq = UART7_IRQn,
    .txPriority = NVIC_PRIO_SERIALUART7_TXDMA,
    .rxPriority = NVIC_PRIO_SERIALUART7
};
#endif
#ifdef USE_UART8
static uartDevice_t uart8 =
{
    .DMAChannel = DMA_CHANNEL_5,
#ifdef USE_UART8_RX_DMA
    .rxDMAStream = DMA1_Stream6,
#endif
#ifdef USE_UART8_TX_DMA
    .txDMAStream = DMA1_Stream0,
#endif
    .dev = UART8,
    .rx = IO_TAG(UART8_RX_PIN),
    .tx = IO_TAG(UART8_TX_PIN),
    .af = GPIO_AF8_UART8,
#ifdef UART8_AHB1_PERIPHERALS
    .rcc_ahb1 = UART8_AHB1_PERIPHERALS,
#endif
    .rcc_apb1 = RCC_APB1(UART8),
    .txIrq = DMA1_ST0_HANDLER,
    .rxIrq = UART8_IRQn,
    .txPriority = NVIC_PRIO_SERIALUART8_TXDMA,
    .rxPriority = NVIC_PRIO_SERIALUART8
};
#endif



static uartDevice_t* uartHardwareMap[] = {
#ifdef USE_UART1
    &uart1,
#else
    NULL,
#endif
#ifdef USE_UART2
    &uart2,
#else
    NULL,
#endif
#ifdef USE_UART3
    &uart3,
#else
    NULL,
#endif
#ifdef USE_UART4
    &uart4,
#else
    NULL,
#endif
#ifdef USE_UART5
    &uart5,
#else
    NULL,
#endif
#ifdef USE_UART6
    &uart6,
#else
    NULL,
#endif
#ifdef USE_UART7
    &uart7,
#else
    NULL,
#endif
#ifdef USE_UART8
    &uart8,
#else
    NULL,
#endif
};

void uartTxIrqHandler(uartPort_t *s)
{
	IfxAsclin_Asc_isrTransmit(&s->asclin);
}

void uartRxIrqHandler(uartPort_t *s)
{
	IfxAsclin_Asc_isrReceive(&s->asclin);

	if(s->port.rxCallback)
	{
		uint8_t data;
		Ifx_SizeT size = 1;
		IfxAsclin_Asc_read(&s->asclin, &data, &size, TIME_INFINITE);
		s->port.rxCallback(data, s->port.rxCallbackData);
	}
}


uartPort_t *serialUART(UARTDevice_e device, uint32_t baudRate, portMode_t mode, portOptions_t options)
{
    uartPort_t *s;

    uartDevice_t *uart = uartHardwareMap[device];
    if (!uart) return NULL;

    s = &(uart->port);
    s->port.vTable = uartVTable;

    s->port.baudRate = baudRate;

    s->port.rxBuffer = uart->rxBuffer;
    s->port.txBuffer = uart->txBuffer;
    s->port.rxBufferSize = UART_RX_BUFFER_SIZE;
    s->port.txBufferSize = UART_TX_BUFFER_SIZE;

    s->USARTx = uart->dev;

    s->txIO = IOGetByTag(uart->tx);
    s->rxIO = IOGetByTag(uart->rx);
    s->txPriority = uart->txPriority;
    s->rxPriority = uart->rxPriority;

    s->oversampling = uart->oversampling;
    s->medianFilter = uart->medianFilter;
    s->samplePointPosition = uart->samplePointPosition;

    if (options & SERIAL_BIDIR) {
        IOInit(s->txIO, OWNER_SERIAL, RESOURCE_UART_TXRX, RESOURCE_INDEX(device));
    }
    else {
        if (mode & MODE_TX) {
            IOInit(s->rxIO, OWNER_SERIAL, RESOURCE_UART_TX, RESOURCE_INDEX(device));
        }

        if (mode & MODE_RX) {
            IOInit(s->rxIO, OWNER_SERIAL, RESOURCE_UART_RX, RESOURCE_INDEX(device));
        }
    }

    return s;
}

#ifdef USE_UART1

uartPort_t *serialUART1(uint32_t baudRate, portMode_t mode, portOptions_t options)
{
    return serialUART(UARTDEV_1, baudRate, mode, options);
}

IFX_INTERRUPT(asclinUSART1RxISR, 0, IFX_INTPRIO_ASCLIN_USART1_RX);
IFX_INTERRUPT(asclinUSART1TxISR, 0, IFX_INTPRIO_ASCLIN_USART1_TX);

void asclinUSART1RxISR(void)
{
	uartPort_t *s = &(uartHardwareMap[UARTDEV_1]->port);
	uartRxIrqHandler(s);
}

void asclinUSART1TxISR(void)
{
	uartPort_t *s = &(uartHardwareMap[UARTDEV_1]->port);
	uartTxIrqHandler(s);
}

#endif

#ifdef USE_UART2

uartPort_t *serialUART2(uint32_t baudRate, portMode_t mode, portOptions_t options)
{
    return serialUART(UARTDEV_2, baudRate, mode, options);
}

IFX_INTERRUPT(asclinUSART2RxISR, 0, IFX_INTPRIO_ASCLIN_USART2_RX);
IFX_INTERRUPT(asclinUSART2TxISR, 0, IFX_INTPRIO_ASCLIN_USART2_TX);

void asclinUSART2RxISR(void)
{
	uartPort_t *s = &(uartHardwareMap[UARTDEV_2]->port);
	uartRxIrqHandler(s);
}

void asclinUSART2TxISR(void)
{
	uartPort_t *s = &(uartHardwareMap[UARTDEV_2]->port);
	uartTxIrqHandler(s);
}

#endif

#ifdef USE_UART3

uartPort_t *serialUART3(uint32_t baudRate, portMode_t mode, portOptions_t options)
{
    return serialUART(UARTDEV_3, baudRate, mode, options);
}

IFX_INTERRUPT(asclinUSART3RxISR, 0, IFX_INTPRIO_ASCLIN_USART3_RX);
IFX_INTERRUPT(asclinUSART3TxISR, 0, IFX_INTPRIO_ASCLIN_USART3_TX);

void asclinUSART3RxISR(void)
{
	uartPort_t *s = &(uartHardwareMap[UARTDEV_3]->port);
	uartRxIrqHandler(s);
}

void asclinUSART3TxISR(void)
{
	uartPort_t *s = &(uartHardwareMap[UARTDEV_3]->port);
	uartTxIrqHandler(s);
}

#endif

#ifdef USE_UART4

uartPort_t *serialUART4(uint32_t baudRate, portMode_t mode, portOptions_t options)
{
    return serialUART(UARTDEV_4, baudRate, mode, options);
}

IFX_INTERRUPT(asclinUSART4RxISR, 0, IFX_INTPRIO_ASCLIN_USART4_RX);
IFX_INTERRUPT(asclinUSART4TxISR, 0, IFX_INTPRIO_ASCLIN_USART4_TX);

void asclinUSART4RxISR(void)
{
	uartPort_t *s = &(uartHardwareMap[UARTDEV_4]->port);
	uartRxIrqHandler(s);
}

void asclinUSART4TxISR(void)
{
	uartPort_t *s = &(uartHardwareMap[UARTDEV_4]->port);
	uartTxIrqHandler(s);
}

#endif

#ifdef USE_UART5
uartPort_t *serialUART5(uint32_t baudRate, portMode_t mode, portOptions_t options)
{
    return serialUART(UARTDEV_5, baudRate, mode, options);
}

// UART5 Rx/Tx IRQ Handler
void UART5_IRQHandler(void)
{
    uartPort_t *s = &(uartHardwareMap[UARTDEV_5]->port);
    uartIrqHandler(s);
}
#endif

#ifdef USE_UART6
uartPort_t *serialUART6(uint32_t baudRate, portMode_t mode, portOptions_t options)
{
    return serialUART(UARTDEV_6, baudRate, mode, options);
}

// USART6 Rx/Tx IRQ Handler
void USART6_IRQHandler(void)
{
    uartPort_t *s = &(uartHardwareMap[UARTDEV_6]->port);
    uartIrqHandler(s);
}
#endif

#ifdef USE_UART7
uartPort_t *serialUART7(uint32_t baudRate, portMode_t mode, portOptions_t options)
{
    return serialUART(UARTDEV_7, baudRate, mode, options);
}

// UART7 Rx/Tx IRQ Handler
void UART7_IRQHandler(void)
{
    uartPort_t *s = &(uartHardwareMap[UARTDEV_7]->port);
    uartIrqHandler(s);
}
#endif

#ifdef USE_UART8
uartPort_t *serialUART8(uint32_t baudRate, portMode_t mode, portOptions_t options)
{
    return serialUART(UARTDEV_8, baudRate, mode, options);
}

// UART8 Rx/Tx IRQ Handler
void UART8_IRQHandler(void)
{
    uartPort_t *s = &(uartHardwareMap[UARTDEV_8]->port);
    uartIrqHandler(s);
}
#endif
