/*****************************************************
 *
 * blink_tc234.c
 *
 * Description : Hello World in C, ANSI-style
 *
 */

#include <stdio.h>
#include <stddef.h>
#include <stdlib.h>
#include <stdarg.h>
#include <string.h>
#include <stdint.h>
#include <errno.h>
#include <sevt.h>

#include <Port/Io/IfxPort_Io.h>

#include <_Reg/IfxStm_reg.h>
#include <_Reg/IfxStm_bf.h>
#include <Stm/Std/IfxStm.h>

#include <_Reg/IfxAsclin_reg.h>
#include <_Reg/IfxAsclin_bf.h>

#include "interrupts.h"

#include "wdtcon.h"

#define	NON_BLOCKING_SERIALIO	1

#ifndef MSC_CLOCK
#define MSC_CLOCK

#define HZ	1000
#endif

#define XMIT_INTERRUPT			3
#define RECV_INTERRUPT			4

/* UART primitives */
#define TX_INT_START(u)			((u)->FLAGSENABLE.B.TFLE = 1, (u)->FLAGSSET.U = (IFX_ASCLIN_FLAGSSET_TFLS_MSK << IFX_ASCLIN_FLAGSSET_TFLS_OFF))
#define TX_INT_STOP(u)			((u)->FLAGSENABLE.B.TFLE = 0)
#define TX_INT_CHECK(u)			((u)->FLAGSENABLE.B.TFLE)

static Ifx_ASCLIN * const asclin0 = (Ifx_ASCLIN *)&MODULE_ASCLIN0;

static Ifx_P * const port = (Ifx_P *)&MODULE_P14;

#define UARTBASE				asclin0

/* baud rate values at 100 MHz */
#define BAUD_9600				(48 * 1)
#define BAUD_19200				(48 * 2)
#define BAUD_38400				(48 * 4)
#define BAUD_57600				(48 * 6)
#define BAUD_115200				(48 * 12)

/* Port Modes */
#define IN_NOPULL0				0x00	/* Port Input No Pull Device */
#define IN_PULLDOWN				0x01	/* Port Input Pull Down Device */
#define IN_PULLUP				0x02	/* Port Input Pull Up Device */
#define IN_NOPULL3				0x03	/* Port Input No Pull Device */
#define OUT_PPGPIO				0x10	/* Port Output General Purpose Push/Pull */
#define OUT_PPALT1				0x11	/* Port Output Alternate 1 Function Push/Pull */
#define OUT_PPALT2				0x12	/* Port Output Alternate 2 Function Push/Pull */
#define OUT_PPALT3				0x13	/* Port Output Alternate 3 Function Push/Pull */
#define OUT_PPALT4				0x14	/* Port Output Alternate 4 Function Push/Pull */
#define OUT_PPALT5				0x15	/* Port Output Alternate 5 Function Push/Pull */
#define OUT_PPALT6				0x16	/* Port Output Alternate 6 Function Push/Pull */
#define OUT_PPALT7				0x17	/* Port Output Alternate 7 Function Push/Pull */
#define OUT_ODGPIO				0x18	/* Port Output General Purpose Open Drain */
#define OUT_ODALT1				0x19	/* Port Output Alternate 1 Function Open Drain */
#define OUT_ODALT2				0x1A	/* Port Output Alternate 2 Function Open Drain */
#define OUT_ODALT3				0x1B	/* Port Output Alternate 3 Function Open Drain */
#define OUT_ODALT4				0x1C	/* Port Output Alternate 4 Function Open Drain */
#define OUT_ODALT5				0x1D	/* Port Output Alternate 5 Function Open Drain */
#define OUT_ODALT6				0x1E	/* Port Output Alternate 6 Function Open Drain */
#define OUT_ODALT7				0x1F	/* Port Output Alternate 7 Function Open Drain */

/* definitions for RX error conditions */
#define ASC_ERROR_MASK			((IFX_ASCLIN_FLAGS_PE_MSK << IFX_ASCLIN_FLAGS_PE_OFF) | \
								 (IFX_ASCLIN_FLAGS_FE_MSK << IFX_ASCLIN_FLAGS_FE_OFF) | \
								 (IFX_ASCLIN_FLAGS_RFO_MSK << IFX_ASCLIN_FLAGS_RFO_OFF))

#define ASC_CLRERR_MASK			((IFX_ASCLIN_FLAGSCLEAR_PEC_MSK << IFX_ASCLIN_FLAGSCLEAR_PEC_OFF) | \
								 (IFX_ASCLIN_FLAGSCLEAR_FEC_MSK << IFX_ASCLIN_FLAGSCLEAR_FEC_OFF) | \
								 (IFX_ASCLIN_FLAGSCLEAR_RFOC_MSK << IFX_ASCLIN_FLAGSCLEAR_RFOC_OFF))

/* UART primitives */
#define RX_CLEAR(u)				((u)->FLAGSCLEAR.U = (IFX_ASCLIN_FLAGSCLEAR_RFLC_MSK << IFX_ASCLIN_FLAGSCLEAR_RFLC_OFF))
#define TX_CLEAR(u)				((u)->FLAGSCLEAR.U = (IFX_ASCLIN_FLAGSCLEAR_TFLC_MSK << IFX_ASCLIN_FLAGSCLEAR_TFLC_OFF))
#define PUT_CHAR(u, c)			((u)->TXDATA.U = (c))
#define GET_CHAR(u)				((u)->RXDATA.U)
#define GET_ERROR_STATUS(u)		(((u)->FLAGS.U) & ASC_ERROR_MASK)
#define RESET_ERROR(u)			((u)->FLAGSCLEAR.U = ASC_CLRERR_MASK)

/* type of a timer callback function */
typedef void (*TCF)(void);

#define SYSTIME_ISR_PRIO	2

static Ifx_STM * const StmBase = (Ifx_STM *)&MODULE_STM0;

/* timer reload value (needed for subtick calculation) */
static unsigned int reload_value = 0;

/* pointer to user specified timer callback function */
static TCF user_handler = (TCF)0;

/* timer interrupt routine */
static void tick_irq(int reload_value)
{
	/* set new compare value */
	StmBase->CMP[0].U += (unsigned int)reload_value;
	if (user_handler)
	{
		user_handler();
	}
}

void TimerInit(unsigned int hz)
{
	unsigned int frequency = (uint32_t)IfxStm_getFrequency(StmBase);
	int irqId = SRC_ID_STM0SR0;

	reload_value = frequency / hz;

	/* install handler for timer interrupt */
//	_sevt_isr_install(irqId, tick_irq, 0);
	_sevt_isr_install(SYSTIME_ISR_PRIO, tick_irq, 0);

	/* prepare compare register */
	StmBase->CMP[0].U = StmBase->TIM0.U + reload_value;
	StmBase->CMCON.B.MSIZE0 = 31;	/* use bits 31:0 for compare */
	/* reset interrupt flag */
	StmBase->ISCR.U = (IFX_STM_ISCR_CMP0IRR_MSK << IFX_STM_ISCR_CMP0IRR_OFF);
	StmBase->ICR.B.CMP0EN = 1;
}

/* Install <handler> as timer callback function */
void TimerSetHandler(TCF handler)
{
	user_handler = handler;
}

volatile uint32_t g_Ticks;

/* timer callback handler */
static void my_timer_handler(void)
{
	++g_Ticks;
}



#ifndef RS232_RX_BUFSIZE
#define RS232_RX_BUFSIZE	0x100
#endif /* RS232_RX_BUFSIZE */

#ifndef RS232_TX_BUFSIZE
#define RS232_TX_BUFSIZE	0x100
#endif /* RS232_TX_BUFSIZE */

#ifndef RX_CLEAR
#define RX_CLEAR(u)
#endif

#ifndef TX_CLEAR
#define TX_CLEAR(u)
#endif

#define SPRINTF		sprintf
#define VSPRINTF	vsprintf

#define BUFSIZE		0x100

#define BAUDRATE	115200

typedef struct
{
	unsigned int	head;
	unsigned int	tail;
	char			buf[RS232_RX_BUFSIZE];
} RxBuffer_t;


typedef struct
{
	unsigned int	head;
	unsigned int	tail;
	char			buf[RS232_TX_BUFSIZE];
} TxBuffer_t;


/* Circular send and receive buffers */
static TxBuffer_t sendBuf;
static RxBuffer_t recvBuf;


/* FIFO support */
static inline int isEmptyTXFifo(void)
{
	return (sendBuf.tail == sendBuf.head);
}

static inline int getFreeTXFifo(void)
{
	int used = (RS232_TX_BUFSIZE + sendBuf.head - sendBuf.tail) % RS232_TX_BUFSIZE;
	return (RS232_TX_BUFSIZE - 1 - used);
}

static inline int readTXFifo(char *cp)
{
	int res = 0;
	if (sendBuf.tail != sendBuf.head)
	{
		unsigned int next = (sendBuf.tail + 1) % RS232_TX_BUFSIZE;
		*cp = sendBuf.buf[sendBuf.tail];
		sendBuf.tail = next;
		res = 1;
	}
	return res;
}

static inline int writeTXFifo(char c)
{
	int res = 0;
	unsigned int next = (sendBuf.head + 1) % RS232_TX_BUFSIZE;
	if (next != sendBuf.tail)
	{
		sendBuf.buf[sendBuf.head] = c;
		sendBuf.head = next;
		res = 1;
	}
	return res;
}

static inline int readRXFifo(char *cp)
{
	int res = 0;
	if (recvBuf.tail != recvBuf.head)
	{
		unsigned int next = (recvBuf.tail + 1) % RS232_RX_BUFSIZE;
		*cp = recvBuf.buf[recvBuf.tail];
		recvBuf.tail = next;
		res = 1;
	}
	return res;
}

static inline int writeRXFifo(char c)
{
	int res = 0;
	unsigned int next = (recvBuf.head + 1) % RS232_RX_BUFSIZE;
	if (next != recvBuf.tail)
	{
		recvBuf.buf[recvBuf.head] = c;
		recvBuf.head = next;
		res = 1;
	}
	return res;
}


/* Send character CHR via the serial line */
static inline void _out_uart(const char chr)
{
	TX_CLEAR(UARTBASE);
	/* send the character */
	PUT_CHAR(UARTBASE, chr);
}

/* Receive (and return) a character from the serial line */
static inline char _in_uart(void)
{
	/* read the character */
	char c = (char)GET_CHAR(UARTBASE);
	/* acknowledge receive */
	RX_CLEAR(UARTBASE);
	return c;
}

/* Interrupt Service Routine for RX */
static void _uart_rx_handler(int arg)
{
	(void)arg;
	/* check for error condition */
	if (GET_ERROR_STATUS(UARTBASE))
	{
		/* ignore this character */
		_in_uart();
		/* reset error flags */
		RESET_ERROR(UARTBASE);
	}
	else
	{
		char c = _in_uart();
		writeRXFifo(c);
	}
}

/* Interrupt Service Routine for TX */
static void _uart_tx_handler(int arg)
{
	char c;
	(void)arg;

	if (readTXFifo(&c))
	{
		_out_uart(c);
	}
	else
	{
		/* all done --> disable TX interrupt */
		TX_INT_STOP(UARTBASE);
	}
}


/* Externally visible functions */

/* get a character from serial line */
int _uart_getchar(char *c)
{
	return readRXFifo(c);
}

/* send a buffer of given size <len> over serial line */
int _uart_send(const char *buffer, int len)
{
	int ret = 0;

	if (len)
	{
		if (getFreeTXFifo() >= len)
		{
			int cnt = 0;
			for (; cnt < len; ++cnt)
			{
				writeTXFifo(*buffer++);
			}
			/* check whether TX must be triggered */
			if (!TX_INT_CHECK(UARTBASE))
			{
				/* enable TX interrupt for sending */
				TX_INT_START(UARTBASE);
			}
			ret = 1;
		}
	}

	return ret;
}

/* send a string over serial line */
int _uart_puts(const char *str)
{
	int len = strlen(str);

	return _uart_send(str, len);
}

/* test UARTs sending state */
int _uart_sending(void)
{
	int ret = (0 == isEmptyTXFifo());
	if (0 == ret)
	{
		/* wait until last byte is sent */
		ret = TX_INT_CHECK(UARTBASE);
	}
	return ret;
}


/* Initialise asynchronous interface to operate at baudrate,8,n,1 */
void _uart_init_bsp(int baudrate, void (*uart_rx_isr)(int arg), void (*uart_tx_isr)(int arg))
{
	unsigned int numerator;
	unsigned int denominator;

	/* install handlers for transmit and receive interrupts */
	_sevt_isr_install(XMIT_INTERRUPT, uart_tx_isr, 0);
	_sevt_isr_install(RECV_INTERRUPT, uart_rx_isr, 0);

	/* on board wiggler is connected to ASCLIN0 */
	/* ARX0A/P14.1 (RXD), ATX0/P14.0 (TXD) */
	/* Set TXD/P14.0 to "output" and "high" */
	port->IOCR0.B.PC0 = OUT_PPALT2;
	port->OMR.B.PS0 = 1;

	/* baudrate values at 100 MHz */
	denominator = 3125;

	switch (baudrate)
	{
		case   9600 : numerator =   BAUD_9600; break;
		case  19200 : numerator =  BAUD_19200; break;
		default     :
		case  38400 : numerator =  BAUD_38400; break;
		case  57600 : numerator =  BAUD_57600; break;
		case 115200 : numerator = BAUD_115200; break;
	}

	/* Enable ASCn */
	unlock_wdtcon();
	UARTBASE->CLC.U = 0;
	lock_wdtcon();
	/* read back for activating module */
	(void)UARTBASE->CLC.U;

	/* select ARX0A/P14.1 as input pin */
	UARTBASE->IOCR.B.ALTI = 0;

	/* Program ASC0 */
	UARTBASE->CSR.U = 0;

	/* configure TX and RX FIFOs */
	UARTBASE->TXFIFOCON.U = (1 << 6)	/* INW: (1 == 1 byte) */
						  | (1 << 1)	/* ENO */
						  | (1 << 0);	/* FLUSH */
	UARTBASE->RXFIFOCON.U = (1 << 6)	/* OUTW: (1 == 1 byte) */
						  | (1 << 1)	/* ENI */
						  | (1 << 0);	/* FLUSH */

	UARTBASE->BITCON.U = ( 9 << 0)		/* PRESCALER: 10 */
					   | (15 << 16)		/* OVERSAMPLING: 16 */
					   | ( 9 << 24)		/* SAMPLEPOINT: position 7,8,9 */
					   | (1u << 31);	/* SM: 3 samples per bit */

	/* data format: 8N1 */
	UARTBASE->FRAMECON.U = (1 << 9)		/* STOP: 1 bit */
						 | (0 << 16)	/* MODE: Init */
						 | (0 << 30);	/* PEN: no parity */
	UARTBASE->DATCON.U = (7 << 0);		/* DATLEN: 8 bit */

	/* set baudrate value */
	UARTBASE->BRG.U = (denominator << 0)	/* DENOMINATOR */
					| (numerator << 16);	/* NUMERATOR */

	UARTBASE->FRAMECON.B.MODE = 1;			/* ASC mode */
	UARTBASE->FLAGSENABLE.U = (1u << 28);	/* FLAGSENABLE.RFLE */
	UARTBASE->CSR.U = 1;					/* select CLC as clock source */
}

/* Initialise asynchronous interface to operate at baudrate,8,n,1 */
void _init_uart(int baudrate)
{
    _uart_init_bsp(baudrate, _uart_rx_handler, _uart_tx_handler);
}


static void my_puts(const char *str)
{
	char buffer[BUFSIZE];

	SPRINTF(buffer, "%s\r\n", str);
	_uart_puts(buffer);
}

static void my_printf(const char *fmt, ...)
{
	char buffer[BUFSIZE];
	va_list ap;

	va_start(ap, fmt);
	VSPRINTF(buffer, fmt, ap);
	va_end(ap);

	_uart_puts(buffer);
}

/* POSIX read function */
/* read characters from file descriptor fd into given buffer, at most count bytes */
/* returns number of characters in buffer */
size_t read(int fd, void *buffer, size_t count)
{
	size_t index = 0;

	if (fileno(stdin) == fd)
	{
#if (NON_BLOCKING_SERIALIO > 0)
		char *ptr = (char *)buffer;
		do
		{
			if (1 == _uart_getchar(ptr))
			{
				++ptr;
				++index;
			}
			else
			{
				/* wait at least for 1 character */
				if (index >= 1)
				{
					break;
				}
			}
		} while (index < count);
#else
		unsigned char *ptr = (unsigned char *)buffer;
		do
		{
			if (1 == _poll_uart(ptr))
			{
				++ptr;
				++index;
			}
			else
			{
				/* wait at least for 1 character */
				if (index >= 1)
				{
					break;
				}
			}
		} while (index < count);
#endif /* NON_BLOCKING_SERIALIO */
	}

	return index;
}

/* POSIX write function */
/* write content of buffer to file descriptor fd */
/* returns number of characters that have been written */
size_t write(int fd, const void *buffer, size_t count)
{
	size_t index = 0;

	if ((fileno(stdout) == fd) || (fileno(stderr) == fd))
	{
#if (NON_BLOCKING_SERIALIO > 0)
		int ret = _uart_send((const char *)buffer, (int)count);
		if (ret)
		{
			index = count;
		}
#else
		const unsigned char *ptr = (const unsigned char *)buffer;
		while (index < count)
		{
			_uart_puts(*ptr++);
			++index;
		}
#endif /* NON_BLOCKING_SERIALIO */
	}

	return index;
}

int main(void)
{
	uint32_t tmpMs;

	gpio_init_pins();

	/* initialise timer at SYSTIME_CLOCK rate */
	TimerInit(HZ);
	/* add own handler for timer interrupts */
	TimerSetHandler(my_timer_handler);

	_init_uart(BAUDRATE);

	/* enable global interrupts */
	__enable();

	while(1)
	{
		my_printf("Test1\n");
		my_puts("Test2\n");
		printf("%u, %u\n", getfosc(), clock_msec());

		IfxPort_setPinHigh(&MODULE_P13, 0);
		IfxPort_setPinHigh(&MODULE_P13, 1);
//		IfxPort_setPinHigh(&MODULE_P13, 2);
//		IfxPort_setPinHigh(&MODULE_P13, 3);

		tmpMs = clock_msec();
		while((tmpMs+1000) > clock_msec())
		{
			__nop();
		}
		IfxPort_setPinLow(&MODULE_P13, 0);
		IfxPort_setPinLow(&MODULE_P13, 1);
//		IfxPort_setPinLow(&MODULE_P13, 2);
//		IfxPort_setPinLow(&MODULE_P13, 3);

		tmpMs = clock_msec();
		while((tmpMs+1000) > clock_msec())
		{
			__nop();
		}
	}
//	printf( "Hello world\n" );
}
