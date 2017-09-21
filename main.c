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
#include <clock.h>

#include <Port/Io/IfxPort_Io.h>

#include <_Reg/IfxStm_reg.h>
#include <_Reg/IfxStm_bf.h>
#include <Stm/Std/IfxStm.h>

#include <_Reg/IfxAsclin_reg.h>
#include <_Reg/IfxAsclin_bf.h>

#include "interrupts.h"

#include "wdtcon.h"

#include "dhry.h"

#define RUN_NUMBER	1000000

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
				/* waitTime( at least for 1 character */
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

#ifndef EXTCLK
# define EXTCLK		(20000000)	/* external oscillator clock (20MHz) */
#endif
static Ifx_SCU * const pSCU = (Ifx_SCU *)&MODULE_SCU;

typedef struct _PllInitValue_t
{
	unsigned int valOSCCON;
	unsigned int valPLLCON0;
	unsigned int valPLLCON1;	/* first step K dividers */
	unsigned int valCCUCON0;
	unsigned int valCCUCON1;
	unsigned int valCCUCON2;
	unsigned int finalK;		/* final K2DIV value */
} PllInitValue_t;

/* 200/100 MHz @ 20MHz ext. clock */
static const PllInitValue_t g_PllInitValue_200_100 =
{
	/* OSCCON,	PLLCON0,	PLLCON1,	CCUCON0,	CCUCON1,	CCUCON2,    finalK */
	0x0007001C, 0x01017600, 0x00020505, 0x12120118, 0x10012242, 0x00000002, 2
};

static const PllInitValue_t g_PllInitValue_200_100;
#define PLL_VALUE_200_100 ((const PllInitValue_t *)(&g_PllInitValue_200_100))

#ifndef DEFAULT_PLL_VALUE
# define DEFAULT_PLL_VALUE		PLL_VALUE_200_100
#endif

static void system_set_pll(const PllInitValue_t *pPllInitValue)
{
	unsigned int k;

	unlock_safety_wdtcon();

	pSCU->OSCCON.U = pPllInitValue->valOSCCON;

	while (pSCU->CCUCON1.B.LCK)
		;
	pSCU->CCUCON1.U = pPllInitValue->valCCUCON1 | (1 << IFX_SCU_CCUCON1_UP_OFF);

	while (pSCU->CCUCON2.B.LCK)
		;
	pSCU->CCUCON2.U = pPllInitValue->valCCUCON2 | (1 << IFX_SCU_CCUCON2_UP_OFF);

	pSCU->PLLCON0.U |= ((1 << IFX_SCU_PLLCON0_VCOBYP_OFF) | (1 << IFX_SCU_PLLCON0_SETFINDIS_OFF));
	pSCU->PLLCON1.U =  pPllInitValue->valPLLCON1;				/* set Kn divider */
	pSCU->PLLCON0.U =  pPllInitValue->valPLLCON0				/* set P,N divider */
					| ((1 << IFX_SCU_PLLCON0_VCOBYP_OFF) | (1 << IFX_SCU_PLLCON0_CLRFINDIS_OFF));

	while (pSCU->CCUCON0.B.LCK)
		;
	pSCU->CCUCON0.U =  pPllInitValue->valCCUCON0 | (1 << IFX_SCU_CCUCON0_UP_OFF);

	lock_safety_wdtcon();

	if (0 == (pPllInitValue->valPLLCON0 & (1 << IFX_SCU_PLLCON0_VCOBYP_OFF)))	/* no prescaler mode requested */
	{
#ifndef SYSTEM_PLL_HAS_NO_LOCK
		/* wait for PLL locked */
		while (0 == pSCU->PLLSTAT.B.VCOLOCK)
			;
#endif

		unlock_safety_wdtcon();
		pSCU->PLLCON0.B.VCOBYP = 0;			/* disable VCO bypass */
		lock_safety_wdtcon();
	}

	/* update K dividers for stepping up to final clock */
	k = pSCU->PLLCON1.B.K2DIV;
	/* wait some time (100 us) */
	waitTime(100);
	while (k > pPllInitValue->finalK)
	{
		Ifx_SCU_PLLCON1 pllcon1 = pSCU->PLLCON1;

		--k;
		/* prepare value to write */
		pllcon1.B.K2DIV = k;
		pllcon1.B.K3DIV = k;
		/* wait until K2 operation is stable */
		while (0 == pSCU->PLLSTAT.B.K2RDY)
			;
		unlock_safety_wdtcon();
		pSCU->PLLCON1 = pllcon1;
		lock_safety_wdtcon();
		/* wait some time (100 us) */
		waitTime(100);
	}
}

/*! \brief System initialisation
 *  \param pPllInitValue ... address of PLL initialisation struct
 */
static void SYSTEM_InitExt(const PllInitValue_t *pPllInitValue)
{
	system_set_pll(pPllInitValue);
}

static unsigned long system_GetPllClock(void)
{
	unsigned int frequency = EXTCLK;	/* fOSC */

	Ifx_SCU_PLLSTAT pllstat = pSCU->PLLSTAT;
	Ifx_SCU_PLLCON0 pllcon0 = pSCU->PLLCON0;
	Ifx_SCU_PLLCON1 pllcon1 = pSCU->PLLCON1;

	if (0 == (pllstat.B.VCOBYST))
	{
		if (0 == (pllstat.B.FINDIS))
		{
			/* normal mode */
			frequency *= (pllcon0.B.NDIV + 1);		/* fOSC*N */
			frequency /= (pllcon0.B.PDIV + 1);		/* .../P  */
			frequency /= (pllcon1.B.K2DIV + 1);		/* .../K2 */
		}
		else	/* freerunning mode */
		{
			frequency = 800000000;		/* fVCOBASE 800 MHz (???) */
			frequency /= (pllcon1.B.K2DIV + 1);		/* .../K2 */
		}
	}
	else	/* prescaler mode */
	{
		frequency /= (pllcon1.B.K1DIV + 1);		/* fOSC/K1 */
	}

	return (unsigned long)frequency;
}

static unsigned long system_GetIntClock(void)
{
	unsigned long frequency = 0;
	switch (pSCU->CCUCON0.B.CLKSEL)
	{
		default:
		case 0:  /* back-up clock (typ. 100 MHz) */
			frequency = 100000000ul;
			break;
		case 1:	 /* fPLL */
			frequency = system_GetPllClock();
			break;
	}
	return frequency;
}

unsigned long SYSTEM_GetCpuClock(void)
{
	unsigned long frequency = system_GetIntClock();
	/* fCPU = fSRI */
	unsigned long divider = pSCU->CCUCON0.B.SRIDIV;
	unsigned long cpudiv = pSCU->CCUCON6.B.CPU0DIV;
	if (0 == divider)
		return 0;
	frequency /= divider;

	if (cpudiv != 0)
	{
		frequency *= (64 - cpudiv);
		frequency /= 64;
	}

	return frequency;
}

unsigned long SYSTEM_GetSysClock(void)
{
	unsigned long frequency = system_GetIntClock();
	unsigned long divider = pSCU->CCUCON0.B.SPBDIV;
	if (0 == divider)
		return 0;
	return (frequency / divider);
}


/* Global Variables: */

Rec_Pointer     Ptr_Glob,
Next_Ptr_Glob;
int             Int_Glob;
Boolean         Bool_Glob;
char            Ch_1_Glob,
Ch_2_Glob;
int             Arr_1_Glob [50];
int             Arr_2_Glob [50] [50];

#define REG	register

#ifndef REG
Boolean Reg = false;
#define REG
/* REG becomes defined as empty */
/* i.e. no register variables   */
#else
Boolean Reg = true;
#endif

/* variables for time measurement: */

#ifdef TIMES
struct tms      time_info;
extern  int     times (void);
/* see library function "times" */
#define Too_Small_Time (2*HZ)
/* Measurements should last at least about 2 seconds */
#endif
#ifdef TIME
extern long     time(long *);
/* see library function "time"  */
#define Too_Small_Time 2
/* Measurements should last at least 2 seconds */
#endif
#ifdef MSC_CLOCK
//extern clock_t clock(void);
#define Too_Small_Time (2*HZ)
#endif

int            Begin_Time,
End_Time,
User_Time;
float           Microseconds,
Dhrystones_Per_Second;

/* end of variables for time measurement */


void Proc_1 (Rec_Pointer Ptr_Val_Par)
/******************/
/* executed once */
{
	REG Rec_Pointer Next_Record = Ptr_Val_Par->Ptr_Comp;
	/* == Ptr_Glob_Next */
	/* Local variable, initialized with Ptr_Val_Par->Ptr_Comp,    */
	/* corresponds to "rename" in Ada, "with" in Pascal           */

	structassign (*Ptr_Val_Par->Ptr_Comp, *Ptr_Glob);
	Ptr_Val_Par->variant.var_1.Int_Comp = 5;
	Next_Record->variant.var_1.Int_Comp = Ptr_Val_Par->variant.var_1.Int_Comp;
	Next_Record->Ptr_Comp = Ptr_Val_Par->Ptr_Comp;
	Proc_3 (&Next_Record->Ptr_Comp);
	/* Ptr_Val_Par->Ptr_Comp->Ptr_Comp == Ptr_Glob->Ptr_Comp */
	if (Next_Record->Discr == Ident_1)
		/* then, executed */
	{
		Next_Record->variant.var_1.Int_Comp = 6;
		Proc_6 (Ptr_Val_Par->variant.var_1.Enum_Comp,
				&Next_Record->variant.var_1.Enum_Comp);
		Next_Record->Ptr_Comp = Ptr_Glob->Ptr_Comp;
		Proc_7 (Next_Record->variant.var_1.Int_Comp, 10,
				&Next_Record->variant.var_1.Int_Comp);
	}
	else /* not executed */
		structassign (*Ptr_Val_Par, *Ptr_Val_Par->Ptr_Comp);
} /* Proc_1 */


void Proc_2 (One_Fifty *Int_Par_Ref)
/******************/
/* executed once */
/* *Int_Par_Ref == 1, becomes 4 */
{
	One_Fifty  Int_Loc;
	Enumeration   Enum_Loc;

	Int_Loc = *Int_Par_Ref + 10;
	do /* executed once */
		if (Ch_1_Glob == 'A')
			/* then, executed */
		{
			Int_Loc -= 1;
			*Int_Par_Ref = Int_Loc - Int_Glob;
			Enum_Loc = Ident_1;
		} /* if */
	while (Enum_Loc != Ident_1); /* true */
} /* Proc_2 */


void Proc_3 (Rec_Pointer *Ptr_Ref_Par)
/******************/
/* executed once */
/* Ptr_Ref_Par becomes Ptr_Glob */
{
	if (Ptr_Glob != Null)
		/* then, executed */
		*Ptr_Ref_Par = Ptr_Glob->Ptr_Comp;
	Proc_7 (10, Int_Glob, &Ptr_Glob->variant.var_1.Int_Comp);
} /* Proc_3 */


void Proc_4 (void) /* without parameters */
/*******/
/* executed once */
{
	Boolean Bool_Loc;

	Bool_Loc = Ch_1_Glob == 'A';
	Bool_Glob = Bool_Loc | Bool_Glob;
	Ch_2_Glob = 'B';
} /* Proc_4 */


void Proc_5 (void) /* without parameters */
/*******/
/* executed once */
{
	Ch_1_Glob = 'A';
	Bool_Glob = false;
} /* Proc_5 */


/* Procedure for the assignment of structures,          */
/* if the C compiler doesn't support this feature       */
#ifdef  NOSTRUCTASSIGN
memcpy (d, s, l)
register char   *d;
register char   *s;
register int    l;
{
	while (l--) *d++ = *s++;
}
#endif

int main(void)
{
	uint32_t tmpMs;
	One_Fifty       Int_1_Loc;
	REG One_Fifty   Int_2_Loc;
	One_Fifty       Int_3_Loc;
	REG char        Ch_Index;
	Enumeration     Enum_Loc;
	Str_30          Str_1_Loc;
	Str_30          Str_2_Loc;
	REG int         Run_Index;
	REG int         Number_Of_Runs;

	SYSTEM_InitExt(DEFAULT_PLL_VALUE);

	gpio_init_pins();

	/* initialise timer at SYSTIME_CLOCK rate */
	TimerInit(HZ);
	/* add own handler for timer interrupts */
	TimerSetHandler(my_timer_handler);

	_init_uart(BAUDRATE);

	/* enable global interrupts */
	__enable();

	/* Initializations */
	Next_Ptr_Glob = (Rec_Pointer) malloc (sizeof (Rec_Type));
	Ptr_Glob = (Rec_Pointer) malloc (sizeof (Rec_Type));

	Ptr_Glob->Ptr_Comp                    = Next_Ptr_Glob;
	Ptr_Glob->Discr                       = Ident_1;
	Ptr_Glob->variant.var_1.Enum_Comp     = Ident_3;
	Ptr_Glob->variant.var_1.Int_Comp      = 40;
	strcpy (Ptr_Glob->variant.var_1.Str_Comp,
			"DHRYSTONE PROGRAM, SOME STRING");
	strcpy (Str_1_Loc, "DHRYSTONE PROGRAM, 1'ST STRING");

	Arr_2_Glob [8][7] = 10;
	/* Was missing in published program. Without this statement,    */
	/* Arr_2_Glob [8][7] would have an undefined value.             */
	/* Warning: With 16-Bit processors and Number_Of_Runs > 32000,  */
	/* overflow may occur for this array element.                   */

	printf ("\n");
	printf ("Dhrystone Benchmark, Version 2.1 (Language: C)\n");
	printf ("\n");
	if (Reg)
	{
		printf ("Program compiled with 'register' attribute\n");
		printf ("\n");
	}
	else
	{
		printf ("Program compiled without 'register' attribute\n");
		printf ("\n");
	}
	printf ("Please give the number of runs through the benchmark: ");
	{
		//    int n = 100000;
		//    scanf ("%d", &n);
		Number_Of_Runs = RUN_NUMBER;
	}
	printf ("\n");

	printf( "Execution starts, %d runs through Dhrystone\n", Number_Of_Runs);
	/***************/
	/* Start timer */
	/***************/

#ifdef TIMES
	times (&time_info);
	Begin_Time = (long) time_info.tms_utime;
#endif
#ifdef TIME
	Begin_Time = time ( (long *) 0);
#endif
#ifdef MSC_CLOCK
	Begin_Time = clock_msec();
#endif

	for (Run_Index = 1; Run_Index <= Number_Of_Runs; ++Run_Index)
	{

		Proc_5();
		Proc_4();
		/* Ch_1_Glob == 'A', Ch_2_Glob == 'B', Bool_Glob == true */
		Int_1_Loc = 2;
		Int_2_Loc = 3;
		strcpy (Str_2_Loc, "DHRYSTONE PROGRAM, 2'ND STRING");
		Enum_Loc = Ident_2;
		Bool_Glob = ! Func_2 (Str_1_Loc, Str_2_Loc);
		/* Bool_Glob == 1 */
		while (Int_1_Loc < Int_2_Loc)  /* loop body executed once */
		{
			Int_3_Loc = 5 * Int_1_Loc - Int_2_Loc;
			/* Int_3_Loc == 7 */
			Proc_7 (Int_1_Loc, Int_2_Loc, &Int_3_Loc);
			/* Int_3_Loc == 7 */
			Int_1_Loc += 1;
		} /* while */
		/* Int_1_Loc == 3, Int_2_Loc == 3, Int_3_Loc == 7 */
		Proc_8 (Arr_1_Glob, Arr_2_Glob, Int_1_Loc, Int_3_Loc);
		/* Int_Glob == 5 */
		Proc_1 (Ptr_Glob);
		for (Ch_Index = 'A'; Ch_Index <= Ch_2_Glob; ++Ch_Index)
			/* loop body executed twice */
		{
			if (Enum_Loc == Func_1 (Ch_Index, 'C'))
				/* then, not executed */
			{
				Proc_6 (Ident_1, &Enum_Loc);
				strcpy (Str_2_Loc, "DHRYSTONE PROGRAM, 3'RD STRING");
				Int_2_Loc = Run_Index;
				Int_Glob = Run_Index;
			}
		}
		/* Int_1_Loc == 3, Int_2_Loc == 3, Int_3_Loc == 7 */
		Int_2_Loc = Int_2_Loc * Int_1_Loc;
		Int_1_Loc = Int_2_Loc / Int_3_Loc;
		Int_2_Loc = 7 * (Int_2_Loc - Int_3_Loc) - Int_1_Loc;
		/* Int_1_Loc == 1, Int_2_Loc == 13, Int_3_Loc == 7 */
		Proc_2 (&Int_1_Loc);
		/* Int_1_Loc == 5 */

	} /* loop "for Run_Index" */

	/**************/
	/* Stop timer */
	/**************/

#ifdef TIMES
	times (&time_info);
	End_Time = (long) time_info.tms_utime;
#endif
#ifdef TIME
	End_Time = time ( (long *) 0);
#endif
#ifdef MSC_CLOCK
	End_Time = clock_msec();
#endif

	printf ("Execution ends\n");
	printf ("\n");
	printf ("Final values of the variables used in the benchmark:\n");
	printf ("\n");
	printf( "Int_Glob:            %d\n", Int_Glob);
	printf("        should be:   %d\n", 5);
	printf( "Bool_Glob:           %d\n", Bool_Glob);

	printf( "        should be:   %d\n", 1);

	printf("Ch_1_Glob:           %c\n", Ch_1_Glob);

	printf("        should be:   %c\n", 'A');

	printf("Ch_2_Glob:           %c\n", Ch_2_Glob);

	printf("        should be:   %c\n", 'B');

	printf("Arr_1_Glob[8]:       %d\n", Arr_1_Glob[8]);

	printf("        should be:   %d\n", 7);

	printf("Arr_2_Glob[8][7]:    %d\n", Arr_2_Glob[8][7]);

	printf ("        should be:   Number_Of_Runs + 10\n");
	printf ("Ptr_Glob->\n");
	printf("  Ptr_Comp:          %d\n", (int) Ptr_Glob->Ptr_Comp);

	printf ("        should be:   (implementation-dependent)\n");
	printf("  Discr:             %d\n", Ptr_Glob->Discr);

	printf("        should be:   %d\n", 0);

	printf("  Enum_Comp:         %d\n", Ptr_Glob->variant.var_1.Enum_Comp);

	printf("        should be:   %d\n", 2);

	printf("  Int_Comp:          %d\n", Ptr_Glob->variant.var_1.Int_Comp);

	printf("        should be:   %d\n", 17);

	printf("  Str_Comp:          %s\n", Ptr_Glob->variant.var_1.Str_Comp);

	printf ("        should be:   DHRYSTONE PROGRAM, SOME STRING\n");
	printf ("Next_Ptr_Glob->\n");
	printf("  Ptr_Comp:          %d\n", (int) Next_Ptr_Glob->Ptr_Comp);

	printf ("        should be:   (implementation-dependent), same as above\n");
	printf("  Discr:             %d\n", Next_Ptr_Glob->Discr);

	printf("        should be:   %d\n", 0);

	printf("  Enum_Comp:         %d\n", Next_Ptr_Glob->variant.var_1.Enum_Comp);

	printf("        should be:   %d\n", 1);

	printf("  Int_Comp:          %d\n", Next_Ptr_Glob->variant.var_1.Int_Comp);

	printf("        should be:   %d\n", 18);

	printf("  Str_Comp:          %s\n",
			Next_Ptr_Glob->variant.var_1.Str_Comp);

	printf ("        should be:   DHRYSTONE PROGRAM, SOME STRING\n");
	printf("Int_1_Loc:           %d\n", Int_1_Loc);

	printf("        should be:   %d\n", 5);

	printf("Int_2_Loc:           %d\n", Int_2_Loc);

	printf("        should be:   %d\n", 13);

	printf("Int_3_Loc:           %d\n", Int_3_Loc);

	printf("        should be:   %d\n", 7);

	printf("Enum_Loc:            %d\n", Enum_Loc);

	printf("        should be:   %d\n", 1);

	printf("Str_1_Loc:           %s\n", Str_1_Loc);

	printf ("        should be:   DHRYSTONE PROGRAM, 1'ST STRING\n");
	printf("Str_2_Loc:           %s\n", Str_2_Loc);

	printf ("        should be:   DHRYSTONE PROGRAM, 2'ND STRING\n");
	printf ("\n");

	User_Time = End_Time - Begin_Time;

	if (User_Time < Too_Small_Time)
	{
		printf( "Measured time too small to obtain meaningful results %d-%d\n", Begin_Time, End_Time);
		printf ("Please increase number of runs\n");
	}
	else
	{
#ifdef TIME
		Microseconds = (float) User_Time * Mic_secs_Per_Second
				/ (float) Number_Of_Runs;
		Dhrystones_Per_Second = (float) Number_Of_Runs / (float) User_Time;
#else
		Microseconds = (float) User_Time * (float)Mic_secs_Per_Second
				/ ((float) HZ * ((float) Number_Of_Runs));
		Dhrystones_Per_Second = ((float) HZ * (float) Number_Of_Runs)
			                        				/ (float) User_Time;
#endif
		printf("Microseconds for one run through Dhrystone[%d-%d]:", Begin_Time, End_Time);

		printf("%6.1f us\n", Microseconds);

		printf ("Dhrystones per Second:");
		printf("%6.1f\n", Dhrystones_Per_Second);
	}

	printf("Dhry @ Sys:%u Hz CPU:%u Hz %d\n",
			SYSTEM_GetSysClock(),
			SYSTEM_GetCpuClock(),
			__clocks_per_sec
			);

	while(1)
	{
//		printf("%u, %u, %u, %u\n", getfosc(), clock_msec(), g_Ticks, clock());

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
