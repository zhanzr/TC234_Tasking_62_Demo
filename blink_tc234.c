/*****************************************************
 *
 * blink_tc234.c
 *
 * Description : Hello World in C, ANSI-style
 *
 */

#include <stdio.h>
#include <stdint.h>
#include <sevt.h>

#include <Port/Io/IfxPort_Io.h>

#include <_Reg/IfxStm_reg.h>
#include <_Reg/IfxStm_bf.h>
#include <Stm/Std/IfxStm.h>

#include "interrupts.h"

void SimpleDelay(uint32_t t)
{
	uint32_t d = t*1000;
	while(--d)
	{
		__nop();
	}
}

#ifndef MSC_CLOCK
#define MSC_CLOCK

#define HZ	1000
#endif

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
	_sevt_isr_install(irqId, tick_irq, SYSTIME_ISR_PRIO);

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

int main(void)
{
	uint32_t tmpMs;

	gpio_init_pins();

	/* initialise timer at SYSTIME_CLOCK rate */
	TimerInit(HZ);
	/* add own handler for timer interrupts */
	TimerSetHandler(my_timer_handler);

	/* enable global interrupts */
	__asm__ volatile ("enable" ::: "memory");

	while(1)
	{
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
