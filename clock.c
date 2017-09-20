/**************************************************************************
**                                                                        *
**  FILE        :  clock.c                                                *
**                                                                        *
**  DESCRIPTION :  The clock function returns the current processor time. *
**                 To determine the time in seconds, the value returned   *
**                 by the clock function should be divided by the value   *
**                 of the macro CLOCKS_PER_SEC, defined in <time.h>.      *
**                                                                        *
**  IMPORTANT NOTE:This module uses the definition of SYSTIME_LOW and     *
**                 SYSTIME_HIGH, as defined in the sfr/def files in the   *
**                 include directory of the product.                      *
**                 By default the library is built for a default          *
**                 target (e.g. tc11ib). If the CPU you are using is      *
**                 different from this default, it may be necessary to    *
**                 rebuild the library to correctly use this function.    *
**                                                                        *
**  Copyright 1996-2017 TASKING BV                                        *
**                                                                        *
**************************************************************************/

/**************************************************************************
**                                                                        *
**  The following registers maintain the system clock in clockticks,      *
**  duration of single clocktick and wraptime are given for 100 MHz       *
**                                                                        *
**  SYSTIME_LOW:  Basecounter,  (  10. ns, 42.95 s )                      *
**  SYSTIME_HIGH: Counts the upper 24 bits of the system time counter     *
**                                                                        *
**  when the system uses a different clockspeed the duration will vary    *
**  proportionally. With a clock of 16 MHz the relevant durations for     *
**  SYSTIME_LOW are ( 62.5 ns, 268.4 s )                                  *
**                                                                        *
**  The total system clock (counter) is 56 bits long.                     *
**                                                                        *
**************************************************************************/

#include <time.h>

#ifdef __CPU__
#include __SFRFILE__(__CPU__)
#endif

/************************************************************************************************************/
unsigned long long setfoschz ( unsigned long long hz )
{
        __clocks_per_sec = ( clock_t )hz;

        return hz;
}

/************************************************************************************************************/

int setfosc ( int newmhz )
{
        __clocks_per_sec = 1000000 * ( clock_t )newmhz;

        return newmhz;
}

/************************************************************************************************************/

int getfosc ( void )
{
        return (int)( __clocks_per_sec / 1000000 );
}

/************************************************************************************************************/

extern unsigned long    clock_msec( void )
{
        return (unsigned long)(clock ()/ ( __clocks_per_sec / 1000 ));
}

/************************************************************************************************************/

extern unsigned long    clock_usec( void )
{
        return (unsigned long)(clock ()/ ( __clocks_per_sec / 1000000 ));
}

/************************************************************************************************************/

extern unsigned long    clock_usec_fast( void )
{
        return (unsigned long)(SYSTIME_LOW/ ( __clocks_per_sec / 1000000 ));
}

/************************************************************************************************************/

extern clock_t clock( void )
{
        unsigned int _1st_SYSTIME_LOW;
        unsigned int _2nd_SYSTIME_HIGH;
        unsigned int previous_interrupt_state;

        previous_interrupt_state = __mfcr(ICR);
        __mtcr(ICR,previous_interrupt_state & 0xFFFFFEFFu );
        _1st_SYSTIME_LOW = SYSTIME_LOW;
        _2nd_SYSTIME_HIGH = SYSTIME_HIGH;
        __mtcr(ICR,previous_interrupt_state);

        return (clock_t)_2nd_SYSTIME_HIGH << 32 | _1st_SYSTIME_LOW;
}
