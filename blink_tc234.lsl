// TASKING VX-toolset for TriCore
// Eclipse project linker script file
//
#if defined(__PROC_TC23X__)
#include "tc23x.lsl"
derivative my_tc23x extends tc23x
{
}
#else
#include <cpu.lsl>
#endif
