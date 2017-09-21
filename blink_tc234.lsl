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
section_layout :vtc:linear
{
    group heap "heap" (size = 32k,tag="user");
}
section_layout :tc0:linear
{
    group stack "ustack_tc0" (size = 32k,tag="user");
}
section_layout :tc0:linear
{
    group stack "istack_tc0" (size = 8k,tag="user");
}
