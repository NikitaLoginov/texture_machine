// Hot audio code runs from ITCM (64 KB of zero-wait instruction RAM in the Cortex-M7 core): the
// linker script (texture_machine_flash.lds, section .itcm_text) stores it in flash; this copies it
// to ITCM at boot, before every other static constructor and main().
#include <cstdint>

extern "C" uint32_t _sitcm_text, _eitcm_text, _sitcm_text_load;   // from the linker script

__attribute__((constructor(101))) static void CopyItcmText()
{
    uint32_t*       dst = &_sitcm_text;
    const uint32_t* src = &_sitcm_text_load;
    while (dst < &_eitcm_text) *dst++ = *src++;
    __asm volatile("dsb\n\tisb" ::: "memory");   // the copied code is visible before it runs
}
