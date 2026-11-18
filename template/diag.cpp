// Diagnostic build (DEBUG_TRACE): fault / unhandled-interrupt catchers, watchdog and the report
// over USB serial. Compiled to nothing in the normal firmware.
#include "app.h"
#include "debug_trace.h"
#include <cmath>

using namespace daisy;

#ifdef DEBUG_TRACE
// Diagnostic build, with (in template/): C_DEFS=-DDEBUG_TRACE make -s BUILD_DIR=build/debug
// TARGET=template_debug. Never part of the normal firmware. Breadcrumbs (debug_trace.h) in backup
// SRAM; a fault saves its registers and resets, a hang of more than 1 s resets through the
// watchdog. After the reset the device runs normally and prints the report over USB serial.
volatile DebugTrace __attribute__((section(".backup_sram"))) g_trace;
static constexpr uint32_t kTraceArmed = 0x7A9E5EEDu;
static DebugTrace trace_report;     // breadcrumbs from before the reset
static bool have_report = false, report_hang = false;
static constexpr int kVectors = 16 + 150;   // Cortex-M7 exceptions + STM32H750 interrupts
alignas(1024) static uint32_t ram_vectors[kVectors];

// Called with the stacked registers of the faulting code: r0-r3, r12, lr, pc, psr.
extern "C" __attribute__((used)) void TraceFault(uint32_t* frame) {
    g_trace.lr = frame[5];
    g_trace.pc = frame[6];
    g_trace.psr = frame[7];
    g_trace.cfsr = SCB->CFSR;
    g_trace.hfsr = SCB->HFSR;
    g_trace.bfar = SCB->BFAR;
    g_trace.mmfar = SCB->MMFAR;
    g_trace.cause = 1;
    __DSB();
    NVIC_SystemReset();
}

// An interrupt without a handler (it would end in libDaisy's endless Default_Handler): save which
// one and where the code was, then reset.
extern "C" __attribute__((used)) void TraceUnhandled(uint32_t* frame) {
    g_trace.lr = frame[5];
    g_trace.pc = frame[6];
    g_trace.psr = frame[7];
    g_trace.ipsr = __get_IPSR();
    g_trace.cause = 2;
    __DSB();
    NVIC_SystemReset();
}

extern "C" __attribute__((naked)) void TraceUnhandledEntry() {
    __asm volatile(
        "tst lr, #4         \n"
        "ite eq             \n"
        "mrseq r0, msp      \n"
        "mrsne r0, psp      \n"
        "b TraceUnhandled   \n");
}

extern "C" __attribute__((naked)) void TraceFaultEntry() {
    __asm volatile(
        "tst lr, #4    \n"
        "ite eq        \n"
        "mrseq r0, msp \n"
        "mrsne r0, psp \n"
        "b TraceFault  \n");
}

// After hw.Init: keep a report from before the reset, re-arm, route the faults to TraceFault.
void TraceBoot() {
    System::InitBackupSram();
    const uint32_t rsr = RCC->RSR;
    RCC->RSR |= RCC_RSR_RMVF;
    volatile uint32_t* src = reinterpret_cast<volatile uint32_t*>(&g_trace);
    uint32_t* dst = reinterpret_cast<uint32_t*>(&trace_report);
    const int words = sizeof(DebugTrace) / 4;
    if (g_trace.magic == kTraceArmed && (g_trace.cause == 1 || g_trace.cause == 2 || (rsr & RCC_RSR_IWDG1RSTF))) {
        for (int i = 0; i < words; i++) dst[i] = src[i];
        have_report = true;
        report_hang = (g_trace.cause == 0);
    }
    for (int i = 0; i < words; i++) src[i] = 0;
    g_trace.magic = kTraceArmed;
    const uint32_t* v = reinterpret_cast<const uint32_t*>(SCB->VTOR);
    for (int i = 0; i < kVectors; i++) ram_vectors[i] = v[i];
    // Interrupts without a handler all point to libDaisy's endless Default_Handler: the most
    // common entry in the table. Route them (and NMI) to the catcher.
    uint32_t dflt = 0;
    int best = 0;
    for (int i = 16; i < kVectors; i++) {
        int n = 0;
        for (int j = 16; j < kVectors; j++)
            if (ram_vectors[j] == ram_vectors[i]) n++;
        if (n > best) {
            best = n;
            dflt = ram_vectors[i];
        }
    }
    for (int i = 2; i < kVectors; i++)
        if (ram_vectors[i] == dflt) ram_vectors[i] = reinterpret_cast<uint32_t>(&TraceUnhandledEntry);
    for (int i = 3; i <= 6; i++)   // HardFault, MemManage, BusFault, UsageFault
        ram_vectors[i] = reinterpret_cast<uint32_t>(&TraceFaultEntry);
    SCB_CleanDCache_by_Addr(ram_vectors, sizeof(ram_vectors));
    __DSB();
    SCB->VTOR = reinterpret_cast<uint32_t>(ram_vectors);
    __DSB();
    __ISB();
    SCB->SHCSR |= SCB_SHCSR_MEMFAULTENA_Msk | SCB_SHCSR_BUSFAULTENA_Msk | SCB_SHCSR_USGFAULTENA_Msk;
}

// Independent watchdog: resets the device if it is not refreshed for ms milliseconds (max ~4 s).
void TraceWatchdogStart(uint32_t ms) {
    IWDG1->KR = 0xCCCC;   // start
    IWDG1->KR = 0x5555;   // unlock the settings
    IWDG1->PR = 3;        // LSI 32 kHz / 32 = 1 kHz
    IWDG1->RLR = (ms > 4095) ? 4095 : ms;   // 1 ms per count
    while (IWDG1->SR != 0) {}
    IWDG1->KR = 0xAAAA;   // refresh
}

static const char* TraceStageName(uint32_t s) {
    static const char* const names[] = {"none", "block start", "player", "grain start", "grain read",
                                        "head", "head ghost", "head write", "head stutter",
                                        "head lead zone", "tape sound", "block end"};
    return (s <= kTrBlockEnd) ? names[s] : "?";
}

#define F3(x) ((x) < 0.0f ? '-' : ' '), (int)fabsf(x), (int)((fabsf(x) - floorf(fabsf(x))) * 1000.0f)
void TracePrint() {
    if (!have_report) {
        hw.PrintLine("diag build running, no report yet (blocks %lu)", (unsigned long)g_trace.blocks);
        return;
    }
    const DebugTrace& t = trace_report;
    hw.PrintLine("REPORT: %s in '%s', after %lu audio blocks, %lu main loops, main loop stage %lu",
                 report_hang ? "HANG (watchdog)" : (t.cause == 2 ? "UNHANDLED INTERRUPT" : "FAULT"),
                 TraceStageName(t.stage), (unsigned long)t.blocks, (unsigned long)t.main_loops,
                 (unsigned long)t.main_stage);
    if (t.cause == 2)
        hw.PrintLine("  interrupt: exception %lu = IRQ %ld", (unsigned long)t.ipsr, (long)t.ipsr - 16);
    if (!report_hang)
        hw.PrintLine("  pc 0x%08lx lr 0x%08lx psr 0x%08lx cfsr 0x%08lx hfsr 0x%08lx bfar 0x%08lx mmfar 0x%08lx",
                     (unsigned long)t.pc, (unsigned long)t.lr, (unsigned long)t.psr, (unsigned long)t.cfsr,
                     (unsigned long)t.hfsr, (unsigned long)t.bfar, (unsigned long)t.mmfar);
    hw.PrintLine("  player: mode %ld loop %ld + %ld pos %c%d.%03d step %c%d.%03d speed %c%d.%03d",
                 (long)t.mode, (long)t.rs, (long)t.rl, F3(t.pos), F3(t.step), F3(t.speed));
    hw.PrintLine("  head: loop %ld + %ld pos %c%d.%03d step %c%d.%03d from %c%d.%03d count %ld k %ld write %ld",
                 (long)t.h_rs, (long)t.h_rl, F3(t.h_pos), F3(t.h_step), F3(t.from), (long)t.count,
                 (long)t.k, (long)t.wi);
    hw.PrintLine("  head jump: fade-in left %ld ghost left %ld ghost from %c%d.%03d",
                 (long)t.fade_left, (long)t.ghost_left, F3(t.ghost_from));
}
#endif
