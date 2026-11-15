#pragma once
// Motion release hot path in internal RAM (P0 timing, 15/11/2026).
//
// Hardware PMU (diagnostic 8530d3a, fixture 15/11): instruction memory stalls
// are 0.47-0.50 of the NOMINAL+COMPENSATE cycles in every class; the code a
// release touches in flash (46-57 KB, QEMU trace) is about three times the
// 16 KB I-cache. Functions marked here are fetched from IRAM, outside the
// cache: same machine code and results, a different section. Chosen by
// simulated misses avoided per byte (16 KB 8-way LRU replay of the 14/11
// fixture sequence); each byte here is a byte less of internal heap.
//
// The pure motion library takes no platform header: the placement is a
// compiler attribute only. ".iram1.*" is the ESP-IDF linker script's input
// section for IRAM text, the one IRAM_ATTR expands to. Host builds (tests,
// rigs) leave the attribute empty.
#if defined(ESP_PLATFORM)
#define MATDOG_MOTION_HOT __attribute__((section(".iram1.motion_hot")))
#else
#define MATDOG_MOTION_HOT
#endif
