#ifndef SSPDMA_H
#define SSPDMA_H

#include "LPC17xx.h"

#include <cstddef>

// A block between memory and an SSP by GPDMA channels 0 and 1, which reach only the AHB SRAM.
namespace SspDma {

// Returns how many went: n, fewer after a stall, 0 where DMA cannot go; the caller sends the
// rest. lost: received bytes were lost, the count is short.
size_t exchange(LPC_SSP_TypeDef* ssp, const char* out, char* in, size_t n, bool& lost);

}

#endif
