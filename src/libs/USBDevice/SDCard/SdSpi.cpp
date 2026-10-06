#include "SdSpi.h"
#include "lpc17xx_ssp.h"
#include "SspDma.h"

// at most 8 in flight, the receive FIFO's depth; the caller's write() leaves it empty
bool SdSpi::exchange(const char *out, char *in, int n)
{
    LPC_SSP_TypeDef *ssp = _spi.spi;
    // after a loss the count is short: clocking past the block's end is harmless
    bool lost;
    int done = SspDma::exchange(ssp, out, in, n, lost);
    int sent = done, got = done;
    while (got < n) {
        while (sent < n && sent - got < 8 && (ssp->SR & SSP_SR_TNF)) {
            ssp->DR = out ? (uint8_t)out[sent] : 0xFF;
            sent++;
        }
        while (got < sent && (ssp->SR & SSP_SR_RNE)) {
            char c = (char)ssp->DR;
            if (in)
                in[got] = c;

            got++;
        }
    }
    return !lost;
}
