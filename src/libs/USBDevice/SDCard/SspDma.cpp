#include "SspDma.h"
#include "cmsis_nvic.h"

#include "FreeRTOS.h"
#include "task.h"

#include <cstdint>

namespace {

const uint32_t AHB_START = 0x2007C000, AHB_END = 0x20084000;
const UBaseType_t NOTIFY_INDEX = 2;
const TickType_t TIMEOUT = pdMS_TO_TICKS(20);

// DMACCControl
const uint32_t BURST_4 = 1, WIDTH_BYTE = 0;
const uint32_t SI = 1u << 26, DI = 1u << 27, TC_IRQ = 1u << 31;
// DMACCConfig
const uint32_t ENABLE = 1, M2P = 1u << 11, P2M = 2u << 11, ITC = 1u << 15, ACTIVE = 1u << 17,
               HALT = 1u << 18;
// SSP
const uint32_t RXDMAE = 1, TXDMAE = 2, SR_RNE = 1u << 2, SR_BSY = 1u << 4, RORRIS = 1, RORIC = 1;

uint8_t fill __attribute__((section("AHBSRAM")));
uint8_t drop __attribute__((section("AHBSRAM")));

volatile TaskHandle_t waiter = nullptr;

bool ready = false;

uint32_t control(size_t n, uint32_t increment)
{
    return n | BURST_4 << 12 | BURST_4 << 15 | WIDTH_BYTE << 18 | WIDTH_BYTE << 21 | increment;
}

void irq()
{
    if (!(LPC_GPDMA->DMACIntTCStat & 1) && !(LPC_GPDMA->DMACIntErrStat & 3))
        return;

    LPC_GPDMA->DMACIntTCClear = 3;
    LPC_GPDMA->DMACIntErrClr = 3;
    BaseType_t woken = pdFALSE;
    if (waiter != nullptr)
        vTaskNotifyGiveIndexedFromISR(waiter, NOTIFY_INDEX, &woken);

    portYIELD_FROM_ISR(woken);
}

void setup()
{
    taskENTER_CRITICAL();
    if (!ready) {
        LPC_SC->PCONP |= 1u << 29;
        LPC_GPDMA->DMACConfig = 1;
        NVIC_SetVector(DMA_IRQn, (uint32_t)&irq);
        // syscall level: it wakes a task itself
        NVIC_SetPriority(DMA_IRQn,
                         configMAX_SYSCALL_INTERRUPT_PRIORITY >> (8 - __NVIC_PRIO_BITS));
        NVIC_EnableIRQ(DMA_IRQn);
        ready = true;
    }
    taskEXIT_CRITICAL();
}

// lets the channel hand on what it holds
void halt(LPC_GPDMACH_TypeDef* ch)
{
    ch->DMACCConfig |= HALT;
    for (int i = 0; (ch->DMACCConfig & ACTIVE) && i < 10000; i++) {}
    ch->DMACCConfig = 0;
}

bool usable(const void* p, size_t n)
{
    uint32_t a = (uint32_t)p;
    return n >= 64 && n < 4096 && a >= AHB_START && a + n <= AHB_END && __get_IPSR() == 0
           && xTaskGetSchedulerState() == taskSCHEDULER_RUNNING;
}

}

namespace SspDma {

size_t exchange(LPC_SSP_TypeDef* ssp, const char* out, char* in, size_t n, bool& lost)
{
    lost = false;
    if (!usable(out ? (const void*)out : in, n))
        return 0;

    if (!ready)
        setup();

    uint32_t tx_peripheral = ssp == LPC_SSP0 ? 0 : 2;
    LPC_GPDMACH_TypeDef* rx = LPC_GPDMACH0;
    LPC_GPDMACH_TypeDef* tx = LPC_GPDMACH1;
    LPC_GPDMA->DMACIntTCClear = 3;
    LPC_GPDMA->DMACIntErrClr = 3;
    fill = 0xFF;

    // the receive channel ranks first, and its end is the block's
    rx->DMACCSrcAddr = (uint32_t)&ssp->DR;
    rx->DMACCDestAddr = in ? (uint32_t)in : (uint32_t)&drop;
    rx->DMACCLLI = 0;
    rx->DMACCControl = control(n, in ? DI : 0) | TC_IRQ;
    tx->DMACCSrcAddr = out ? (uint32_t)out : (uint32_t)&fill;
    tx->DMACCDestAddr = (uint32_t)&ssp->DR;
    tx->DMACCLLI = 0;
    tx->DMACCControl = control(n, out ? SI : 0);

    waiter = xTaskGetCurrentTaskHandle();
    ulTaskNotifyTakeIndexed(NOTIFY_INDEX, pdTRUE, 0);
    rx->DMACCConfig = ENABLE | (tx_peripheral + 1) << 1 | P2M | ITC;
    tx->DMACCConfig = ENABLE | tx_peripheral << 6 | M2P;
    ssp->DMACR = RXDMAE | TXDMAE;

    bool done = ulTaskNotifyTakeIndexed(NOTIFY_INDEX, pdTRUE, TIMEOUT) != 0
                && !(LPC_GPDMA->DMACEnbldChns & 1) && !(LPC_GPDMA->DMACRawIntErrStat & 3);
    waiter = nullptr;
    if (done) {
        ssp->DMACR = 0;
        rx->DMACCConfig = 0;
        tx->DMACCConfig = 0;
        return n;
    }

    // stalled: how far it got counts on the memory side; the caller sends the rest
    halt(tx);
    halt(rx);
    ssp->DMACR = 0;
    for (int i = 0; (ssp->SR & SR_BSY) && i < 100000; i++) {}
    size_t moved = in ? rx->DMACCDestAddr - (uint32_t)in : tx->DMACCSrcAddr - (uint32_t)out;
    while (ssp->SR & SR_RNE) {
        char c = (char)ssp->DR;
        if (in && moved < n)
            in[moved++] = c;
    }
    lost = in && (ssp->RIS & RORRIS);
    ssp->ICR = RORIC;
    return moved;
}

}
