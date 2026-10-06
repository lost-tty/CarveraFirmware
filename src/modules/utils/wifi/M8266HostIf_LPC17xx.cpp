// The M8266 driver's host side on the LPC1768: the module's pins and SSP1, after the vendor's
// M8266HostIf.c (Anylinkin Technology).

#include "M8266HostIf.h"

#include "mbed.h"
#include "FreeRTOS.h"
#include "task.h"

namespace {
    const uint32_t NCS_PIN = 6;      // P0.6, also the ESP8266's GPIO15 boot strap
    const uint32_t NRESET_PIN = 10;  // P2.10
}

void M8266HostIf_Init(void)
{
    LPC_GPIO2->FIODIR |= 1UL << NRESET_PIN;
    LPC_GPIO2->FIOSET = 1UL << NRESET_PIN;
    LPC_GPIO0->FIODIR |= 1UL << NCS_PIN;
    LPC_GPIO0->FIOSET = 1UL << NCS_PIN;

    // P0.7 to P0.9 to SSP1, P0.6 stays a GPIO
    LPC_PINCON->PINSEL0 = (LPC_PINCON->PINSEL0 & ~(3UL << 12) & ~(3UL << 14) & ~(3UL << 16)
                           & ~(3UL << 18))
                          | (2UL << 14) | (2UL << 16) | (2UL << 18);

    // powered (PCONP bit 10), clocked at the CPU clock (PCLKSEL0 bits 21:20 = 01)
    LPC_SC->PCONP |= 1UL << 10;
    LPC_SC->PCLKSEL0 = (LPC_SC->PCLKSEL0 & ~(3UL << 20)) | (1UL << 20);

    // 8-bit SPI frames, CPOL 0, CPHA 0 (M8266HostIf_SPI_Select sets CPHA for fast clocks),
    // master, enabled; CPU clock / 8 until the speed is set
    M8266WIFI_INTERFACE_SPI->CR0 = 7;
    M8266WIFI_INTERFACE_SPI->CR1 = 1UL << 1;
    M8266WIFI_INTERFACE_SPI->CPSR = 8;
    M8266WIFI_INTERFACE_SPI->ICR = 3;
}

void M8266HostIf_SPI_SetSpeed(u32 prescaler)
{
    M8266WIFI_INTERFACE_SPI->CPSR = prescaler;
}

void M8266HostIf_Set_nRESET_Pin(u8 level)
{
    if (level)
        LPC_GPIO2->FIOSET = 1UL << NRESET_PIN;
    else
        LPC_GPIO2->FIOCLR = 1UL << NRESET_PIN;
}

void M8266HostIf_Set_SPI_nCS_Pin(u8 level)
{
    if (level)
        LPC_GPIO0->FIOSET = 1UL << NCS_PIN;
    else
        LPC_GPIO0->FIOCLR = 1UL << NCS_PIN;
}

void M8266HostIf_delay_us(u8 nus)
{
    uint32_t start = us_ticker_read();
    while (us_ticker_read() - start < nus)
        ;
}

// The driver's waits of a millisecond or more: the calling task sleeps.
void M8266HostIf_delay_ms(u16 nms)
{
    // the first tick may come at once
    vTaskDelay(pdMS_TO_TICKS(nms) + 1);
}

u32 M8266HostIf_now_us(void)
{
    return us_ticker_read();
}
