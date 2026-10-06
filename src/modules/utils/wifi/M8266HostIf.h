#ifndef _M8266_HOST_IF_H_
#define _M8266_HOST_IF_H_

#include "LPC17xx.h"
#include "M8266WIFIDrv.h"

// The M8266 on the Carvera: SSP1 (P0.7 SCK, P0.8 MISO, P0.9 MOSI), P0.6 as nCS, P2.10 as
// nRESET. The driver's own hooks (pins, delays, the byte exchange) are in M8266WIFIDrv.h.
#define M8266WIFI_INTERFACE_SPI LPC_SSP1

void M8266HostIf_Init(void);
void M8266HostIf_SPI_SetSpeed(u32 prescaler);   // SSP clock = CPU clock / prescaler, even

#endif
