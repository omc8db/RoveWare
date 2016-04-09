// RoveComm.h
// Author: Gbenga Osibodu

#ifndef ROVECOMM_H
#define ROVECOMM_H

#include "../RoveBoard/RoveEthernet.h"

#include <stdint.h>

void RoveComm_begin(uint8_t ip_octet1, uint8_t ip_octet2, uint8_t ip_octet3, uint8_t ip_octet4);
void RoveComm_read(uint16_t* data_id, size_t* size, void* data);
void RoveComm_write(uint16_t data, size_t size, const void* data);

#endif

