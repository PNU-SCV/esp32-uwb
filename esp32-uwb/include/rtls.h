#ifndef RTLS_H
#define RTLS_H 

#include <SPI.h>
#include <DW1000Ranging.h>
#include <WiFi.h>
#include "link.h"
#include "rasp.h"

void newRange();

void newDevice(DW1000Device *device);

void inactiveDevice(DW1000Device *device);

void send_udp(String *msg_json);

#endif