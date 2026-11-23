// comm.h - UART, comm_text.c or comm_binary.c
#ifndef COMM_H
#define COMM_H

#include "config.h"

void Comm_Init(void);
void Comm_Poll(void);
void Comm_SendStatus(void);

#if USE_BINARY_COMM
void Comm_SendInputs(void);     // from inputs.c, each 32 Hz sample while external
#else
#define Comm_SendInputs()
#endif

#endif
