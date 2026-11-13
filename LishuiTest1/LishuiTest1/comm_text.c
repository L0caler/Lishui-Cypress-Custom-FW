// comm_text.c - text terminal
#include <m8c.h>
#include "PSoCAPI.h"
#include "config.h"

#if !USE_BINARY_COMM
#include "bldc.h"
#include "PAS.h"
#include "sense.h"
#include "limit.h"
#include "inputs.h"
#include "comm.h"

#define CURRENT_REL_STEP 4

static void putDec(uint16_t w)
{
    static const uint16_t awPow[4] = { 10000, 1000, 100, 10 };
    uint8_t i, d, lead = 1;

    for (i = 0; i < 4; i++) {
        d = '0';
        while (w >= awPow[i]) {
            w -= awPow[i];
            d++;
        }
        if (d != '0' || !lead) {
            UART_PutChar(d);
            lead = 0;
        }
    }
    UART_PutChar('0' + (uint8_t)w);
}

// Stack high-water mark: the stack grows from the globals to 0xFF.
// Fills from just above stackFill's own frame.
#define STK_PAT  0x5A
static uint8_t bStkLo;
static void stackFill(void)
{
    uint8_t a = (uint8_t)(uint16_t)&a + 4;
    bStkLo = a;
    do { *(uint8_t *)a = STK_PAT; } while (++a);
}
static uint8_t stackFree(void)
{
    uint8_t a = 0xFF;
    while (a != bStkLo && *(uint8_t *)a == STK_PAT) a--;
    return 0xFF - a;
}

static void putLim(uint8_t f)
{
    if (!f) return;
    UART_CPutString(" LIM");
    if (f & BLDC_LIM_BAT)  UART_CPutString(" bat");
    if (f & BLDC_LIM_PH)   UART_CPutString(" ph");
    if (f & BLDC_LIM_W)    UART_CPutString(" W");
    if (f & BLDC_LIM_VBAT) UART_CPutString(" vbat");
    if (f & BLDC_LIM_ERPM) UART_CPutString(" erpm");
}

void Comm_Init(void)
{
    stackFill();
    UART_Start(UART_PARITY_NONE);
    UART_IntCntl(UART_DISABLE_RX_INT);  // polled one byte at a time
    UART_CPutString("\r\n*** RESET *** L0caler BLDC\r\n");
    UART_CPutString("g=go s=stop +/-=current x=ext/int a/d=bat lim j/k=ph lim\r\n");
}

void Comm_SendStatus(void)
{
    if (BLDC_bIsEnabled()) UART_CPutString("run "); else UART_CPutString("stop");
    UART_CPutString(" duty ");
    putDec(Live.bDuty);
    UART_PutChar('/');
    putDec(Cfg.bDutyMax);
    UART_CPutString(" I ");
    if (Live.iIbat_mA < 0) {
        UART_PutChar('-');
        putDec(-Live.iIbat_mA);
    } else {
        putDec(Live.iIbat_mA);
    }
    UART_CPutString("mA V ");
    putDec(Live.wVbat_mV);
    UART_CPutString("mV P ");
    putDec(Live.wPbat_W);
    UART_CPutString("W erpm ");
    putDec(Live.wErpm);
    UART_CPutString(" Iph ");
    putDec(Live.wIph_mA);
    UART_CPutString("mA lim ");
    putDec(Cfg.wIbatMax_mA);
    UART_CPutString("/");
    putDec(Cfg.wIphMax_mA);
    UART_CPutString("mA ceil ");
    putDec(BLDC_bGetCeil());
    putLim(BLDC_bLimiting());
    if (BLDC_bHallFault()) UART_CPutString(" HALL FAULT");
    UART_CPutString(" | i ");
    putDec(Live.wIraw);
    UART_CPutString(" z ");
    putDec(Live.wIzero);
    UART_CPutString(" v ");
    putDec(Live.wVraw);
    UART_CPutString(" stk ");
    putDec(stackFree());
    UART_CPutString(" rel ");
    putDec(BLDC_bGetCurrentRel());
    if (Cfg.bExternalRel) UART_CPutString(" ext"); else UART_CPutString(" int");
    UART_CPutString(" pas ");
    putDec(PAS_wGetRpm());
    UART_CPutString(" pasPin ");
    putDec((PAS_DR & PAS_MASK) ? 1 : 0);
    UART_CPutString(" pasAct ");
    putDec(PAS_bIsActive());
    UART_CPutString(" thr ");
    putDec(Inputs_bThrottle());
    UART_CPutString(" brk ");
    putDec(Inputs_bBrake());
    UART_PutCRLF();
}

void Comm_Poll(void)
{
    static char c;
    static uint8_t rel;

    c = UART_cReadChar();
    if (c == 0) return;
    rel = BLDC_bGetCurrentRel();
    switch (c) {
    case 'g':
        if (!BLDC_bIsEnabled()) Sense_Zero();
        Limit_Update();
        BLDC_Enable(1);
        break;
    case 's':
        BLDC_Enable(0);
        break;
    case '+':
        if (!Cfg.bExternalRel) return;
        if (rel > 255 - CURRENT_REL_STEP) rel = 255;
        else rel += CURRENT_REL_STEP;
        BLDC_SetCurrentRel(rel);
        break;
    case '-':
        if (!Cfg.bExternalRel) return;
        if (rel < CURRENT_REL_STEP) rel = 0;
        else rel -= CURRENT_REL_STEP;
        BLDC_SetCurrentRel(rel);
        break;
    case 'x':
        Cfg.bExternalRel = !Cfg.bExternalRel;
        BLDC_SetCurrentRel(0);
        break;
    case 'a':
        if (Cfg.wIbatMax_mA > 100) Cfg.wIbatMax_mA -= 100;
        Limit_Update();
        break;
    case 'd':
        if (Cfg.wIbatMax_mA < IBAT_MAX_LIMIT_MA) Cfg.wIbatMax_mA += 100;
        Limit_Update();
        break;
    case 'j':
        if (Cfg.wIphMax_mA > 100) Cfg.wIphMax_mA -= 100;
        Limit_Update();
        break;
    case 'k':
        if (Cfg.wIphMax_mA < IPH_MAX_LIMIT_MA) Cfg.wIphMax_mA += 100;
        Limit_Update();
        break;
    default:
        return;
    }
    UART_PutChar(c);                    // echo: command received
    UART_PutCRLF();
}
#endif
