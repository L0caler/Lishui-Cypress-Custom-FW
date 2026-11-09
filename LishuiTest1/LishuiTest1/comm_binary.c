// comm_binary.c - binary frames, little endian, checksum = sum of all bytes before it
// In:  AA 55 tCfg(16) checksum
//      AA 56 currentRel checksum
// Out: AA 55 enabled duty limiting hallFault Ibat(2) Vbat(2) erpm(2) Iph(2)
//      currentRel pasActive pasRpm(2) throttle brake checksum
#include <m8c.h>
#include "PSoCAPI.h"
#include "config.h"

#if USE_BINARY_COMM
#include "bldc.h"
#include "PAS.h"
#include "sense.h"
#include "limit.h"
#include "inputs.h"
#include "comm.h"

#define UART_RX_RING_MASK       31
#define CONFIG_FRAME_SIZE       19
#define CURRENT_REL_FRAME_SIZE  4
#define CONFIG_FRAME_TYPE       0x55
#define CURRENT_REL_FRAME_TYPE  0x56

// UARTINT.asm RX ISR
extern volatile BYTE UART_aRxRing[32];
extern volatile BYTE UART_bRxHead;
extern volatile BYTE UART_bRxTail;
extern volatile BYTE UART_bRxOverflow;

static uint8_t rxFrame[CONFIG_FRAME_SIZE];
static uint8_t rxFramePos;
static uint8_t lastCommandTick;
static uint8_t commandTimeoutArmed;
static uint8_t txChecksum;

void Comm_Init(void)
{
    UART_Start(UART_PARITY_NONE);
    UART_bRxHead = UART_bRxTail = UART_bRxOverflow = 0;
    UART_IntCntl(UART_ENABLE_RX_INT);
    UART_CPutString("\r\n*** RESET *** L0caler BLDC\r\n");
}

static void putByte(uint8_t b)
{
    UART_PutChar(b);
    txChecksum += b;
}

static void putWord(uint16_t w)
{
    putByte((uint8_t)w);
    putByte((uint8_t)(w >> 8));
}

void Comm_SendStatus(void)
{
    txChecksum = 0;
    putByte(0xAA);
    putByte(0x55);
    putByte(BLDC_bIsEnabled() ? 1 : 0);
    putByte(Live.bDuty);
    putByte(BLDC_bLimiting());
    putByte(BLDC_bHallFault() ? 1 : 0);
    putWord((uint16_t)Live.iIbat_mA);
    putWord(Live.wVbat_mV);
    putWord(Live.wErpm);
    putWord(Live.wIph_mA);
    putByte(BLDC_bGetCurrentRel());
    putByte(PAS_bIsActive() ? 1 : 0);
    putWord(PAS_wGetRpm());
    putByte(Inputs_bThrottle());
    putByte(Inputs_bBrake());
    UART_PutChar(txChecksum);
}

static uint8_t rxPop(uint8_t *c)
{
    if (UART_bRxTail == UART_bRxHead) return 0;
    *c = UART_aRxRing[UART_bRxTail];
    UART_bRxTail = (UART_bRxTail + 1) & UART_RX_RING_MASK;
    return 1;
}

static void refreshCommandTimeout(void)
{
    lastCommandTick = BLDC_bGetTick();
    commandTimeoutArmed = 1;
}

static uint16_t readLe16(uint8_t offset)
{
    return (uint16_t)rxFrame[offset] | ((uint16_t)rxFrame[offset + 1] << 8);
}

static uint8_t checksumOk(uint8_t size)
{
    uint8_t i, sum = 0;

    for (i = 0; i < size - 1; ++i) sum += rxFrame[i];
    return sum == rxFrame[size - 1];
}

static void applyConfigFrame(void)
{
    if (!checksumOk(CONFIG_FRAME_SIZE) || BLDC_bIsEnabled()) return;

    Cfg.wPmax_W = readLe16(2);
    Cfg.bDutyMax = rxFrame[4];
    Cfg.wIbatMax_mA = readLe16(5);
    Cfg.wIphMax_mA = readLe16(7);
    Cfg.wErpmMax = readLe16(9);
    Cfg.wErpmLimStart = readLe16(11);
    Cfg.wVbatMin_mV = readLe16(13);
    Cfg.wVbatLimStart_mV = readLe16(15);
    Cfg.bExternalRel = rxFrame[17] ? 1 : 0;

    if (Cfg.bDutyMax > BLDC_DUTY_MAX)
        Cfg.bDutyMax = BLDC_DUTY_MAX;
    if (Cfg.wIbatMax_mA > IBAT_MAX_LIMIT_MA)
        Cfg.wIbatMax_mA = IBAT_MAX_LIMIT_MA;
    if (Cfg.wIphMax_mA > IPH_MAX_LIMIT_MA)
        Cfg.wIphMax_mA = IPH_MAX_LIMIT_MA;
    if (Cfg.wErpmLimStart > Cfg.wErpmMax)
        Cfg.wErpmLimStart = Cfg.wErpmMax;
    if (Cfg.wVbatLimStart_mV <= Cfg.wVbatMin_mV)
        Cfg.wVbatMin_mV = Cfg.wVbatLimStart_mV ? Cfg.wVbatLimStart_mV - 1 : 0;

    Limit_Update();
    Sense_Zero();
    BLDC_Enable(1);
    refreshCommandTimeout();
}

static void applyCurrentRelFrame(void)
{
    if (!checksumOk(CURRENT_REL_FRAME_SIZE) || !Cfg.bExternalRel) return;
    BLDC_SetCurrentRel(rxFrame[2]);
    refreshCommandTimeout();
}

static void checkCommandTimeout(void)
{
    if (!Cfg.bExternalRel || !commandTimeoutArmed || !BLDC_bIsEnabled()) return;
    if ((uint8_t)(BLDC_bGetTick() - lastCommandTick) < COMMAND_TIMEOUT_TICKS) return;

    commandTimeoutArmed = 0;
    BLDC_Enable(0);
}

void Comm_Poll(void)
{
    static uint8_t c;

    checkCommandTimeout();

    if (UART_bRxOverflow) {
        UART_bRxOverflow = 0;
        UART_bRxTail = UART_bRxHead;
        rxFramePos = 0;
        return;
    }

    while (rxPop(&c)) {
        if (rxFramePos == 0) {
            if (c == 0xAA) rxFrame[rxFramePos++] = c;
        } else if (rxFramePos == 1) {
            if (c == CONFIG_FRAME_TYPE || c == CURRENT_REL_FRAME_TYPE) {
                rxFrame[rxFramePos++] = c;
            } else {
                rxFramePos = (c == 0xAA) ? 1 : 0;
            }
        } else {
            rxFrame[rxFramePos++] = c;
            if (rxFrame[1] == CONFIG_FRAME_TYPE) {
                if (rxFramePos == CONFIG_FRAME_SIZE) {
                    applyConfigFrame();
                    rxFramePos = 0;
                }
            } else if (rxFramePos == CURRENT_REL_FRAME_SIZE) {
                applyCurrentRelFrame();
                rxFramePos = 0;
            }
        }
    }
}
#endif
