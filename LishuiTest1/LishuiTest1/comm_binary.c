// comm_binary.c - binary frames, little endian, checksum = sum of all bytes before it
// In:  AA 55 tCfg(16) checksum               config, only while stopped; enables
//      AA 56 currentRel checksum             current command, only while external
//      AA 57 source ~source checksum         command source: 0 = inputs.c, 1 = UART
//      AA 5A limits(15) checksum ~checksum   tCfg without bExternalRel, any time;
//                                            only sets the limits, answered by AA 59
// Out: AA 55 enabled duty limiting hallFault Ibat(2) Vbat(2) erpm(2) Iph(2)
//      currentRel pasActive pasRpm(2) throttle brake stackFree checksum   4 Hz
//      AA 58 throttle brake pasActive checksum     32 Hz, only while external,
//                                                  right after each throttle sample
//      AA 59 limitsChecksum checksum               limits frame applied
// While external, no AA 56 for COMMAND_TIMEOUT_TICKS switches back to inputs.c.
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

#define UART_RX_RING_SIZE       8       // lib/UARTINT.asm UART_RX_RING_SIZE
#define UART_RX_RING_MASK       (UART_RX_RING_SIZE - 1)
#define CONFIG_FRAME_SIZE       19
#define CURRENT_REL_FRAME_SIZE  4
#define SOURCE_FRAME_SIZE       5
#define LIMITS_FRAME_SIZE       19      // = CONFIG_FRAME_SIZE, fits rxFrame
#define CONFIG_FRAME_TYPE       0x55
#define CURRENT_REL_FRAME_TYPE  0x56
#define SOURCE_FRAME_TYPE       0x57
#define INPUTS_FRAME_TYPE       0x58
#define LIMITS_ACK_FRAME_TYPE   0x59
#define LIMITS_FRAME_TYPE       0x5A

// UARTINT.asm RX ISR
extern volatile BYTE UART_aRxRing[UART_RX_RING_SIZE];
extern volatile BYTE UART_bRxHead;
extern volatile BYTE UART_bRxTail;
extern volatile BYTE UART_bRxOverflow;

static uint8_t rxFrame[CONFIG_FRAME_SIZE];
static uint8_t rxFramePos;
static uint8_t lastCommandTick;
static uint8_t commandTimeoutArmed;
static uint8_t txChecksum;

// Stack high-water mark, as in comm_text.c.  The stack grows from the end
// of the globals up to 0xFF, and past it wraps onto RAM 0x00, where
// bldc.c keeps bIrelTarget and bIrelSmoothed.  0 free = it may have wrapped.
#define STK_PAT  0x5A
static uint8_t bStkLo;
static void stackFill(void)
{
    uint8_t a = (uint8_t)(uint16_t)&a + 4;  // from just above this frame
    bStkLo = a;
    do { *(uint8_t *)a = STK_PAT; } while (++a);
}
static uint8_t stackFree(void)
{
    uint8_t a = 0xFF;
    while (a != bStkLo && *(uint8_t *)a == STK_PAT) a--;
    return 0xFF - a;
}

void Comm_Init(void)
{
    stackFill();
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
    putByte(stackFree());
    UART_PutChar(txChecksum);
}

// Everything inputs.c uses to set the current, so the external controller
// can compute exactly the same thing.  ~3 ms at 19200 baud.
void Comm_SendInputs(void)
{
    txChecksum = 0;
    putByte(0xAA);
    putByte(INPUTS_FRAME_TYPE);
    putByte(Inputs_bThrottle());
    putByte(Inputs_bBrake());
    putByte(PAS_bIsActive() ? 1 : 0);
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

// The limits part of tCfg, rxFrame[2..16], with the same clamps for both
// frames.  Main loop only, like every other reader of these fields; the ISR
// only reads bDutyMax, a single byte.
//
// No Limit_Update() here: main() runs it on every ERPM update, 8 times a
// second, so the new limits are in the PI within 125 ms.  Calling it from
// down here added two call levels to its 32 bit maths, the deepest stack
// path, and the stack has only ~70 bytes before it wraps onto RAM 0x00.
static void applyLimits(void)
{
    Cfg.wPmax_W = readLe16(2);
    Cfg.bDutyMax = rxFrame[4];
    Cfg.wIbatMax_mA = readLe16(5);
    Cfg.wIphMax_mA = readLe16(7);
    Cfg.wErpmMax = readLe16(9);
    Cfg.wErpmLimStart = readLe16(11);
    Cfg.wVbatMin_mV = readLe16(13);
    Cfg.wVbatLimStart_mV = readLe16(15);

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
}

static void applyConfigFrame(void)
{
    if (!checksumOk(CONFIG_FRAME_SIZE) || BLDC_bIsEnabled()) return;

    applyLimits();
    Limit_Update();     // as before: the motor is stopped, it is enabled below
    Cfg.bExternalRel = rxFrame[17] ? 1 : 0;
    Sense_Zero();
    BLDC_Enable(1);
    refreshCommandTimeout();
}

// Limits only, also while running: no current zeroing, no enable, the
// command source stays.  Changing the limits while riding needs more than
// the 1 in 256 of a checksum against a noisy RX line, hence the complement.
static void applyLimitsFrame(void)
{
    if (!checksumOk(LIMITS_FRAME_SIZE - 1)) return;
    if (rxFrame[LIMITS_FRAME_SIZE - 1] != (uint8_t)(rxFrame[LIMITS_FRAME_SIZE - 2] ^ 0xFF)) return;

    applyLimits();

    txChecksum = 0;
    putByte(0xAA);
    putByte(LIMITS_ACK_FRAME_TYPE);
    putByte(rxFrame[LIMITS_FRAME_SIZE - 2]);
    UART_PutChar(txChecksum);
}

static void applyCurrentRelFrame(void)
{
    if (!checksumOk(CURRENT_REL_FRAME_SIZE) || !Cfg.bExternalRel) return;
    // brake pulled (bBrake 0): no current, whatever the UART says
    BLDC_SetCurrentRel(Inputs_bBrake() ? rxFrame[2] : 0);
    refreshCommandTimeout();
}

// The second byte must be the complement of the first, so that noise on an
// unconnected RX line is unlikely to switch the source.
static void applySourceFrame(void)
{
    if (!checksumOk(SOURCE_FRAME_SIZE)) return;
    if (rxFrame[2] > 1 || rxFrame[3] != (uint8_t)(rxFrame[2] ^ 0xFF)) return;

    Cfg.bExternalRel = rxFrame[2];
    if (Cfg.bExternalRel) refreshCommandTimeout();
    else commandTimeoutArmed = 0;
}

// The external controller went quiet: hand the current back to inputs.c,
// which sets it again from throttle, PAS and brake on its next 32 Hz sample.
static void checkCommandTimeout(void)
{
    if (!Cfg.bExternalRel || !commandTimeoutArmed) return;
    if ((uint8_t)(BLDC_bGetTick() - lastCommandTick) < COMMAND_TIMEOUT_TICKS) return;

    commandTimeoutArmed = 0;
    Cfg.bExternalRel = 0;
}

static uint8_t frameSize(uint8_t type)
{
    if (type == CONFIG_FRAME_TYPE) return CONFIG_FRAME_SIZE;
    if (type == CURRENT_REL_FRAME_TYPE) return CURRENT_REL_FRAME_SIZE;
    if (type == SOURCE_FRAME_TYPE) return SOURCE_FRAME_SIZE;
    if (type == LIMITS_FRAME_TYPE) return LIMITS_FRAME_SIZE;
    return 0;
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
            if (frameSize(c)) {
                rxFrame[rxFramePos++] = c;
            } else {
                rxFramePos = (c == 0xAA) ? 1 : 0;
            }
        } else {
            rxFrame[rxFramePos++] = c;
            if (rxFramePos == frameSize(rxFrame[1])) {
                if (rxFrame[1] == CONFIG_FRAME_TYPE) applyConfigFrame();
                else if (rxFrame[1] == CURRENT_REL_FRAME_TYPE) applyCurrentRelFrame();
                else if (rxFrame[1] == SOURCE_FRAME_TYPE) applySourceFrame();
                else applyLimitsFrame();
                rxFramePos = 0;
            }
        }
    }
}
#endif
