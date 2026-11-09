#include <m8c.h>
#include "bldc.h"
#include "sense.h"
#include "limit.h"

static uint16_t toCounts(uint16_t mA)
{
    return (uint16_t)(((uint32_t)mA << 8) / CS_MA_PER_COUNT_Q8);
}

// 255 at 'full', 0 at 'zero', linear between.  full == zero: off
static uint8_t derate(uint16_t x, uint16_t full, uint16_t zero)
{
    if (full == zero) return 255;
    if (full > zero) {                      // falling x lowers (Vbat)
        if (x >= full) return 255;
        if (x <= zero) return 0;
        return (uint8_t)((uint32_t)(x - zero) * 255 / (full - zero));
    }
    if (x <= full) return 255;              // rising x lowers (ERPM)
    if (x >= zero) return 0;
    return (uint8_t)((uint32_t)(zero - x) * 255 / (zero - full));
}

//Battery current target: min(Ibat max, Pmax / Vbat), then the Vbat and ERPM derate.  The phase term (Iph max * duty) is added by the ISR
void Limit_Update(void)
{
    static tIlim st;
    static uint32_t x;
    static uint16_t mA;
    static uint8_t s;

    mA = Cfg.wIbatMax_mA;
    st.bFlags = BLDC_LIM_BAT;
    if (Live.wVbat_mV) {
        x = (uint32_t)Cfg.wPmax_W * 1000000 / Live.wVbat_mV;
        if (x < mA) {
            mA = (uint16_t)x;
            st.bFlags = BLDC_LIM_W;
        }
    }

    st.bScale = derate(Live.wVbat_mV, Cfg.wVbatLimStart_mV, Cfg.wVbatMin_mV);
    if (st.bScale != 255) st.bFlags |= BLDC_LIM_VBAT;
    s = derate(Live.wErpm, Cfg.wErpmLimStart, Cfg.wErpmMax);
    if (s != 255) {
        st.bFlags |= BLDC_LIM_ERPM;
        if (s < st.bScale) st.bScale = s;
    }

    st.wZero = Live.wIzero;
    st.wBat = toCounts(mA);
    st.wPh = toCounts(Cfg.wIphMax_mA);
    BLDC_SetIlim(&st);
}
