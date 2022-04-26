#include "../esc.hpp"
#include <Arduino.h>
#include "imxrt.h"

void ESC::xbarInit()
{
    CCM_CCGR2 |= CCM_CCGR2_XBAR1(CCM_CCGR_ON);                               // turn clock on for xbara1
    xbarConnect(XBARA1_IN_FLEXPWM1_PWM1_OUT_TRIG0, XBARA1_OUT_ACMP1_SAMPLE); // pwm module 1.0 to acmp 1 sample
    xbarConnect(XBARA1_IN_FLEXPWM1_PWM1_OUT_TRIG0, XBARA1_OUT_ACMP2_SAMPLE); // pwm module 1.0 to acmp 2 sample
    xbarConnect(XBARA1_IN_FLEXPWM1_PWM1_OUT_TRIG0, XBARA1_OUT_ACMP3_SAMPLE); // pwm module 1.0 to acmp 3 sample
};

void ESC::xbarConnect(unsigned int input, unsigned int output)
{
    if (input >= 88)
        return;
    if (output >= 132)
        return;
    volatile uint16_t *xbar = &XBARA1_SEL0 + (output / 2);
    uint16_t val = *xbar;
    if (!(output & 1))
    {
        val = (val & 0xFF00) | input;
    }
    else
    {
        val = (val & 0x00FF) | (input << 8);
    }
    *xbar = val;
};