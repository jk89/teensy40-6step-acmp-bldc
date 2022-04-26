#include "../esc.hpp"
#include <Arduino.h>
#include "imxrt.h"

void ESC::initPWM()
{
    analogWriteRes(8);
    analogWriteFrequency(SD_PINS[0], PWM_FREQUENCY); // set sm0 freq
    // pwm 1.0
    FLEXPWM1_SM0TCTRL = FLEXPWM_SMTCTRL_OUT_TRIG_EN(1 << 4); // enable trig 4 as pwm out 1 for sm0
    // pwm 1.1
    FLEXPWM1_MCTRL |= FLEXPWM_MCTRL_CLDOK(0x0F);                                                          //  Clear Load Okay LDOK(SM) -> no reload of PWM settings
    FLEXPWM1_SM1CTRL2 = FLEXPWM_SMCTRL2_INDEP | FLEXPWM_SMCTRL2_CLK_SEL(2) | FLEXPWM_SMCTRL2_INIT_SEL(0); // A & B independant | sm0 chosen as clock
    FLEXPWM1_MCTRL |= FLEXPWM_MCTRL_LDOK(0x0F);                                                           // Load Okay LDOK(SM) -> reload setting again
    FLEXPWM1_SM1TCTRL = FLEXPWM_SMTCTRL_OUT_TRIG_EN(1 << 4);                                              // enable trig 4 as pwm out 1 for sm1
    // pwm 1.3
    FLEXPWM1_MCTRL |= FLEXPWM_MCTRL_CLDOK(0x0F);                                                          //  Clear Load Okay LDOK(SM) -> no reload of PWM settings
    FLEXPWM1_SM3CTRL2 = FLEXPWM_SMCTRL2_INDEP | FLEXPWM_SMCTRL2_CLK_SEL(2) | FLEXPWM_SMCTRL2_INIT_SEL(0); // A & B independant | sm0 chosen as clock
    FLEXPWM1_MCTRL |= FLEXPWM_MCTRL_LDOK(0x0F);                                                           // Load Okay LDOK(SM) -> reload setting again
    FLEXPWM1_SM3TCTRL = FLEXPWM_SMTCTRL_OUT_TRIG_EN(1 << 4);                                              // enable trig 4 as pwm out 1 for sm3
};