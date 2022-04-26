#include "../esc.hpp"
#include <Arduino.h>
#include "imxrt.h"

// cmp output mask
auto fallingMask = 0B00000000;
auto risingMask = 0B00000001;

// volatile uint8_t offset03;
void ESC::debounce(volatile uint8_t &CMPSCRRegister)
{
    for (int i = 0; i < DEBOUNCE_DISTANCE; i++)
    {
        if (ELECTRICAL_STEP_CTR & 1)
        { // odd means falling
            if ((CMPSCRRegister & fallingMask) != fallingMask)
            {
                i -= 1;
            }
        }
        else
        { // rising
            if ((CMPSCRRegister & risingMask) != risingMask)
            {
                i -= 1;
            }
        }
    }
    CMPSCRRegister &= 0x00; // turn off falling or rising
};

// handleZeroCrossing
void ESC::acmp1_isr(){
    // debounce(CMP1_SCR); not gonna work because static
    // get instance 
    ESC instance = ESC::get_instance();
    instance.debounce(CMP1_SCR);
    // wait to trigger zero cross
};
void ESC::acmp2_isr(){
    ESC instance = ESC::get_instance();
    instance.debounce(CMP2_SCR);
    // wait to trigger zero cross
};
void ESC::acmp3_isr(){
    ESC instance = ESC::get_instance();
    instance.debounce(CMP3_SCR);
    // wait to trigger zero cross
};

void ESC::initACMP()
{

    // make sure the cycle counter register is active
    ARM_DWT_CTRL |= ARM_DWT_CTRL_CYCCNTENA;

    CCM_CCGR3 |= CCM_CCGR3_ACMP1(CCM_CCGR_ON) | CCM_CCGR3_ACMP2(CCM_CCGR_ON) | CCM_CCGR3_ACMP3(CCM_CCGR_ON);

    attachInterruptVector(IRQ_ACMP1, acmp1_isr);
    attachInterruptVector(IRQ_ACMP2, acmp2_isr);
    attachInterruptVector(IRQ_ACMP3, acmp3_isr);

    // 0 - [x - x - x] - 0 - 0 - [x - x]
    // 0 - [FILTER_CNT] - 0 - 0 -[HYSTCTR]
    CMP1_CR0 = 0B00000000; // no filtering 5mV hysteresis
    CMP2_CR0 = 0B00000000; // no filtering 5mV hysteresis
    CMP3_CR0 = 0B00000000; // no filtering 5mV hysteresis

    // DACCR - disable
    CMP1_DACCR = 0x00;
    CMP2_DACCR = 0x00;
    CMP3_DACCR = 0x00;

    // (CMPx_FPR)
    // Specifies the sampling period, in bus clock cycles, of the comparator output filter, when CR1[SE]=0.
    // Setting FILT_PER to 0x0 disables the filter.
    CMP1_FPR = 0;
    CMP2_FPR = 0;
    CMP3_FPR = 0;

    // MUX

    //  [0] - 0 - [PSEL] - [MSEL]
    // 0 - 0 - [x - x - x] - [x - x - x]
    /* MSEL / PSEL
      000 IN0
      001 IN1
      010 IN2
      011 IN3
      100 IN4
      101 IN5
      110 IN6
      111 IN7
    */
    // so for negative [msel] we have the virtual netural pin 18 which is IN0 for each acmp
    // x-x-x
    // 0-0-0

    // for positive [psel] we have phase A,B,C -> pin 21,22,23 ->

    // pin 21 is acmp1_in6 110
    CMP1_MUXCR = 0B00110000; // set + to pin 21 (IN6) - to pin 18 (IN0) [phaseA - red]

    // pin 22 is acmp2_in5 101
    CMP2_MUXCR = 0B00101000; // set + to pin 22 (IN5) - to pin 18 (IN0) [phaseB - yellow]

    // pin 23 is acmp3_in5 101
    CMP3_MUXCR = 0B00101000; // set + to pin 23 (IN5) - to pin 18 (IN0)   [phaseC - black]

    // CMPx_CR1
    // 65.3.2 CMP Control Register 1 (CMPx_CR1)
    // x - x - x - x - x - x - x - x
    // [SE] - [WE] - [x] - [PMODE] - [INVERT] - [COS] - [OPE] - [EN]
    CMP1_CR1 = 0B01010001; // set high speed and power on, window mode enabled
    CMP2_CR1 = 0B01010001;
    CMP3_CR1 = 0B01010001;
}

void ESC::enableACMPInterrupts()
{
    NVIC_ENABLE_IRQ(IRQ_ACMP1);
    NVIC_ENABLE_IRQ(IRQ_ACMP2);
    NVIC_ENABLE_IRQ(IRQ_ACMP3);
};

void ESC::disableACMPInterrupts()
{
    NVIC_DISABLE_IRQ(IRQ_ACMP1);
    NVIC_DISABLE_IRQ(IRQ_ACMP2);
    NVIC_DISABLE_IRQ(IRQ_ACMP3);
};