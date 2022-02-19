#include "ESC.hpp"
#include <Arduino.h>
#include "imxrt.h"

// uint8_t

class ESC
{
private:
    inline static constexpr PinState antiClockwisePinState[] = {
        {PHASE_C, PHASE_B, PHASE_A, RISING},
        {PHASE_C, PHASE_A, PHASE_B, FALLING},
        {PHASE_B, PHASE_A, PHASE_C, RISING},
        {PHASE_B, PHASE_C, PHASE_A, FALLING},
        {PHASE_A, PHASE_C, PHASE_B, RISING},
        {PHASE_A, PHASE_B, PHASE_C, FALLING}};

    inline static constexpr PinState clockwisePinState[] = {
        {PHASE_A, PHASE_B, PHASE_C, RISING},
        {PHASE_A, PHASE_C, PHASE_B, FALLING},
        {PHASE_B, PHASE_C, PHASE_A, RISING},
        {PHASE_B, PHASE_A, PHASE_C, FALLING},
        {PHASE_C, PHASE_A, PHASE_B, RISING},
        {PHASE_C, PHASE_B, PHASE_A, FALLING}};

    // motor var
    char MOTOR_POLES = 14;

    // system var
    int PWM_FREQUENCY = 32000;
    char MIN_DUTY = 60;
    char MAX_DUTY = 240;

    bool STARTUP_MODE = true;
    bool LED_EN_ON = false;

    // Constants
    const char ELECTRICAL_CYCLE_MOD = 6;

    // Computed mechanical
    char ELECTRICAL_CYCLES_PER_MECHANICAL_CYCLE = (MOTOR_POLES / 2);
    char MECHANICAL_CYCLE_MOD = ELECTRICAL_CYCLES_PER_MECHANICAL_CYCLE * ELECTRICAL_CYCLE_MOD;
    float COMMUTATION_STEP_ANGLE = FULL_CYCLE / MECHANICAL_CYCLE_MOD;

    // Counters
    char ELECTRICAL_STEP_CTR = 0;
    int MECHANICAL_STEP_CTR = 0;
    int SPEED_STEP_CTR = 0;

    // float SIN_PRECOMP[MECHANICAL_CYCLE_MOD];
    // float COS_PRECOMP[MECHANICAL_CYCLE_MOD];

    // dynamics
    float RPM = 0;

    // Control
    bool clockwise = true;
    float ROLL = 0;
    float PITCH = 0;
    char THRUST = MIN_DUTY;
    char NEXT_DUTY = MIN_DUTY;

    char SD_PINS[3] = {1, 0, 7};
    char IN_PINS[3] = {2, 9, 8};
    int LED_PIN = 13;

    void incrementCommutation();
    void enforceCommutation();
    float computeSpeed();
    float angularDisplacement();
    bool readHostControlProfile();
    void setup();
    void startup();
    void controlLoop();
    void enableACMPInterrupts();
    void disableACMPInterrupts();
    void calculateNextCommutationStepDuty();
    void initPWM();
    void initACMP();
    void buildTrigTables();
    void xbarConnect(unsigned int input, unsigned int output);
    void xbarInit();

    static void acmp1_isr(void);
    static void acmp2_isr(void);
    static void acmp3_isr(void);

public:
    ESC(){}; // constructor
};

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
void ESC::incrementCommutation()
{
    ELECTRICAL_STEP_CTR++;
    MECHANICAL_STEP_CTR++;
    if (MECHANICAL_STEP_CTR % MECHANICAL_CYCLE_MOD == 0)
    {
        computeSpeed();
    }
    ELECTRICAL_STEP_CTR %= ELECTRICAL_CYCLE_MOD;
    MECHANICAL_STEP_CTR %= MECHANICAL_CYCLE_MOD;
    if (STARTUP_MODE == true)
    { // fast blink mode every electrical cycle
        if (ELECTRICAL_STEP_CTR == 0)
        {
            if (LED_EN_ON == true)
            {
                LED_EN_ON = false;
                digitalWriteFast(LED_PIN, LOW);
            }
            else if (LED_EN_ON == false)
            {
                LED_EN_ON = true;
                digitalWriteFast(LED_PIN, HIGH);
            }
        }
    }
    else
    { // slow blink mode every mechanical cycle
        if (MECHANICAL_STEP_CTR == 0)
        {
            if (LED_EN_ON == true)
            {
                LED_EN_ON = false;
                digitalWriteFast(LED_PIN, LOW);
            }
            else if (LED_EN_ON == false)
            {
                LED_EN_ON = true;
                digitalWriteFast(LED_PIN, HIGH);
            }
        }
    }
    SPEED_STEP_CTR++;
};

void ESC::enforceCommutation(){};
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
/*

    // ACMP registers

    // 0 - [DMA] - 0 - [RISING INTERUPT] - [FALLING INTERUPT] - [RISE FLAG] - [FALL FLAG] - [ANALOG COMP OUTPUT]
    int ACMP_STATUS_CONTROL_REGISTERS[3] = {
        CMP1_SCR,
        CMP2_SCR,
        CMP3_SCR};

    // 0 - [FILTER_CNT] - 0 - 0 -[HYSTCTR]
    int ACMP_CONTROL_REGISTER_0[3] = {
        CMP1_CR0,
        CMP2_CR0,
        CMP3_CR0};

    // [SE] - [WE] - [x] - [PMODE] - [INVERT] - [COS] - [OPE] - [EN]
    int ACMP_CONTROL_REGISTER_1[3] = {
        CMP1_CR1,
        CMP2_CR1,
        CMP3_CR1};

    // disable 0x00
    int ACMP_DAC_CONTROL_REGISTER[3] = {
        CMP1_DACCR,
        CMP2_DACCR,
        CMP3_DACCR};

    Specifies the sampling period, in bus clock cycles, of the comparator output filter, when CR1[SE]=0.
     Setting FILT_PER to 0x0 disables the filter.
    int ACMP_FILTER_PERIOD_REGISTER[3] = {
        CMP1_FPR,
        CMP2_FPR,
        CMP3_FPR};



    [0] - 0 - [PSEL] - [MSEL]
    0 - 0 - [x - x - x] - [x - x - x]

    MSEL / PSEL
      000 IN0
      001 IN1
      010 IN2
      011 IN3
      100 IN4
      101 IN5
      110 IN6
      111 IN7

   int ACMP_MUX[3] = {
       CMP1_MUXCR,
       CMP2_MUXCR,
       CMP3_MUXCR
   };
*/