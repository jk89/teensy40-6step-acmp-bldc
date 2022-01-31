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

    /* Specifies the sampling period, in bus clock cycles, of the comparator output filter, when CR1[SE]=0.
     Setting FILT_PER to 0x0 disables the filter. */
    int ACMP_FILTER_PERIOD_REGISTER[3] = {
        CMP1_FPR,
        CMP2_FPR,
        CMP3_FPR};


    /* 
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
    */
   int ACMP_MUX[3] = {
       CMP1_MUXCR,
       CMP2_MUXCR,
       CMP3_MUXCR
   };

    void incrementCommutation();
    void enforceCommutation();
    float computeSpeed();
    float angularDisplacement();
    bool readHostControlProfile();
    void setup() {}
    void startup() {}
    void controlLoop() {}
    void enableACMPInterrupts() {}
    void disableACMPInterrupts() {}
    void calculateNextCommutationStepDuty() {}
    void initPWM() {}
    void initACMP() {}
    void buildTrigTables() {}

public:
    ESC(){}; // constructor
};

void ESC::incrementCommutation() {}
void ESC::enforceCommutation() {}
void ESC::initPWM() {
    analogWriteRes(8);
    // pwm 1.0
    //analogWriteFrequency(A_SD, PWM_FREQUENCY);
    FLEXPWM1_SM0TCTRL = FLEXPWM_SMTCTRL_OUT_TRIG_EN(1 << 4);
}