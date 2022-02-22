#include "esc.hpp"
#include <Arduino.h>
#include "imxrt.h"

#include "lib/acmp.cpp"
#include "lib/pwm.cpp"
#include "lib/xbar.cpp"
#include "lib/acmp.cpp"

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
    int DEBOUNCE_DISTANCE = 323; // 330; // 165 //170 //150; // ;125 // 115

    bool STARTUP_MODE = true;
    bool LED_EN_ON = false;

    bool MODULATED_THRUST_MODE;

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
    float* SIN_PRECOMP = new float[MECHANICAL_CYCLE_MOD];
    float* COS_PRECOMP = new float[MECHANICAL_CYCLE_MOD];

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
    void handleZeroCrossing();

    static void acmp1_isr(void);
    static void acmp2_isr(void);
    static void acmp3_isr(void);
    void debounce(volatile uint8_t &CMPSCRRegister);

public:
    ESC(){}; // constructor
};