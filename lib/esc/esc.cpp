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
    inline static constexpr PinState antiCLOCKWISEPinState[] = {
        {PHASE_C, PHASE_B, PHASE_A, RISING},
        {PHASE_C, PHASE_A, PHASE_B, FALLING},
        {PHASE_B, PHASE_A, PHASE_C, RISING},
        {PHASE_B, PHASE_C, PHASE_A, FALLING},
        {PHASE_A, PHASE_C, PHASE_B, RISING},
        {PHASE_A, PHASE_B, PHASE_C, FALLING}};

    inline static constexpr PinState CLOCKWISEPinState[] = {
        {PHASE_A, PHASE_B, PHASE_C, RISING},
        {PHASE_A, PHASE_C, PHASE_B, FALLING},
        {PHASE_B, PHASE_C, PHASE_A, RISING},
        {PHASE_B, PHASE_A, PHASE_C, FALLING},
        {PHASE_C, PHASE_A, PHASE_B, RISING},
        {PHASE_C, PHASE_B, PHASE_A, FALLING}};

    static char MOTOR_POLES; // 14

    // system var
    static int PWM_FREQUENCY;     // 32k
    static int MIN_DUTY;          // 60
    static int MAX_DUTY;          // 240
    static int DEBOUNCE_DISTANCE; // 323 // 330; // 165 //170 //150; // ;125 // 115

    bool STARTUP_MODE = true;
    bool LED_EN_ON = false;

    bool MODULATED_THRUST_MODE;

    // Constants
    const char ELECTRICAL_CYCLE_MOD = 6;

    // Computed mechanical
    char ELECTRICAL_CYCLES_PER_MECHANICAL_CYCLE;
    char MECHANICAL_CYCLE_MOD;
    float COMMUTATION_STEP_ANGLE;

    // Counters
    char ELECTRICAL_STEP_CTR;
    int MECHANICAL_STEP_CTR;
    int SPEED_STEP_CTR;

    // float SIN_PRECOMP[MECHANICAL_CYCLE_MOD];
    // float COS_PRECOMP[MECHANICAL_CYCLE_MOD];
    float *SIN_PRECOMP; // = new float[MECHANICAL_CYCLE_MOD];
    float *COS_PRECOMP; // = new float[MECHANICAL_CYCLE_MOD];

    // dynamics
    float RPM;

    // Control
    static bool CLOCKWISE;
    float ROLL;
    float PITCH;
    char THRUST;    // MIN_DUTY;
    char NEXT_DUTY; // MIN_DUTY;

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
    static void init(bool CLOCKWISE, int MOTOR_POLES, int PWM_FREQUENCY, int MIN_DUTY, int MAX_DUTY, int DEBOUNCE_DISTANCE);
    static ESC get_instance();
    static bool initialised;
    ESC();

    /* Disabling copy and move semantics because singletons cannot be copied and shouldnt be moved */
    // ESC(const ESC &) = delete;
    // ESC(ESC &&) = delete;
    ESC &operator=(const ESC &) = delete;
    // ESC &operator=(ESC &&) = delete;
};

bool ESC::initialised = false;

void ESC::init(bool CLOCKWISE, int MOTOR_POLES, int PWM_FREQUENCY, int MIN_DUTY, int MAX_DUTY, int DEBOUNCE_DISTANCE)
{
    // deal with args
    ESC::MOTOR_POLES = MOTOR_POLES;
    ESC::PWM_FREQUENCY = PWM_FREQUENCY;
    ESC::MIN_DUTY = MIN_DUTY;
    ESC::MAX_DUTY = MAX_DUTY;
    ESC::DEBOUNCE_DISTANCE = DEBOUNCE_DISTANCE;
    ESC::CLOCKWISE = CLOCKWISE;
    initialised = true;
};

ESC ESC::get_instance()
{
    if (initialised == false)
    {
        throw "Need to call ESC::init(...args) before retreving an instance";
    }
    // apply args to init
    static ESC instance;
    return instance;
};

ESC::ESC()
{
    ELECTRICAL_CYCLES_PER_MECHANICAL_CYCLE = (MOTOR_POLES / 2);
    MECHANICAL_CYCLE_MOD = ELECTRICAL_CYCLES_PER_MECHANICAL_CYCLE * ELECTRICAL_CYCLE_MOD;
    COMMUTATION_STEP_ANGLE = FULL_CYCLE / MECHANICAL_CYCLE_MOD;

    SIN_PRECOMP = new float[MECHANICAL_CYCLE_MOD];
    COS_PRECOMP = new float[MECHANICAL_CYCLE_MOD];

    ELECTRICAL_STEP_CTR = 0;
    MECHANICAL_STEP_CTR = 0;
    SPEED_STEP_CTR = 0;
    RPM = 0;

    CLOCKWISE = true;
    ROLL = 0;
    PITCH = 0;
    THRUST = MIN_DUTY;
    NEXT_DUTY = MIN_DUTY;
}
