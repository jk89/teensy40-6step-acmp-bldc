#define RISING 1
#define FALLING 0
#define PHASE_A 0
#define PHASE_B 1
#define PHASE_C 2
#define PI 3.1415926535897932384626433832795
const float FULL_CYCLE = 2 * PI;
const int SIZE_OF_FLOAT = sizeof(float);

struct PinState
{
    char IN_HIGH;
    char SD_PWM;
    char BEMF_Phase;
    bool BEMF_Direction;
};

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
    static char MOTOR_POLES;

    // system var
    static int PWM_FREQUENCY;
    static char MIN_DUTY;
    static char MAX_DUTY;
    static int DEBOUNCE_DISTANCE;

    bool STARTUP_MODE;
    bool LED_EN_ON;

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

    // dynamics
    float RPM;

    // Control
    static bool clockwise;
    float ROLL;
    float PITCH;
    char THRUST;
    char NEXT_DUTY;

    char SD_PINS[3];
    char IN_PINS[3];
    int LED_PIN;

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

    void debounce(volatile uint8_t &CMPSCRRegister);

    static void acmp1_isr(void);
    static void acmp2_isr(void);
    static void acmp3_isr(void);

public:
    static void init() {}
    static ESC get_instance() {}
    static bool initialised;
    //ESC(const ESC &);
    //ESC(ESC &&);
    ESC(int arg);
    // ESC &operator=(const ESC &);
    // ESC &operator=(ESC &&);
};
