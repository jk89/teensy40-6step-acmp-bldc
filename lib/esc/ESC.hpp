#define RISING 1
#define FALLING 0
#define PHASE_A 0
#define PHASE_B 1
#define PHASE_C 2
#define PI 3.1415926535897932384626433832795
const float FULL_CYCLE = 2 * PI;
const int SIZE_OF_FLOAT = sizeof(float);


struct PinState {
    char IN_HIGH;
    char SD_PWM;
    char BEMF_Phase;
    bool BEMF_Direction;
};

/*
class ESC {
    private:
    public:
        bool clockwise = true;
        static PinState clockwisePinState[6];
        static PinState antiClockwisePinState[6];
        
        
};

*/
