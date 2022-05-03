#include <Arduino.h>
#include "imxrt.h"

#define PI 3.1415926535897932384626433832795

#define A_IN 2  // A High/low switch
#define B_IN 9  // B High/low switch
#define C_IN 8  // C High/low switch
#define A_SD 1  // PWM A
#define B_SD 0  // PWM B
#define C_SD 7 // PWM C
#define LED_EN 13

// acmp1_isr
// IRQ_ACMP1

/*
  ACMP
  21: PhaseA
  22: PhaseB
  23: PhaseC
  18: Virtual Neutral
*/

#define PWM_FREQUENCY 32000 // 60k 64000 // 2300// should be 20khz+ 1950 seems to atleast do something.

#define MIN_DUTY 60
#define MAX_DUTY 240

#define MOTOR_POLES 14
#define ELECTRICAL_CYCLES_PER_MECHANICAL_CYCLE (MOTOR_POLES / 2) // per physical revolution
#define ELECTRICAL_CYCLE_MOD 6

const float FULL_CYCLE = 2 * PI;
byte ELECTRICAL_STEP_CTR = 0;
int MECHANICAL_STEP_CTR = 0;
int SPEED_STEP_CTR = 0;
float RPM = 0;
elapsedMicros TIME_SINCE_LAST_MEASUREMENT;

const byte MECHANICAL_CYCLE_MOD = ELECTRICAL_CYCLES_PER_MECHANICAL_CYCLE * ELECTRICAL_CYCLE_MOD;
const float COMMUTATION_STEP_ANGLE = FULL_CYCLE / MECHANICAL_CYCLE_MOD;

bool anticlockwise = false;

float computeSpeed() {
  float elapsedMicrosPerStep = (float) TIME_SINCE_LAST_MEASUREMENT / (float) SPEED_STEP_CTR;

  float microsPerRevolution = elapsedMicrosPerStep * (float) MECHANICAL_CYCLE_MOD;

  // 1 microsseconds = 1 / 6e+7 minutes
  // float minutesPerRevolution = microsPerRevolution / 60000000; // 6e+7
  float rpm = 60000000 / microsPerRevolution;
  RPM = rpm;

  TIME_SINCE_LAST_MEASUREMENT = 0;
  SPEED_STEP_CTR = 0;

  return rpm;
}

bool LED_EN_ON = false;
int LED_STEP_CTR = 0;
int LED_CYCLE_MOD = 66;
boolean STARTUP_MODE = true;

void incrementCommutationState()
{
  ELECTRICAL_STEP_CTR++;
  MECHANICAL_STEP_CTR++;
  if (MECHANICAL_STEP_CTR % MECHANICAL_CYCLE_MOD == 0) {
    computeSpeed();
  }
  ELECTRICAL_STEP_CTR %= ELECTRICAL_CYCLE_MOD;
  MECHANICAL_STEP_CTR %= MECHANICAL_CYCLE_MOD;
  if (STARTUP_MODE == true) { // fast blink mode every electrical cycle
    if (ELECTRICAL_STEP_CTR == 0) {
      if (LED_EN_ON == true) {
        LED_EN_ON = false;
        digitalWriteFast(LED_EN, LOW);
      }
      else if (LED_EN_ON == false) {
        LED_EN_ON = true;
        digitalWriteFast(LED_EN, HIGH);
      }
    }
  }
  else { // slow blink mode every mechanical cycle
    if (MECHANICAL_STEP_CTR == 0) {
      if (LED_EN_ON == true) {
        LED_EN_ON = false;
        digitalWriteFast(LED_EN, LOW);
      }
      else if (LED_EN_ON == false) {
        LED_EN_ON = true;
        digitalWriteFast(LED_EN, HIGH);
      }
    }
  }

  SPEED_STEP_CTR++;
}

float angularDisplacement()
{
  if (anticlockwise == true)
  {
    // subtract our progress from a full rotation
    return FULL_CYCLE - (MECHANICAL_STEP_CTR * COMMUTATION_STEP_ANGLE);
  }
  else
  {
    // positive progression of ctr adds to our angle
    return MECHANICAL_STEP_CTR * COMMUTATION_STEP_ANGLE;
  }
}

float SIN_PRECOMP[MECHANICAL_CYCLE_MOD];
float COS_PRECOMP[MECHANICAL_CYCLE_MOD];
void buildTrigTables()
{
  for (byte z = 0; z < MECHANICAL_CYCLE_MOD; z++)
  {
    const float angle = COMMUTATION_STEP_ANGLE * (float)z;
    SIN_PRECOMP[z] = sin(angle);
    COS_PRECOMP[z] = cos(angle);
  }
}

float PITCH = 0.0;
float ROLL = 0.0;
byte DUTY_TARGET = MIN_DUTY;
byte NEXT_DUTY = MIN_DUTY;


boolean MODULATED_THRUST_MODE = false;

// COMMUNICATION --------------------------------------------------------------------------------------------------

const int SIZE_OF_FLOAT = sizeof(float); // 4
struct DATA_W
{
  float roll;
  float pitch;
} AXES_STRUCT;

// we read 9 bytes in total
const int SIZE_OF_PROFILE = (SIZE_OF_FLOAT * 2) + 1;

char HOST_PROFILE_BUFFER[SIZE_OF_PROFILE];
byte HOST_PROFILE_BUFFER_CTR = 0;

bool readHostControlProfile()
{
  bool proccessedAFullProfile = false;
  cli();
  while (Serial.available()) {
    HOST_PROFILE_BUFFER[HOST_PROFILE_BUFFER_CTR] = Serial.read();
    HOST_PROFILE_BUFFER_CTR++;

    if (HOST_PROFILE_BUFFER_CTR % SIZE_OF_PROFILE == 0) {
      // unpack profile
      char rollBuffer[SIZE_OF_FLOAT];
      char pitchBuffer[SIZE_OF_FLOAT];
      float roll;
      float pitch;
      char thrust;

      thrust = HOST_PROFILE_BUFFER[0];

      *((unsigned char *)&roll + 0) = HOST_PROFILE_BUFFER[1];
      *((unsigned char *)&roll + 1) = HOST_PROFILE_BUFFER[2];
      *((unsigned char *)&roll + 2) = HOST_PROFILE_BUFFER[3];
      *((unsigned char *)&roll + 3) = HOST_PROFILE_BUFFER[4];

      *((unsigned char *)&pitch + 0) = HOST_PROFILE_BUFFER[5];
      *((unsigned char *)&pitch + 1) = HOST_PROFILE_BUFFER[6];
      *((unsigned char *)&pitch + 2) = HOST_PROFILE_BUFFER[7];
      *((unsigned char *)&pitch + 3) = HOST_PROFILE_BUFFER[8];

      if (thrust == 0) {
        // this is the reset signal
        setup();
      }
      else {
        PITCH = pitch;
        ROLL = roll;
        DUTY_TARGET = max(thrust, MIN_DUTY);
        NEXT_DUTY = DUTY_TARGET;
      }
      proccessedAFullProfile = true;

    }
    HOST_PROFILE_BUFFER_CTR %= SIZE_OF_PROFILE;
  }
  sei();
  return proccessedAFullProfile;
}

// END COMMUNICATION  ---------------------------------------------------------------------------------------------

// ACMP -----------------------------------------------------------------------------------------------------------


void initACMP() {

  //make sure the cycle counter register is active
  ARM_DWT_CTRL |= ARM_DWT_CTRL_CYCCNTENA; 

  CCM_CCGR3 |= CCM_CCGR3_ACMP1(CCM_CCGR_ON) | CCM_CCGR3_ACMP2(CCM_CCGR_ON) | CCM_CCGR3_ACMP3(CCM_CCGR_ON);

  attachInterruptVector(IRQ_ACMP1, acmp1_isr );
  attachInterruptVector(IRQ_ACMP2, acmp2_isr );
  attachInterruptVector(IRQ_ACMP3, acmp3_isr );

  // 0 - [x - x - x] - 0 - 0 - [x - x]
  // 0 - [FILTER_CNT] - 0 - 0 -[HYSTCTR]
  CMP1_CR0 = 0B00000000; // no filtering 5mV hysteresis
  CMP2_CR0 = 0B00000000; // no filtering 5mV hysteresis
  CMP3_CR0 = 0B00000000; // no filtering 5mV hysteresis // 0B01000000

  // 0B01100000

  // DACCR - disable
  CMP1_DACCR = 0x00;
  CMP2_DACCR = 0x00;
  CMP3_DACCR = 0x00;

  // CMPx_SCR
  // 0 - [DMA] - 0 - [RISING INTERUPT] - [FALLING INTERUPT] - [RISE FLAG] - [FALL FLAG] - [ANALOG COMP OUTPUT]
  /*CMP1_SCR = 0B00000000; // set to known state
    CMP2_SCR = 0B00000000;
    CMP3_SCR = 0B00000000;*/

  // (CMPx_FPR)
  // Specifies the sampling period, in bus clock cycles, of the comparator output filter, when CR1[SE]=0.
  // Setting FILT_PER to 0x0 disables the filter.
  CMP1_FPR = 0;
  CMP2_FPR = 0;
  CMP3_FPR = 0; // 2

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
  CMP1_MUXCR = 0B00110000; //set + to pin 21 (IN6) - to pin 18 (IN0) [phaseA - red]

  // pin 22 is acmp2_in5 101
  CMP2_MUXCR = 0B00101000; //set + to pin 22 (IN5) - to pin 18 (IN0) [phaseB - yellow]

  // pin 23 is acmp3_in5 101
  CMP3_MUXCR = 0B00101000; //set + to pin 23 (IN5) - to pin 18 (IN0)   [phaseC - black]


  // CMPx_CR1
  // 65.3.2 CMP Control Register 1 (CMPx_CR1)
  // x - x - x - x - x - x - x - x
  // [SE] - [WE] - [x] - [PMODE] - [INVERT] - [COS] - [OPE] - [EN]
  CMP1_CR1 = 0B01010001; //set high speed and power on, sample mode enabled
  CMP2_CR1 = 0B01010001;
  CMP3_CR1 = 0B01010001;
}

void BEMF_A_RISING() {
  // enable a rising
  CMP1_SCR |= 0B00010000;
  // CMP0_CR1 |= 0x40; //set windowing on
}
void BEMF_A_FALLING() {
  // enabling a falling
  CMP1_SCR |= 0B00001000;
  // CMP0_CR1 |= 0x40; //set windowing on
}
void BEMF_B_RISING() {
  // enable b rising
  CMP2_SCR |= 0B00010000;
  // CMP1_CR1 |= 0x40; //set windowing on
}
void BEMF_B_FALLING() {
  // enable b falling
  CMP2_SCR |= 0B00001000;
  // CMP1_CR1 |= 0x40; //set windowing on
}
void BEMF_C_RISING() {
  // enable c rising
  CMP3_SCR |= 0B00010000;
  // CMP2_CR1 |= 0x40; //set windowing on
}
void BEMF_C_FALLING() {
  // enable c falling
  CMP3_SCR |= 0B00001000;
  // CMP2_CR1 |= 0x40; //set windowing on
}

// 0 - [DMA] - 0 - [RISING INTERUPT] - [FALLING INTERUPT] - [RISE FLAG] - [FALL FLAG] - [ANALOG COMP OUTPUT]
auto fallingMask = 0B00000000;   // falling flag and interrupt
auto risingMask =  0B00000001; // rising flag and interrupt
auto coutHigh = 0x01;


/*
 auto fallingMask = 0B00000000;   // falling flag and interrupt
auto risingMask =  0B00000001; // rising flag and interrupt
this was good with debounce 150
 */

/*
 auto fallingMask = 0B00000010;   // falling flag and interrupt
auto risingMask =  0B00000101; // rising flag and interrupt
 */
auto debounceDistance = 323; // 330; // 165 //170 //150; // ;125 // 115

void enableACMPInterrupts() {
  NVIC_ENABLE_IRQ(IRQ_ACMP1);
  NVIC_ENABLE_IRQ(IRQ_ACMP2);
  NVIC_ENABLE_IRQ(IRQ_ACMP3);
}

void disableACMPInterrupts() {
  NVIC_DISABLE_IRQ(IRQ_ACMP1);
  NVIC_DISABLE_IRQ(IRQ_ACMP2);
  NVIC_DISABLE_IRQ(IRQ_ACMP3);
}

/*
  cmpflags = CMP1_SCR;

  { // clear interrupt status flags:
    uint8_t scr = (CMP1_SCR & ~(CMP_SCR_CFR_MASK | CMP_SCR_CFF_MASK));
    CMP1_SCR = scr | CMP_SCR_CFR_MASK | CMP_SCR_CFF_MASK;
  }

  if (cmpflags & CMP_SCR_CFR_MASK) {
    // See below! This line will be modified:
    digitalWrite(15, HIGH);
  }

  if (cmpflags & CMP_SCR_CFF_MASK) {
    digitalWrite(15, LOW);
  }
*/

/*
  handleZeroCrossing();
*/

auto delayTime = 0;

void acmp1_isr(void) {
  /*cli();
    Serial.println("acmp1_isr");
    sei();*/
  STARTUP_MODE = false;
  // debounce
  for (int i = 0; i < debounceDistance; i++) {
    if (ELECTRICAL_STEP_CTR & 1)  {
      // odd means falling
      if (!(CMP1_SCR & coutHigh)) {
        i -= 1;
      }
      /*if ((CMP1_SCR & fallingMask) != fallingMask) {
        i -= 1;
      }*/
    }
    else {
      //rising
      if (CMP1_SCR & coutHigh) {
        i -= 1;
      }
      /*if ((CMP1_SCR & risingMask) != risingMask) {
        i -= 1;
      }*/
    }
  }
  // Serial.print(0); Serial.print("\t");
  CMP1_SCR &= 0x00; // turn off A falling or rising
  asm volatile("dsb");
  // CMP0_CR1 &= 0xBF; // turn off windowing
  delayMicroseconds(delayTime);
  handleZeroCrossing();
}
void acmp2_isr(void) {
  /*cli();
    Serial.println("acmp2_isr");
    sei();*/
  STARTUP_MODE = false;
  // debounce
  for (int i = 0; i < debounceDistance; i++) {
    if (ELECTRICAL_STEP_CTR & 1)  {
      // odd means falling
      if (!(CMP2_SCR & coutHigh)) {
        i -= 1;
      }
      /*if ((CMP2_SCR & fallingMask) != fallingMask) {
        i -= 1;
      }*/
    }
    else {
      //rising
      if (CMP2_SCR & coutHigh) {
        i -= 1;
      }
      /*if ((CMP2_SCR & risingMask) != risingMask) {
        i -= 1;
      }*/
    }
    // clear flag
  }
  // Serial.print(1); Serial.print("\t");
  CMP2_SCR &= 0x00;// turn off B falling or rising
  asm volatile("dsb");
  // CMP1_CR1 &= 0xBF; // turn off windowing
  delayMicroseconds(delayTime);
  handleZeroCrossing();
}
void acmp3_isr(void) {
  /*cli();
    Serial.println("acmp3_isr");
    sei();*/
  STARTUP_MODE = false;
  // debounce
  for (int i = 0; i < debounceDistance; i++) {
    if (ELECTRICAL_STEP_CTR & 1)  {
      // odd means falling
      if (!(CMP3_SCR & coutHigh)) {
        i -= 1;
      }
      /*if ((CMP3_SCR & fallingMask) != fallingMask) {
        i -= 1;
      }*/
    }
    else {
      //rising
      if (CMP3_SCR & coutHigh) {
        i -= 1;
      }
      /*if ((CMP3_SCR & risingMask) != risingMask) {
        i -= 1;
      }*/
    }
  }
  // Serial.print(2); Serial.print("\t");
  // turn off C falling or rising
  CMP3_SCR &= 0x00;
  asm volatile("dsb");
  // CMP2_CR1 &= 0xBF; // turn off windowing
  delayMicroseconds(delayTime);
  handleZeroCrossing();
}

// END ACMP -----------------------------------------------------------------------------------------------------------

// XBAR  ----------------------------------------------------------------------------------------------------------

void xbarConnect(unsigned int input, unsigned int output)
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
}

void xbarInit()
{
  CCM_CCGR2 |= CCM_CCGR2_XBAR1(CCM_CCGR_ON); //turn clock on for xbara1
  // connect xbar pwm trigger output for the three pwm signals and pipe to adc etc trigger input

  // xbarConnect(XBARA1_IN_FLEXPWM1_PWM2_OUT_TRIG0, XBARA1_OUT_ADC_ETC_TRIG01); // pwm module 1.1 pin 0 phase B PWMING
  // xbarConnect(XBARA1_IN_FLEXPWM1_PWM4_OUT_TRIG0, XBARA1_OUT_ADC_ETC_TRIG02); // pwm module 1.3 pin 7 phase C PWMING

  // xbarConnect(XBARA1_IN_FLEXPWM4_PWM1_OUT_TRIG0, XBARA1_OUT_ADC_ETC_TRIG02); // pwm module 4.0 pin 22 phase C PWMING

  // 1.2 was added

  /*
     pwmInit_1_0();
    pwmInit_1_1();
    pwmInit_1_2(); // just used for timing no pin out
    pwmInit_1_3();
  */

  // cmp
  /*xbarConnect(XBARA1_IN_FLEXPWM1_PWM1_OUT_TRIG0, XBARA1_OUT_ACMP1_SAMPLE);
  xbarConnect(XBARA1_IN_FLEXPWM1_PWM2_OUT_TRIG0, XBARA1_OUT_ACMP2_SAMPLE);
  xbarConnect(XBARA1_IN_FLEXPWM1_PWM3_OUT_TRIG0, XBARA1_OUT_ADC_ETC_TRIG00);
  xbarConnect(XBARA1_IN_FLEXPWM1_PWM4_OUT_TRIG0, XBARA1_OUT_ACMP3_SAMPLE);*/

  xbarConnect(XBARA1_IN_FLEXPWM1_PWM1_OUT_TRIG0, XBARA1_OUT_ACMP1_SAMPLE);
  xbarConnect(XBARA1_IN_FLEXPWM1_PWM1_OUT_TRIG0, XBARA1_OUT_ACMP2_SAMPLE);
  xbarConnect(XBARA1_IN_FLEXPWM1_PWM1_OUT_TRIG0, XBARA1_OUT_ACMP3_SAMPLE);
  //
}

// END XBAR  ------------------------------------------------------------------------------------------------------

// MOTION    ------------------------------------------------------------------------------------------------------


void AH_BL()
{

  digitalWriteFast(C_IN, LOW);
  digitalWriteFast(B_IN, LOW);
  // turn on A_IN
  digitalWriteFast(A_IN, HIGH);

  digitalWriteFast(A_SD, LOW);
  digitalWriteFast(C_SD, LOW);
  // turn on / keep on PWM for B
  analogWrite(B_SD, NEXT_DUTY);

} // A_IN HIGH B_SB PWM

void AH_CL()
{
  digitalWriteFast(B_IN, LOW);
  digitalWriteFast(C_IN, LOW);
  // keep A_IN on
  digitalWriteFast(A_IN, HIGH);


  digitalWriteFast(A_SD, LOW);
  digitalWriteFast(B_SD, LOW);
  // turn on PWM for C
  analogWrite(C_SD, NEXT_DUTY);
} // A_IN high C_SD PWM

void BH_CL()
{
  digitalWriteFast(C_IN, LOW);
  // turn B_IN on
  digitalWriteFast(A_IN, LOW);
  digitalWriteFast(B_IN, HIGH);

  digitalWriteFast(A_SD, LOW);
  digitalWriteFast(B_SD, LOW);
  // keep PWM on for C
  analogWrite(C_SD, NEXT_DUTY);

} // B_IN high C_SD PWM

void BH_AL()
{
  digitalWriteFast(A_IN, LOW);
  digitalWriteFast(C_IN, LOW);
  // keep B_IN on
  digitalWriteFast(B_IN, HIGH);

  digitalWriteFast(B_SD, LOW);
  digitalWriteFast(C_SD, LOW);
  // turn pwn for A
  analogWrite(A_SD, NEXT_DUTY);

} // B_IN high A_SD PWM

void CH_AL()
{
  digitalWriteFast(A_IN, LOW);
  // turn C_IN on
  digitalWriteFast(B_IN, LOW);
  digitalWriteFast(C_IN, HIGH);

  digitalWriteFast(B_SD, LOW);
  digitalWriteFast(C_SD, LOW);
  // keep PWM on for A
  analogWrite(A_SD, NEXT_DUTY);

} // C_IN high A_SD PWM

void CH_BL()
{
  digitalWriteFast(A_IN, LOW);
  digitalWriteFast(B_IN, LOW);
  // keep c_in high
  digitalWriteFast(C_IN, HIGH);

  digitalWriteFast(C_SD, LOW);
  // turn pwm on for B
  digitalWriteFast(A_SD, LOW);
  analogWrite(B_SD, NEXT_DUTY);

  // C_IN HIGH B_SD PWM
}

void enforceCommutationState()
{
  switch (ELECTRICAL_STEP_CTR)
  {
    case 0:
      if (anticlockwise == false)
      {
        AH_BL(); // C RISING
        BEMF_C_RISING();
      }
      else
      {
        CH_BL(); // A RISING
        BEMF_A_RISING();
      }
      break;
    case 1:
      if (anticlockwise == false)
      {
        AH_CL(); // B FALLING
        BEMF_B_FALLING();
      }
      else
      {
        CH_AL(); // B FALLING
        BEMF_B_FALLING();
      }
      break;
    case 2:
      if (anticlockwise == false)
      {
        BH_CL(); // A RISING
        BEMF_A_RISING();
      }
      else
      {
        BH_AL(); // C RISING
        BEMF_C_RISING();
      }
      break;
    case 3:
      if (anticlockwise == false)
      {
        BH_AL(); // C FALLING
        BEMF_C_FALLING();
      }
      else
      {
        BH_CL(); // A FALLING
        BEMF_A_FALLING();
      }
      break;
    case 4:
      if (anticlockwise == false)
      {
        CH_AL(); // B RISING
        BEMF_B_RISING();
      }
      else
      {
        AH_CL(); // B RISING
        BEMF_B_RISING();
      }
      break;
    case 5:
      if (anticlockwise == false)
      {
        CH_BL(); // A FALLING
        BEMF_A_FALLING();
      }
      else
      {
        AH_BL(); // C FALLING
        BEMF_C_FALLING();
      }
      break;
  }
}

bool LAST_LOOP_WAS_STARTUP = false;
void startup()
{
  if (STARTUP_MODE == false)
  {
    return;
  }
  // do a linear chirp to force initial motor commutation as we lack a bemf trigger
  int i, x = 0;
  while (x < 1)
  {
    i = 5000;
    while (i > 20)
    {
      if (STARTUP_MODE == false)
      { // allow for early bemf detection exit
        return;
      }
      delayMicroseconds(i);
      incrementCommutationState();
      enforceCommutationState();
      i = i - 20;
    }
    x++;
    /*if (DEBUG_MODE == true)
      { // FIXME probably disable this if it messes with the timings
      logInfo();
      }*/
  }
}

void calculateNextCommutationStepDuty()
{
  // calculate next steps pwm according to the Host Control Profile

  byte dutyAvailable = min(MAX_DUTY - DUTY_TARGET, DUTY_TARGET);

  float sinComponent = SIN_PRECOMP[MECHANICAL_STEP_CTR];
  float cosComponent = COS_PRECOMP[MECHANICAL_STEP_CTR];
  // roll sine
  float sinComp = ROLL * dutyAvailable * sinComponent;
  // pitch cos
  float cosComp = PITCH * dutyAvailable * cosComponent;

  // thrust
  float thrust = round(sinComp + cosComp + DUTY_TARGET);

  NEXT_DUTY = thrust;
}

// END MOTION -----------------------------------------------------------------------------------------------------

// BEMF -----------------------------------------------------------------------------------------------------------



bool processZeroCrossing()
{
  // we need to parse a reference to the buffer in question
}

#define CHANGE_THRESHOLD 0

void handleZeroCrossing()
{
  if (MODULATED_THRUST_MODE == true)
  {
    calculateNextCommutationStepDuty();
  }

  // commute motor step
  incrementCommutationState();
  enforceCommutationState();

  // we have some time before the next change of state...
  readHostControlProfile();


  // read the host profile
  /*if (Serial.available()) {
    readHostControlProfile();
    MODULATED_THRUST_MODE = true;
    }*/
  // this would be better done in a timer callback say 200hz
}

// END BEMF -----------------------------------------------------------------------------------------------------------


// PWM CTRL -----------------------------------------------------------------------------------------------------------

void pwmInit_1_0()
{ // this works ok pin 1
  analogWriteFrequency(A_SD, PWM_FREQUENCY);
  FLEXPWM1_SM0TCTRL = FLEXPWM_SMTCTRL_OUT_TRIG_EN(1 << 4);
}

// sync clocks of flexpwm1_SM0 and flexpwm1_SM1
/*
  CLK_SEL
  Clock Source Select
  These read/write bits determine the source of the clock signal for this submodule.
  00b - The IPBus clock is used as the clock for the local prescaler and counter.
  01b - EXT_CLK is used as the clock for the local prescaler and counter.
  10b - Submodule 0’s clock (AUX_CLK) is used as the source clock for the local prescaler and
  counter. This setting should not be used in submodule 0 as it will force the clock to logic 0.
  11b - reserved
*/
void pwmInit_1_1()
{ // this works ok pin 0
  // analogWriteFrequency(B_SD, PWM_FREQUENCY);
  FLEXPWM1_MCTRL |= FLEXPWM_MCTRL_CLDOK(0x0F);                            //  Clear Load Okay LDOK(SM) -> no reload of PWM settings
  FLEXPWM1_SM1CTRL2 = FLEXPWM_SMCTRL2_INDEP | FLEXPWM_SMCTRL2_CLK_SEL(2) | FLEXPWM_SMCTRL2_INIT_SEL(0); //A & B independant | sm0 chosen as clock
  FLEXPWM1_MCTRL |= FLEXPWM_MCTRL_LDOK(0x0F);                             // Load Okay LDOK(SM) -> reload setting again
  FLEXPWM1_SM1TCTRL = FLEXPWM_SMTCTRL_OUT_TRIG_EN(1 << 4);
}

void pwmInit_1_3()
{ // this works ok pin 0
  // analogWriteFrequency(C_SD, PWM_FREQUENCY);
  FLEXPWM1_MCTRL |= FLEXPWM_MCTRL_CLDOK(0x0F);                            //  Clear Load Okay LDOK(SM) -> no reload of PWM settings
  FLEXPWM1_SM3CTRL2 = FLEXPWM_SMCTRL2_INDEP | FLEXPWM_SMCTRL2_CLK_SEL(2) | FLEXPWM_SMCTRL2_INIT_SEL(0); //A & B independant | sm0 chosen as clock
  FLEXPWM1_MCTRL |= FLEXPWM_MCTRL_LDOK(0x0F);                             // Load Okay LDOK(SM) -> reload setting again
  FLEXPWM1_SM3TCTRL = FLEXPWM_SMTCTRL_OUT_TRIG_EN(1 << 4);
  // | FLEXPWM_SMCTRL2_INIT_SEL(1)
}


void pwmInit_1_2()
{
  FLEXPWM1_MCTRL |= FLEXPWM_MCTRL_CLDOK(0x0F);                            //  Clear Load Okay LDOK(SM) -> no reload of PWM settings
  FLEXPWM1_SM2CTRL2 = FLEXPWM_SMCTRL2_INDEP | FLEXPWM_SMCTRL2_CLK_SEL(2) | FLEXPWM_SMCTRL2_INIT_SEL(0); //A & B independant | sm0 chosen as clock
  FLEXPWM1_MCTRL |= FLEXPWM_MCTRL_LDOK(0x0F);                             // Load Okay LDOK(SM) -> reload setting again
  FLEXPWM1_SM2TCTRL = FLEXPWM_SMTCTRL_OUT_TRIG_EN(1 << 4);
}

void pwmInit()
{
  analogWriteRes(8);
  pwmInit_1_0();
  pwmInit_1_1();
  pwmInit_1_2(); // just used for timing no pin out
  pwmInit_1_3();
}

// END PWM CTRL -----------------------------------------------------------------------------------------------------------

void setup()
{
  pinMode(A_IN, OUTPUT);
  pinMode(B_IN, OUTPUT);
  pinMode(C_IN, OUTPUT);
  pinMode(A_SD, OUTPUT);
  pinMode(B_SD, OUTPUT);
  pinMode(C_SD, OUTPUT);
  pinMode(LED_EN, OUTPUT);
  buildTrigTables();
  MODULATED_THRUST_MODE = false;
  DUTY_TARGET = MIN_DUTY;
  NEXT_DUTY = MIN_DUTY;
  xbarInit();
  pwmInit();
  initACMP();
  disableACMPInterrupts();
  // force off everything
  digitalWriteFast(A_IN, LOW);
  digitalWriteFast(B_IN, LOW);
  digitalWriteFast(C_IN, LOW);
  analogWrite(A_SD, LOW);
  analogWrite(B_SD, LOW);
  analogWrite(C_SD, LOW);
  STARTUP_MODE = true;
  LAST_LOOP_WAS_STARTUP = false;
}


float CACHE_PITCH = 0.0;
float CACHE_ROLL = 0.0;
byte CACHE_DUTY_TARGET = 0;

void loop()
{
  // delay until we a signal
  if (STARTUP_MODE == true) {
    digitalWriteFast(LED_EN, LOW);
    LED_EN_ON = false;
    while (!readHostControlProfile()) {
      delay(100);
    }
    // cache read profile
    CACHE_PITCH = PITCH;
    CACHE_ROLL = ROLL;
    CACHE_DUTY_TARGET = DUTY_TARGET;
    // override host settings, sorry host but unless u know what ur doing, everythings gonna explode.
    PITCH = 0.0;
    ROLL = 0.0;
    DUTY_TARGET = MIN_DUTY;
    NEXT_DUTY = MIN_DUTY;
    digitalWriteFast(LED_EN, HIGH);
    LED_EN_ON = true;
  }



  if (STARTUP_MODE == true) {

    startup();

    // Serial.println("got past startup");

    enableACMPInterrupts();

    // Serial.println("got past enableACMPInterrupts");
    STARTUP_MODE = false;
    LAST_LOOP_WAS_STARTUP = true;

  }







  if (LAST_LOOP_WAS_STARTUP == true) {
    // restore host profile if it is safe to do so
    if (CACHE_DUTY_TARGET < 30) {
      PITCH = CACHE_PITCH;
      ROLL = CACHE_ROLL;
      DUTY_TARGET = CACHE_DUTY_TARGET;
      NEXT_DUTY = DUTY_TARGET;
    }
    LAST_LOOP_WAS_STARTUP = false;
    digitalWriteFast(LED_EN, LOW);
    LED_EN_ON = false;
  }


  delayMicroseconds(200);


  readHostControlProfile();


}
