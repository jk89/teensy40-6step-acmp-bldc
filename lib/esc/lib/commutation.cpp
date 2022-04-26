#include "../esc.hpp"
#include <Arduino.h>
#include "imxrt.h"

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

void ESC::handleZeroCrossing()
{
  if (MODULATED_THRUST_MODE == true)
  {
    calculateNextCommutationStepDuty();
  }

  // commute motor step
  incrementCommutation();
  enforceCommutation();

  // we have some time before the next change of state...
  readHostControlProfile();

  if (DEBUG_MODE == true)
  {

    cli(); // halt interrupts

    // we are past the incrementor so we have advanced one step beyond what we are printing.
    auto stepnow = ELECTRICAL_STEP_CTR - 1;
    if (stepnow == -1)
    {
      stepnow = ELECTRICAL_CYCLE_MOD - 1; // ELECTRICAL_CYCLE_MOD
    }

    switch (stepnow)
    {
    case 0:
      if (clockwise == true)
      {
        // AH_BL() C RISING
        Serial.print(LOW);
        Serial.print("\t");
        Serial.print(LOW);
        Serial.print("\t");
        Serial.print(300);
        Serial.print("\t");
      }
      else
      {
        // CH_BL() A RISING
        Serial.print(300);
        Serial.print("\t");
        Serial.print(LOW);
        Serial.print("\t");
        Serial.print(LOW);
        Serial.print("\t");
      }
      break;
    case 1:
      // AH_CL()  B FALLING
      if (clockwise == true)
      {
        Serial.print(LOW);
        Serial.print("\t");
        Serial.print(-300);
        Serial.print("\t");
        Serial.print(LOW);
        Serial.print("\t");
      }
      else
      {
        // CH_AL() B FALLING
        Serial.print(LOW);
        Serial.print("\t");
        Serial.print(-300);
        Serial.print("\t");
        Serial.print(LOW);
        Serial.print("\t");
      }
      break;
    case 2:
      if (clockwise == true)
      {
        // BH_CL()  A RISING
        Serial.print(300);
        Serial.print("\t");
        Serial.print(LOW);
        Serial.print("\t");
        Serial.print(LOW);
        Serial.print("\t");
      }
      else
      {
        // BH_AL() C RISING
        Serial.print(LOW);
        Serial.print("\t");
        Serial.print(LOW);
        Serial.print("\t");
        Serial.print(300);
        Serial.print("\t");
      }
      break;
    case 3:
      if (clockwise == true)
      {
        // BH_AL() C FALLING
        Serial.print(LOW);
        Serial.print("\t");
        Serial.print(LOW);
        Serial.print("\t");
        Serial.print(-300);
        Serial.print("\t");
      }
      else
      {
        // BH_CL() A FALLING
        Serial.print(-300);
        Serial.print("\t");
        Serial.print(LOW);
        Serial.print("\t");
        Serial.print(LOW);
        Serial.print("\t");
      }
      break;

    case 4:
      if (clockwise == true)
      {
        // CH_AL() B RISING
        Serial.print(LOW);
        Serial.print("\t");
        Serial.print(300);
        Serial.print("\t");
        Serial.print(LOW);
        Serial.print("\t");
      }
      else
      {
        // AH_CL() B RISING
        Serial.print(LOW);
        Serial.print("\t");
        Serial.print(300);
        Serial.print("\t");
        Serial.print(LOW);
        Serial.print("\t");
      }
      break;
    case 5:
      if (clockwise == true)
      {
        // CH_BL() A FALLING
        Serial.print(-300);
        Serial.print("\t");
        Serial.print(LOW);
        Serial.print("\t");
        Serial.print(LOW);
        Serial.print("\t");
      }
      else
      {
        // AH_BL() C FALLING
        Serial.print(LOW);
        Serial.print("\t");
        Serial.print(LOW);
        Serial.print("\t");
        Serial.print(-300);
        Serial.print("\t");
      }
      break;
    }
    Serial.print(MECHANICAL_STEP_CTR);
    Serial.print("\t");
    Serial.print(ROLL);
    Serial.print("\t");
    Serial.print(PITCH);
    Serial.print("\t");
    Serial.print(THRUST);
    Serial.print("\t");
    Serial.print(RPM);
    Serial.print("\n");
    sei();
  }

  // read the host profile
  /*if (Serial.available()) {
    readHostControlProfile();
    MODULATED_THRUST_MODE = true;
    }*/
  // this would be better done in a timer callback say 200hz
}