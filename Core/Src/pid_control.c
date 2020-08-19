/*
 * pid_control.c
 *
 *  Created on: Jul 8, 2020
 *      Author: Florian Brenot
 */

#include "pid_control.h"

float PID_Compute(const PID_Terms *Terms, PID_History *History, float TimeDelta, float Setpoint, float ProcessVariable)
{
  const float error = Setpoint - ProcessVariable;

  const float proportional = Terms->ProportionalGain * error;

  History->Integral += error * TimeDelta;
  const float integral = Terms->IntegralGain * History->Integral;

  const float derivative = Terms->DerivativeGain * ((error - History->PreviousError) / TimeDelta);
  History->PreviousError = error;

  return proportional + integral + derivative;
}
