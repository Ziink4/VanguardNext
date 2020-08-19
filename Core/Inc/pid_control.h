/*
 * pid_control.h
 *
 *  Created on: Jul 8, 2020
 *      Author: Florian Brenot
 */

#ifndef __PID_CONTROL_H
#define __PID_CONTROL_H

typedef struct __PID_Terms
{
  const float ProportionalGain;
  const float IntegralGain;
  const float DerivativeGain;
} PID_Terms;

typedef struct __PID_History
{
  float PreviousError;
  float Integral;
} PID_History;

float PID_Compute(const PID_Terms *Terms, PID_History *History, float TimeDelta, float Setpoint, float ProcessVariable);

#endif /* __PID_CONTROL_H */
