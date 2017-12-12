/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2011-2012 Bitcraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 *
 * pid.c - implementation of the PID regulator
 */

#include "pid.h"
#include "num.h"
#include <float.h>
static float height = 0.4;
static float zero = 0;

static float z1_old  = 0;
static float roll1_old = 0;
static float pitch1_old = 0;
static float z_dot1_old = 0;
static float roll_dot1_old = 0;
static float pitch_dot1_old = 0;
static float yaw_dot1_old = 0;
static float lpf1 = 0.945;
static float lpf2 = 0.0549;




void pidInit(PidObject* pid, const float desired, const float kp,
             const float ki, const float kd, const float dt,
             const float samplingRate, const float cutoffFreq,
             bool enableDFilter)
{
  pid->error         = 0;
  pid->prevError     = 0;
  pid->integ         = 0;
  pid->deriv         = 0;
  pid->desired       = desired;
  pid->kp            = kp;
  pid->ki            = ki;
  pid->kd            = kd;
  pid->iLimit        = DEFAULT_PID_INTEGRATION_LIMIT;
  pid->outputLimit   = DEFAULT_PID_OUTPUT_LIMIT;
  pid->dt            = dt;
  pid->enableDFilter = enableDFilter;
  if (pid->enableDFilter)
  {
    lpf2pInit(&pid->dFilter, samplingRate, cutoffFreq);
  }
}

float pidUpdate(PidObject* pid, const float measured, const bool updateError)
{
    float output = 0.0f;

    if (updateError)
    {
        pid->error = pid->desired - measured;
    }

    pid->outP = pid->kp * pid->error;
    output += pid->outP;

    float deriv = (pid->error - pid->prevError) / pid->dt;
    if (pid->enableDFilter)
    {
      pid->deriv = lpf2pApply(&pid->dFilter, deriv);
    } else {
      pid->deriv = deriv;
    }
    if (isnan(pid->deriv)) {
      pid->deriv = 0;
    }
    pid->outD = pid->kd * pid->deriv;
    output += pid->outD;

    pid->integ += pid->error * pid->dt;

    // Constrain the integral (unless the iLimit is zero)
    if(pid->iLimit != 0)
    {
    	pid->integ = constrain(pid->integ, -pid->iLimit, pid->iLimit);
    }

    pid->outI = pid->ki * pid->integ;
    output += pid->outI;

    // Constrain the total PID output (unless the outputLimit is zero)
    if(pid->outputLimit != 0)
    {
      output = constrain(output, -pid->outputLimit, pid->outputLimit);
    }


    pid->prevError = pid->error;

    return output;
}

void pidSetIntegralLimit(PidObject* pid, const float limit) {
    pid->iLimit = limit;
}


void pidReset(PidObject* pid)
{
  pid->error     = 0;
  pid->prevError = 0;
  pid->integ     = 0;
  pid->deriv     = 0;
}

void pidSetError(PidObject* pid, const float error)
{
  pid->error = error;
}

void pidSetDesired(PidObject* pid, const float desired)
{
  pid->desired = desired;
}

float pidGetDesired(PidObject* pid)
{
  return pid->desired;
}

bool pidIsActive(PidObject* pid)
{
  bool isActive = true;

  if (pid->kp < 0.0001f && pid->ki < 0.0001f && pid->kd < 0.0001f)
  {
    isActive = false;
  }

  return isActive;
}

void pidSetKp(PidObject* pid, const float kp)
{
  pid->kp = kp;
}

void pidSetKi(PidObject* pid, const float ki)
{
  pid->ki = ki;
}

void pidSetKd(PidObject* pid, const float kd)
{
  pid->kd = kd;
}
void pidSetDt(PidObject* pid, const float dt) {
    pid->dt = dt;
}







float lqr_m1(state_t *state, sensorData_t *sensors, setpoint_t *setpoint)
{

  const float k11 = 447.2;
  const float k12 = -316.1;
  const float k13 = -316.4;
  const float k14 = 3812.6;
  const float k15 = -0;
  const float k16 = -90.6;
  const float k17 = -83.9;

  float z = state->position.z ;
  float roll = state->attitude.roll;
  float pitch = state->attitude.pitch;
//  float yaw = state->attitude.yaw;
  float z_dot = state->velocity.z;
  float roll_dot = sensors->gyro.x;
  float pitch_dot = sensors->gyro.y;
  float yaw_dot = sensors->gyro.z;
  // float Ct = 3.1582 * pow(10,-10);
  // float x1 = setpoint->position.z - z;
  // float x2 = setpoint->attitude.pitch - pitch;
  // float x3 = setpoint->attitude.roll - roll;
  // float x4 = setpoint->velocity.z - z_dot;
  // float x5 = setpoint->attitudeRate.yaw - yaw_dot;
  // float x6 = setpoint->attitudeRate.pitch - pitch_dot;
  // float x7 = setpoint->attitudeRate.roll - roll_dot;

  // low pass filter
  z = lpf1 * z + lpf2 * z1_old;
  roll = lpf1 * roll + lpf2 * roll1_old;
  pitch = lpf1 * pitch + lpf2 * pitch1_old;
  z_dot = lpf1 * z_dot + lpf2 * z_dot1_old;
  roll_dot = lpf1 * roll_dot + lpf2 * roll_dot1_old;
  pitch_dot = lpf1 * pitch_dot + lpf2 * pitch_dot1_old;
  yaw_dot = lpf1 * yaw_dot + lpf2 * yaw_dot1_old;

  z1_old = z;
  roll1_old = roll;
  pitch1_old = pitch;
  z_dot1_old = z_dot;
  roll_dot1_old = roll1_old;
  pitch_dot1_old = pitch_dot;
  yaw_dot1_old = yaw_dot;


  float x1 = height - z;
  float x2 = zero - pitch;
  float x3 = zero - roll;
  float x4 = zero - z_dot;
  float x5 = zero - yaw_dot;
  float x6 = zero - pitch_dot;
  float x7 = zero - roll_dot;
  
  float omega1 = k11 * x1 + k12 * x2 + k13 * x3 + k14 * x4 + k15 * x5 + k16 * x6 + k17 + x7;
  // convert to thrust;
  // return Ct * omega1 * omega1;
  return omega1;
}

float lqr_m2(state_t *state, sensorData_t *sensors, setpoint_t *setpoint)
{
  const float k21 = 13;
  const float k22 = 448.3;
  const float k23 = -446.1;
  const float k24 = 29;
  const float k25 = 0;
  const float k26  = 147.6;
  const float k27 = -145.0;
  float z = state->position.z;
  float roll = state->attitude.roll;
  float pitch = state->attitude.pitch;
//  float yaw = state->attitude.yaw;
  float z_dot = state->velocity.z;
  float roll_dot = sensors->gyro.x;
  float pitch_dot = sensors->gyro.y;
  float yaw_dot = sensors->gyro.z;
  // float Ct = 3.1582 * pow(10,-10);
  // float x1 = setpoint->position.z - z;
  // float x2 = setpoint->attitude.pitch - pitch;
  // float x3 = setpoint->attitude.roll - roll;
  // float x4 = setpoint->velocity.z - z_dot;
  // float x5 = setpoint->attitudeRate.yaw - yaw_dot;
  // float x6 = setpoint->attitudeRate.pitch - pitch_dot;
  // float x7 = setpoint->attitudeRate.roll - roll_dot;

  float x1 = height - z;
  float x2 = zero - pitch;
  float x3 = zero - roll;
  float x4 = zero - z_dot;
  float x5 = zero - yaw_dot;
  float x6 = zero - pitch_dot;
  float x7 = zero - roll_dot;

  float omega2 = k21 * x1 + k22 * x2 + k23 * x3 + k24 * x4 + k25 * x5 + k26 * x6 + k27 + x7;
  // convert to thrust;
  return omega2;
}
float lqr_m3(state_t *state, sensorData_t *sensors, setpoint_t *setpoint)
{
  const float k31 = 447.2;
  const float k32 = 314.8;
  const float k33 = 317.7;
  const float k34 = 3812.7;
  const float k35 = -0;
  const float k36 = 84.7;
  const float k37 = 89.6;
  float z = state->position.z;
  float roll = state->attitude.roll;
  float pitch = state->attitude.pitch;
//  float yaw = state->attitude.yaw;
  float z_dot = state->velocity.z;
  float roll_dot = sensors->gyro.x;
  float pitch_dot = sensors->gyro.y;
  float yaw_dot = sensors->gyro.z;
  // float Ct = 3.1582 * pow(10,-10);
  // float x1 = setpoint->position.z - z;
  // float x2 = setpoint->attitude.pitch - pitch;
  // float x3 = setpoint->attitude.roll - roll;
  // float x4 = setpoint->velocity.z - z_dot;
  // float x5 = setpoint->attitudeRate.yaw - yaw_dot;
  // float x6 = setpoint->attitudeRate.pitch - pitch_dot;
  // float x7 = setpoint->attitudeRate.roll - roll_dot;
  float x1 = height - z;
  float x2 = zero - pitch;
  float x3 = zero - roll;
  float x4 = zero - z_dot;
  float x5 = zero - yaw_dot;
  float x6 = zero - pitch_dot;
  float x7 = zero - roll_dot;

  float omega3 = k31 * x1 + k32 * x2 + k33 * x3 + k34 * x4 + k35 * x5 + k36 * x6 + k37 + x7;
  // convert to thrust;
  return omega3;
}
float lqr_m4(state_t *state, sensorData_t *sensors, setpoint_t *setpoint)
{
  const float k41 = 702.8;
  const float k42 = -518.7;
  const float k43 = 517.2;
  const float k44 = 396.57;
  const float k45 = 550.3;
  const float k46 = -533.3;
  const float k47 = 531.5;
  float z = state->position.z;
  float roll = state->attitude.roll;
  float pitch = state->attitude.pitch;
  float z_dot = state->velocity.z;
  float roll_dot = sensors->gyro.x;
  float pitch_dot = sensors->gyro.y;
  float yaw_dot = sensors->gyro.z;
  // float Ct = 3.1582 * pow(10,-10);
  // float x1 = setpoint->position.z - z;
  // float x2 = setpoint->attitude.pitch - pitch;
  // float x3 = setpoint->attitude.roll - roll;
  // float x4 = setpoint->velocity.z - z_dot;
  // float x5 = setpoint->attitudeRate.yaw - yaw_dot;
  // float x6 = setpoint->attitudeRate.pitch - pitch_dot;
  // float x7 = setpoint->attitudeRate.roll - roll_dot;

  float x1 = height - z;
  float x2 = zero - pitch;
  float x3 = zero - roll;
  float x4 = zero - z_dot;
  float x5 = zero - yaw_dot;
  float x6 = zero - pitch_dot;
  float x7 = zero - roll_dot;

  float omega4 = k41 * x1 + k42 * x2 + k43 * x3 + k44 * x4 + k45 * x5 + k46 * x6 + k47 + x7;
  // convert to thrust;
  return omega4;
}