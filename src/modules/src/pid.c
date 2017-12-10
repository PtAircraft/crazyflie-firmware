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

#include <math.h>
//
//#include "FreeRTOS.h"
//#include "task.h"
//
//#include "system.h"
//#include "log.h"
//#include "param.h"
//
//#include "stabilizer.h"
//
//#include "sensors.h"
//#include "commander.h"
//#include "crtp_localization_service.h"
//#include "sitaw.h"
//#include "controller.h"
//#include "power_distribution.h"
//
//#include "estimator_kalman.h"
#include "estimator.h"

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


//////////////////*************///////////////////////
float lqr_m1(state_t *state, sensorData_t *sensors, setpoint_t *setpoint)
{
	const float k11 = 5317.15;
	const float k12 = -5638.62;
	const float k13 = -5905.65;
	const float k14 = -9350.27;
	const float k15 = -307.46;
	const float k16 = -458.50;
	const float k17 = -493.05;
	float z = state->position.z;
	float roll = state->attitude.roll;
	float pitch = state->attitude.pitch;
//	float yaw = state->attitude.yaw;
	float z_dot = state->velocity.z;
	float roll_dot = sensors->gyro.x;
	float pitch_dot = sensors->gyro.y;
	float yaw_dot = sensors->gyro.z;
	float Ct = 3.1582 * pow(10,-10);
	float x1 = setpoint->position.z - z;
	float x2 = setpoint->attitude.pitch - pitch;
	float x3 = setpoint->attitude.roll - roll;
	float x4 = setpoint->velocity.z - z_dot;
	float x5 = setpoint->attitudeRate.yaw - yaw_dot;
	float x6 = setpoint->attitudeRate.pitch - pitch_dot;
	float x7 = setpoint->attitudeRate.roll - roll_dot;
	float omega1 = k11 * x1 + k12 * x2 + k13 * x3 + k14 * x4 + k15 * x5 + k16 * x6 + k17 + x7;
	// convert to thrust;
	return Ct * omega1 * omega1;
}

float lqr_m2(state_t *state, sensorData_t *sensors, setpoint_t *setpoint)
{
	const float k21 = 5217.27;
	const float k22 = 6439.52;
	const float k23 = -6429.81;
	const float k24 = 9206.82;
	const float k25 = 241.37;
	const float k26  = 545.54;
	const float k27 = -539.97;
	float z = state->position.z;
	float roll = state->attitude.roll;
	float pitch = state->attitude.pitch;
//	float yaw = state->attitude.yaw;
	float z_dot = state->velocity.z;
	float roll_dot = sensors->gyro.x;
	float pitch_dot = sensors->gyro.y;
	float yaw_dot = sensors->gyro.z;
	float Ct = 3.1582 * pow(10,-10);
	float x1 = setpoint->position.z - z;
	float x2 = setpoint->attitude.pitch - pitch;
	float x3 = setpoint->attitude.roll - roll;
	float x4 = setpoint->velocity.z - z_dot;
	float x5 = setpoint->attitudeRate.yaw - yaw_dot;
	float x6 = setpoint->attitudeRate.pitch - pitch_dot;
	float x7 = setpoint->attitudeRate.roll - roll_dot;

	float omega2 = k21 * x1 + k22 * x2 + k23 * x3 + k24 * x4 + k25 * x5 + k26 * x6 + k27 + x7;
	// convert to thrust;
	return Ct * omega2 * omega2;
}
float lqr_m3(state_t *state, sensorData_t *sensors, setpoint_t *setpoint)
{
	const float k31 = 5371.16;
	const float k32 = 5897.9;
	const float k33 = 5651.84;
	const float k34 = 9350.21;
	const float k35 = -307.23;
	const float k36 = 497.21;
	const float k37 = 455.31;
	float z = state->position.z;
	float roll = state->attitude.roll;
	float pitch = state->attitude.pitch;
//	float yaw = state->attitude.yaw;
	float z_dot = state->velocity.z;
	float roll_dot = sensors->gyro.x;
	float pitch_dot = sensors->gyro.y;
	float yaw_dot = sensors->gyro.z;
	float Ct = 3.1582 * pow(10,-10);
	float x1 = setpoint->position.z - z;
	float x2 = setpoint->attitude.pitch - pitch;
	float x3 = setpoint->attitude.roll - roll;
	float x4 = setpoint->velocity.z - z_dot;
	float x5 = setpoint->attitudeRate.yaw - yaw_dot;
	float x6 = setpoint->attitudeRate.pitch - pitch_dot;
	float x7 = setpoint->attitudeRate.roll - roll_dot;

	float omega3 = k31 * x1 + k32 * x2 + k33 * x3 + k34 * x4 + k35 * x5 + k36 * x6 + k37 + x7;
	// convert to thrust;
	return Ct * omega3 * omega3;
}
float lqr_m4(state_t *state, sensorData_t *sensors, setpoint_t *setpoint)
{
	const float k41 = 6958.08;
	const float k42 = -5028.59;
	const float k43 = 5017.07;
	const float k44 = 12282.62;
	const float k45 = 293.52;
	const float k46 = -446.6;
	const float k47 = 441.38;
	float z = state->position.z;
	float roll = state->attitude.roll;
	float pitch = state->attitude.pitch;
//	float yaw = state->attitude.yaw;
	float z_dot = state->velocity.z;
	float roll_dot = sensors->gyro.x;
	float pitch_dot = sensors->gyro.y;
	float yaw_dot = sensors->gyro.z;
	float Ct = 3.1582 * pow(10,-10);
	float x1 = setpoint->position.z - z;
	float x2 = setpoint->attitude.pitch - pitch;
	float x3 = setpoint->attitude.roll - roll;
	float x4 = setpoint->velocity.z - z_dot;
	float x5 = setpoint->attitudeRate.yaw - yaw_dot;
	float x6 = setpoint->attitudeRate.pitch - pitch_dot;
	float x7 = setpoint->attitudeRate.roll - roll_dot;

	float omega4 = k41 * x1 + k42 * x2 + k43 * x3 + k44 * x4 + k45 * x5 + k46 * x6 + k47 + x7;
	// convert to thrust;
	return Ct * omega4 * omega4;
}



