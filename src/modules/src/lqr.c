/*
 * lqr.c
 *
 *  Created on: Nov 20, 2017
 *      Author: ptaircraft
 */
#include "pid.h"
#include "num.h"
#include <float.h>
#include <math.h>
#include "lqr.h"
#include <math.h>

#include "FreeRTOS.h"
#include "task.h"

#include "system.h"
#include "log.h"
#include "param.h"

#include "stabilizer.h"

#include "sensors.h"
#include "commander.h"
#include "crtp_localization_service.h"
#include "sitaw.h"
#include "controller.h"
#include "power_distribution.h"

#include "estimator_kalman.h"
#include "estimator.h"
float lqr_m1(state_t *state, sensorData_t *sensors)
{
	const float k11 = 5317.15;
	const float k12 = -5638.62;
	const float k13 = -5905.65;
	const float k14 = -9350.27;
	const float k15 = -307.46;
	const float k16 = -458.50;
	const float k17 = -493.05;
	float z = state->position.z + 0.4;
	float roll = state->attitude.roll;
	float pitch = state->attitude.pitch;
	float yaw = state->attitude.yaw;
	float z_dot = state->velocity.z;
	float roll_dot = sensors->gyro.x;
	float pitch_dot = sensors->gyro.y;
	float yaw_dot = sensors->gyro.z;
	float Ct = 3.1582 * power(10,-10);


	float omega1 = k11 * z + k12 * pitch + k13 * roll + k14 * z_dot + k15 * yaw_dot + k16 * pitch_dot + k17 + roll_dot;
	// convert to thrust;
	return Ct * omega1;
}

float lqr_m2(state_t *state, sensorData_t *sensors)
{
	const float k21 = 5217.27;
	const float k22 = 6439.52;
	const float k23 = -6429.81;
	const float k24 = 9206.82;
	const float k25 = 241.37;
	const float k26  = 545.54;
	const float k27 = -539.97;
	float z = state->position.z + 0.4;
	float roll = state->attitude.roll;
	float pitch = state->attitude.pitch;
	float yaw = state->attitude.yaw;
	float z_dot = state->velocity.z;
	float roll_dot = sensors->gyro.x;
	float pitch_dot = sensors->gyro.y;
	float yaw_dot = sensors->gyro.z;
	float Ct = 3.1582 * power(10,-10);


	float omega2 = k21 * z + k22 * pitch + k23 * roll + k24 * z_dot + k25 * yaw_dot + k26 * pitch_dot + k27 + roll_dot;
	// convert to thrust;
	return Ct * omega2;
}
float lqr_m3(state_t *state, sensorData_t *sensors)
{
	const float k31 = 5371.16;
	const float k32 = 5897.9;
	const float k33 = 5651.84;
	const float k34 = 9350.21;
	const float k35 = -307.23;
	const float k36 = 497.21;
	const float k37 = 455.31;
	float z = state->position.z + 0.4;
	float roll = state->attitude.roll;
	float pitch = state->attitude.pitch;
	float yaw = state->attitude.yaw;
	float z_dot = state->velocity.z;
	float roll_dot = sensors->gyro.x;
	float pitch_dot = sensors->gyro.y;
	float yaw_dot = sensors->gyro.z;
	float Ct = 3.1582 * power(10,-10);


	float omega3 = k31 * z + k32 * pitch + k33 * roll + k34 * z_dot + k35 * yaw_dot + k36 * pitch_dot + k37 + roll_dot;
	// convert to thrust;
	return Ct * omega3;
}
float lqr_m4(state_t *state, sensorData_t *sensors)
{
	const float k41 = 6958.08;
	const float k42 = -5028.59;
	const float k43 = 5017.07;
	const float k44 = 12282.62;
	const float k45 = 293.52;
	const float k46 = -446.6;
	const float k47 = 441.38;
	float z = state->position.z + 0.4;
	float roll = state->attitude.roll;
	float pitch = state->attitude.pitch;
	float yaw = state->attitude.yaw;
	float z_dot = state->velocity.z;
	float roll_dot = sensors->gyro.x;
	float pitch_dot = sensors->gyro.y;
	float yaw_dot = sensors->gyro.z;
	float Ct = 3.1582 * power(10,-10);


	float omega4 = k41 * z + k42 * pitch + k43 * roll + k44 * z_dot + k45 * yaw_dot + k46 * pitch_dot + k47 + roll_dot;
	// convert to thrust;
	return Ct * omega4;
}

