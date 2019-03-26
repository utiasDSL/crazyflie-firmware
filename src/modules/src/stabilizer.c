/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie Firmware
 *
 * Copyright (C) 2011-2016 Bitcraze AB
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
 */
#include <math.h>
#include <string.h>
#include "FreeRTOS.h"
#include "task.h"

#include "system.h"
#include "log.h"
#include "param.h"

#include "stabilizer.h"

#include "sensors.h"
#include "commander.h"

// note: BROADCAST_ENABLE is set in config.mk
#ifdef BROADCAST_ENABLE
#include "crtp_broadcast_service.h"
#else
#include "crtp_localization_service.h"
#endif

#include "sitaw.h"
#include "controller.h"
#include "power_distribution.h"

#include "estimator_kalman.h"
#include "estimator.h"

static bool isInit;
static bool emergencyStop = false;
static int emergencyStopTimeout = EMERGENCY_STOP_TIMEOUT_DISABLED;

// State variables for the stabilizer
static setpoint_t setpoint;
static sensorData_t sensorData;
static state_t state;
static control_t control;
static setpoint_t setpoint_record;

static void stabilizerTask(void* param);

void stabilizerInit(StateEstimatorType estimator)
{
  if(isInit)
    return;

  sensorsInit();
  stateEstimatorInit(estimator);
  stateControllerInit();
  powerDistributionInit();
#if defined(SITAW_ENABLED)
  sitAwInit();
#endif

  xTaskCreate(stabilizerTask, STABILIZER_TASK_NAME,
              STABILIZER_TASK_STACKSIZE, NULL, STABILIZER_TASK_PRI, NULL);

  isInit = true;
}

bool stabilizerTest(void)
{
  bool pass = true;

  pass &= sensorsTest();
  pass &= stateEstimatorTest();
  pass &= stateControllerTest();
  pass &= powerDistributionTest();

  return pass;
}

static void checkEmergencyStopTimeout()
{
  if (emergencyStopTimeout >= 0) {
    emergencyStopTimeout -= 1;

    if (emergencyStopTimeout == 0) {
      emergencyStop = true;
    }
  }
}

/* The stabilizer loop runs at 1kHz (stock) or 500Hz (kalman). It is the
 * responsibility of the different functions to run slower by skipping call
 * (ie. returning without modifying the output structure).
 */

static void stabilizerTask(void* param)
{
  uint32_t tick;
  uint32_t lastWakeTime;
  vTaskSetApplicationTaskTag(0, (void*)TASK_STABILIZER_ID_NBR);

  //Wait for the system to be fully started to start stabilization loop
  systemWaitStart();

  // Wait for sensors to be calibrated
  lastWakeTime = xTaskGetTickCount ();
  while(!sensorsAreCalibrated()) {
    vTaskDelayUntil(&lastWakeTime, F2T(RATE_MAIN_LOOP));
  }
  // Initialize tick to something else then 0
  tick = 1;

  while(1) {
    vTaskDelayUntil(&lastWakeTime, F2T(RATE_MAIN_LOOP));
    // ifndef here, ifdef above
    #ifndef BROADCAST_ENABLE
    getExtPosition(&state);
    #else
//    getExtPositionBC(&state);
//    getExtPosVelBC(&state); // we are currently here
    getExtPosVelYawBC(&state); // [CHANGE] yaw estimation
    #endif

    stateEstimator(&state, &sensorData, &control, tick);

    commanderGetSetpoint(&setpoint, &state);

    sitAwUpdateSetpoint(&setpoint, &sensorData, &state);

    stateController(&control, &setpoint, &sensorData, &state, tick);

    memcpy(&setpoint_record, &setpoint, sizeof(setpoint));
    checkEmergencyStopTimeout();

    if (emergencyStop) {
      powerStop();
    } else {
      powerDistribution(&control);
    }

    tick++;
  }
}

void stabilizerSetEmergencyStop()
{
  emergencyStop = true;
}

void stabilizerResetEmergencyStop()
{
  emergencyStop = false;
}

void stabilizerSetEmergencyStopTimeout(int timeout)
{
  emergencyStop = false;
  emergencyStopTimeout = timeout;
}

LOG_GROUP_START(ctrltarget)
LOG_ADD(LOG_FLOAT, X, &setpoint_record.position.x)
LOG_ADD(LOG_FLOAT, Y, &setpoint_record.position.y)
LOG_ADD(LOG_FLOAT, Z, &setpoint_record.position.z)

LOG_ADD(LOG_FLOAT, Vx, &setpoint_record.velocity.x)
LOG_ADD(LOG_FLOAT, Vy, &setpoint_record.velocity.y)
LOG_ADD(LOG_FLOAT, Vz, &setpoint_record.velocity.z)

LOG_ADD(LOG_FLOAT, Accx, &setpoint_record.acceleration.x)
LOG_ADD(LOG_FLOAT, Accy, &setpoint_record.acceleration.y)
LOG_ADD(LOG_FLOAT, Accz, &setpoint_record.acceleration.z)

LOG_ADD(LOG_FLOAT, roll, &setpoint_record.attitude.roll)
LOG_ADD(LOG_FLOAT, pitch, &setpoint_record.attitude.pitch)
LOG_ADD(LOG_FLOAT, yaw,  &setpoint_record.attitude.yaw)

LOG_ADD(LOG_FLOAT, yawr, &setpoint_record.attitudeRate.yaw)
LOG_ADD(LOG_FLOAT, thrust, &setpoint_record.thrust)

LOG_GROUP_STOP(ctrltarget)

LOG_GROUP_START(eststate)
LOG_ADD(LOG_FLOAT, X, &state.position.x)
LOG_ADD(LOG_FLOAT, Y, &state.position.y)
LOG_ADD(LOG_FLOAT, Z, &state.position.z)

LOG_ADD(LOG_FLOAT, Vx, &state.velocity.x)
LOG_ADD(LOG_FLOAT, Vy, &state.velocity.y)
LOG_ADD(LOG_FLOAT, Vz, &state.velocity.z)

LOG_ADD(LOG_FLOAT, roll, &state.attitude.roll)
LOG_ADD(LOG_FLOAT, pitch, &state.attitude.pitch)
LOG_ADD(LOG_FLOAT, yaw, &state.attitude.yaw)

LOG_GROUP_STOP(eststate)

LOG_GROUP_START(acc)
LOG_ADD(LOG_FLOAT, x, &sensorData.acc.x)
LOG_ADD(LOG_FLOAT, y, &sensorData.acc.y)
LOG_ADD(LOG_FLOAT, z, &sensorData.acc.z)
LOG_GROUP_STOP(acc)

#ifdef LOG_SEC_IMU
LOG_GROUP_START(accSec)
LOG_ADD(LOG_FLOAT, x, &sensorData.accSec.x)
LOG_ADD(LOG_FLOAT, y, &sensorData.accSec.y)
LOG_ADD(LOG_FLOAT, z, &sensorData.accSec.z)
LOG_GROUP_STOP(accSec)
#endif

LOG_GROUP_START(baro)
LOG_ADD(LOG_FLOAT, asl, &sensorData.baro.asl)
LOG_ADD(LOG_FLOAT, temp, &sensorData.baro.temperature)
LOG_ADD(LOG_FLOAT, pressure, &sensorData.baro.pressure)
LOG_GROUP_STOP(baro)

LOG_GROUP_START(gyro)
LOG_ADD(LOG_FLOAT, x, &sensorData.gyro.x)
LOG_ADD(LOG_FLOAT, y, &sensorData.gyro.y)
LOG_ADD(LOG_FLOAT, z, &sensorData.gyro.z)
LOG_GROUP_STOP(gyro)

#ifdef LOG_SEC_IMU
LOG_GROUP_START(gyroSec)
LOG_ADD(LOG_FLOAT, x, &sensorData.gyroSec.x)
LOG_ADD(LOG_FLOAT, y, &sensorData.gyroSec.y)
LOG_ADD(LOG_FLOAT, z, &sensorData.gyroSec.z)
LOG_GROUP_STOP(gyroSec)
#endif

LOG_GROUP_START(mag)
LOG_ADD(LOG_FLOAT, x, &sensorData.mag.x)
LOG_ADD(LOG_FLOAT, y, &sensorData.mag.y)
LOG_ADD(LOG_FLOAT, z, &sensorData.mag.z)
LOG_GROUP_STOP(mag)

