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
#define DEBUG_MODULE "STAB"

#include <math.h>
#include <string.h>
#include "FreeRTOS.h"
#include "task.h"

#include "system.h"
#include "log.h"
#include "param.h"
#include "debug.h"
#include "motors.h"
#include "pm.h"

#include "config.h"

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
#include "controller_primitives.h"
#include "power_distribution.h"

#include "estimator_kalman.h"
#include "estimator.h"

static bool isInit;
static bool emergencyStop = false;
static int emergencyStopTimeout = EMERGENCY_STOP_TIMEOUT_DISABLED;

#define PROPTEST_NBR_OF_VARIANCE_VALUES   100
static bool startPropTest = false;

uint32_t inToOutLatency;

// State variables for the stabilizer
static setpoint_t setpoint;
static setpoint_t setpoint2;  // only use for onboard primitives
static sensorData_t sensorData;
static state_t state;
static control_t control;
static setpoint_t setpoint_record;

static StateEstimatorType estimatorType;
static ControllerType controllerType;
static uint8_t usePrimitives = 0;  // 0 - normal, 1 - use onboard primitives

typedef enum { configureAcc, measureNoiseFloor, measureProp, testBattery, restartBatTest, evaluateResult, testDone } TestState;
#ifdef RUN_PROP_TEST_AT_STARTUP
  static TestState testState = configureAcc;
#else
  static TestState testState = testDone;
#endif

static void stabilizerTask(void* param);
static void testProps(sensorData_t *sensors);

static void calcSensorToOutputLatency(const sensorData_t *sensorData)
{
  uint64_t outTimestamp = usecTimestamp();
  inToOutLatency = outTimestamp - sensorData->interruptTimestamp;
}

void stabilizerInit(StateEstimatorType estimator)
{
  if(isInit)
    return;

  sensorsInit();
  stateEstimatorInit(estimator);
  controllerInit(ControllerTypeAny);
  powerDistributionInit();
  if (estimator == kalmanEstimator)
  {
    sitAwInit();
  }
  estimatorType = getStateEstimator();
  controllerType = getControllerType();

  xTaskCreate(stabilizerTask, STABILIZER_TASK_NAME,
              STABILIZER_TASK_STACKSIZE, NULL, STABILIZER_TASK_PRI, NULL);

  isInit = true;
}

bool stabilizerTest(void)
{
  bool pass = true;

  pass &= sensorsTest();
  pass &= stateEstimatorTest();
  pass &= controllerTest();
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
    vTaskDelayUntil(&lastWakeTime, F2T(RATE_MAIN_LOOP));     // make the 1000 HZ working
  }
  // Initialize tick to something else then 0
  tick = 1;

  while(1) {
    // The sensor should unlock at 1kHz
    sensorsWaitDataReady();
    //vTaskDelayUntil(&lastWakeTime, F2T(RATE_MAIN_LOOP));
    // ifndef here, ifdef above
    #ifndef BROADCAST_ENABLE
    getExtPosition(&state);
    #else
//    getExtPositionBC(&state);
//    getExtPosVelBC(&state); // we are currently here
    getExtPosVelYawBC(&state); // [CHANGE] yaw estimation
    #endif

    if (startPropTest != false) {
      // TODO: What happens with estimator when we run tests after startup?
      testState = configureAcc;
      startPropTest = false;
    }

    if (testState != testDone) {
      sensorsAcquire(&sensorData, tick);
      testProps(&sensorData);
    } else {
      // allow to update estimator dynamically
      if (getStateEstimator() != estimatorType) {
        stateEstimatorInit(estimatorType);
        estimatorType = getStateEstimator();
      }
      // allow to update controller dynamically
      if (getControllerType() != controllerType) {
        controllerInit(controllerType);
        controllerType = getControllerType();
      }

      stateEstimator(&state, &sensorData, &control, tick);
      
      commanderGetSetpoint(&setpoint, &state);

      if (usePrimitives)
      {
          // setpoint has the primitive commands
          // setpoint2 will have the position commands
          controllerPrimitives(&setpoint2, &setpoint, &state);
          sitAwUpdateSetpoint(&setpoint2, &sensorData, &state);
          controller(&control, &setpoint2, &sensorData, &state, tick);
      }
      else
      {
          // the normal set-up, setpoint has the (position) commands
          sitAwUpdateSetpoint(&setpoint, &sensorData, &state);
          controller(&control, &setpoint, &sensorData, &state, tick);
      }

      checkEmergencyStopTimeout();

      if (emergencyStop) {
        powerStop();
      } else {
        powerDistribution(&control);
      }
    }
    calcSensorToOutputLatency(&sensorData);
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

static float variance(float *buffer, uint32_t length)
{
  uint32_t i;
  float sum = 0;
  float sumSq = 0;

  for (i = 0; i < length; i++)
  {
    sum += buffer[i];
    sumSq += buffer[i] * buffer[i];
  }

  return sumSq - (sum * sum) / length;
}

/** Evaluate the values from the propeller test
 * @param low The low limit of the self test
 * @param high The high limit of the self test
 * @param value The value to compare with.
 * @param string A pointer to a string describing the value.
 * @return True if self test within low - high limit, false otherwise
 */
static bool evaluateTest(float low, float high, float value, uint8_t motor)
{
  if (value < low || value > high)
  {
    DEBUG_PRINT("Propeller test on M%d [FAIL]. low: %0.2f, high: %0.2f, measured: %0.2f\n",
                motor, (double)low, (double)high, (double)value);
    return false;
  }
  return true;
}


static void testProps(sensorData_t *sensors)
{
  static uint32_t i = 0;
  static float accX[PROPTEST_NBR_OF_VARIANCE_VALUES];
  static float accY[PROPTEST_NBR_OF_VARIANCE_VALUES];
  static float accZ[PROPTEST_NBR_OF_VARIANCE_VALUES];
  static float accVarX[NBR_OF_MOTORS];
  static float accVarY[NBR_OF_MOTORS];
  static float accVarZ[NBR_OF_MOTORS];
  static float accVarXnf;
  static float accVarYnf;
  static float accVarZnf;
  static int motorToTest = 0;
  static uint8_t nrFailedTests = 0;

  static float idleVoltage;
  static float minSingleLoadedVoltage[NBR_OF_MOTORS];
  static float minLoadedVoltage;

  if (testState == configureAcc)
  {
    sensorsSetAccMode(ACC_MODE_PROPTEST);
    testState = measureNoiseFloor;
    minLoadedVoltage = idleVoltage = pmGetBatteryVoltage();
    minSingleLoadedVoltage[MOTOR_M1] = minLoadedVoltage;
    minSingleLoadedVoltage[MOTOR_M2] = minLoadedVoltage;
    minSingleLoadedVoltage[MOTOR_M3] = minLoadedVoltage;
    minSingleLoadedVoltage[MOTOR_M4] = minLoadedVoltage;
  }
  if (testState == measureNoiseFloor)
  {
    accX[i] = sensors->acc.x;
    accY[i] = sensors->acc.y;
    accZ[i] = sensors->acc.z;

    if (++i >= PROPTEST_NBR_OF_VARIANCE_VALUES)
    {
      i = 0;
      accVarXnf = variance(accX, PROPTEST_NBR_OF_VARIANCE_VALUES);
      accVarYnf = variance(accY, PROPTEST_NBR_OF_VARIANCE_VALUES);
      accVarZnf = variance(accZ, PROPTEST_NBR_OF_VARIANCE_VALUES);
      DEBUG_PRINT("Acc noise floor variance X+Y:%f, (Z:%f)\n",
                  (double)accVarXnf + (double)accVarYnf, (double)accVarZnf);
      testState = measureProp;
    }

  }
  else if (testState == measureProp)
  {
    if (i < PROPTEST_NBR_OF_VARIANCE_VALUES)
    {
      accX[i] = sensors->acc.x;
      accY[i] = sensors->acc.y;
      accZ[i] = sensors->acc.z;
      if (pmGetBatteryVoltage() < minSingleLoadedVoltage[motorToTest])
      {
        minSingleLoadedVoltage[motorToTest] = pmGetBatteryVoltage();
      }
    }
    i++;

    if (i == 1)
    {
      motorsSetRatio(motorToTest, 0xFFFF);
    }
    else if (i == 50)
    {
      motorsSetRatio(motorToTest, 0);
    }
    else if (i == PROPTEST_NBR_OF_VARIANCE_VALUES)
    {
      accVarX[motorToTest] = variance(accX, PROPTEST_NBR_OF_VARIANCE_VALUES);
      accVarY[motorToTest] = variance(accY, PROPTEST_NBR_OF_VARIANCE_VALUES);
      accVarZ[motorToTest] = variance(accZ, PROPTEST_NBR_OF_VARIANCE_VALUES);
      DEBUG_PRINT("Motor M%d variance X+Y:%f (Z:%f)\n",
                   motorToTest+1, (double)accVarX[motorToTest] + (double)accVarY[motorToTest],
                   (double)accVarZ[motorToTest]);
    }
    else if (i >= 1000)
    {
      i = 0;
      motorToTest++;
      if (motorToTest >= NBR_OF_MOTORS)
      {
        i = 0;
        motorToTest = 0;
        testState = evaluateResult;
        sensorsSetAccMode(ACC_MODE_FLIGHT);
      }
    }
  }
  else if (testState == testBattery)
  {
    if (i == 0)
    {
      minLoadedVoltage = idleVoltage = pmGetBatteryVoltage();
    }
    if (i == 1)
    {
      motorsSetRatio(MOTOR_M1, 0xFFFF);
      motorsSetRatio(MOTOR_M2, 0xFFFF);
      motorsSetRatio(MOTOR_M3, 0xFFFF);
      motorsSetRatio(MOTOR_M4, 0xFFFF);
    }
    else if (i < 50)
    {
      if (pmGetBatteryVoltage() < minLoadedVoltage)
        minLoadedVoltage = pmGetBatteryVoltage();
    }
    else if (i == 50)
    {
      motorsSetRatio(MOTOR_M1, 0);
      motorsSetRatio(MOTOR_M2, 0);
      motorsSetRatio(MOTOR_M3, 0);
      motorsSetRatio(MOTOR_M4, 0);
//      DEBUG_PRINT("IdleV: %f, minV: %f, M1V: %f, M2V: %f, M3V: %f, M4V: %f\n", (double)idleVoltage,
//                  (double)minLoadedVoltage,
//                  (double)minSingleLoadedVoltage[MOTOR_M1],
//                  (double)minSingleLoadedVoltage[MOTOR_M2],
//                  (double)minSingleLoadedVoltage[MOTOR_M3],
//                  (double)minSingleLoadedVoltage[MOTOR_M4]);
      DEBUG_PRINT("%f %f %f %f %f %f\n", (double)idleVoltage,
                  (double)(idleVoltage - minLoadedVoltage),
                  (double)(idleVoltage - minSingleLoadedVoltage[MOTOR_M1]),
                  (double)(idleVoltage - minSingleLoadedVoltage[MOTOR_M2]),
                  (double)(idleVoltage - minSingleLoadedVoltage[MOTOR_M3]),
                  (double)(idleVoltage - minSingleLoadedVoltage[MOTOR_M4]));
      testState = restartBatTest;
      i = 0;
    }
    i++;
  }
  else if (testState == restartBatTest)
  {
    if (i++ > 2000)
    {
      testState = configureAcc;
      i = 0;
    }
  }
  else if (testState == evaluateResult)
  {
    for (int m = 0; m < NBR_OF_MOTORS; m++)
    {
      if (!evaluateTest(0, PROPELLER_BALANCE_TEST_THRESHOLD,  accVarX[m] + accVarY[m], m + 1))
      {
        nrFailedTests++;
        for (int j = 0; j < 3; j++)
        {
          motorsBeep(m, true, testsound[m], (uint16_t)(MOTORS_TIM_BEEP_CLK_FREQ / A4)/ 20);
          vTaskDelay(M2T(MOTORS_TEST_ON_TIME_MS));
          motorsBeep(m, false, 0, 0);
          vTaskDelay(M2T(100));
        }
      }
    }
#ifdef PLAY_STARTUP_MELODY_ON_MOTORS
    if (nrFailedTests == 0)
    {
      for (int m = 0; m < NBR_OF_MOTORS; m++)
      {
        motorsBeep(m, true, testsound[m], (uint16_t)(MOTORS_TIM_BEEP_CLK_FREQ / A4)/ 20);
        vTaskDelay(M2T(MOTORS_TEST_ON_TIME_MS));
        motorsBeep(m, false, 0, 0);
        vTaskDelay(M2T(MOTORS_TEST_DELAY_TIME_MS));
      }
    }
#endif
    testState = testDone;
  }
}
PARAM_GROUP_START(health)
PARAM_ADD(PARAM_UINT8, startPropTest, &startPropTest)
PARAM_GROUP_STOP(health)


PARAM_GROUP_START(stabilizer)
PARAM_ADD(PARAM_UINT8, estimator, &estimatorType)
PARAM_ADD(PARAM_UINT8, controller, &controllerType)
PARAM_ADD(PARAM_UINT8, usePrims, &usePrimitives)
PARAM_GROUP_STOP(stabilizer)

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

//LOG_GROUP_START(baro)
//LOG_ADD(LOG_FLOAT, asl, &sensorData.baro.asl)
//LOG_ADD(LOG_FLOAT, temp, &sensorData.baro.temperature)
//LOG_ADD(LOG_FLOAT, pressure, &sensorData.baro.pressure)
//LOG_GROUP_STOP(baro)
//
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

//LOG_GROUP_START(mag)
//LOG_ADD(LOG_FLOAT, x, &sensorData.mag.x)
//LOG_ADD(LOG_FLOAT, y, &sensorData.mag.y)
//LOG_ADD(LOG_FLOAT, z, &sensorData.mag.z)
//LOG_GROUP_STOP(mag)
//
//
//LOG_GROUP_START(stateEstimate)
//LOG_ADD(LOG_FLOAT, x, &state.position.x)
//LOG_ADD(LOG_FLOAT, y, &state.position.y)
//LOG_ADD(LOG_FLOAT, z, &state.position.z)
//LOG_GROUP_STOP(stateEstimate)
//
//LOG_GROUP_START(latency)
//LOG_ADD(LOG_UINT32, intToOut, &inToOutLatency)
//LOG_GROUP_STOP(latency)

