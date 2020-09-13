/**
 * Authored by Michael Hamer (http://www.mikehamer.info), June 2016
 * Thank you to Mark Mueller (www.mwm.im) for advice during implementation,
 * and for derivation of the original filter in the below-cited paper.
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
 * ============================================================================
 *
 * The Kalman filter implemented in this file is based on the papers:
 *
 * "Fusing ultra-wideband range measurements with accelerometers and rate gyroscopes for quadrocopter state estimation"
 * http://ieeexplore.ieee.org/xpl/articleDetails.jsp?arnumber=7139421
 *
 * and
 *
 * "Covariance Correction Step for Kalman Filtering with an Attitude"
 * http://arc.aiaa.org/doi/abs/10.2514/1.G000848
 *
 * Academic citation would be appreciated.
 *
 * BIBTEX ENTRIES:
      @INPROCEEDINGS{MuellerHamerUWB2015,
      author  = {Mueller, Mark W and Hamer, Michael and D’Andrea, Raffaello},
      title   = {Fusing ultra-wideband range measurements with accelerometers and rate gyroscopes for quadrocopter state estimation},
      booktitle = {2015 IEEE International Conference on Robotics and Automation (ICRA)},
      year    = {2015},
      month   = {May},
      pages   = {1730-1736},
      doi     = {10.1109/ICRA.2015.7139421},
      ISSN    = {1050-4729}}

      @ARTICLE{MuellerCovariance2016,
      author={Mueller, Mark W and Hehn, Markus and D’Andrea, Raffaello},
      title={Covariance Correction Step for Kalman Filtering with an Attitude},
      journal={Journal of Guidance, Control, and Dynamics},
      pages={1--7},
      year={2016},
      publisher={American Institute of Aeronautics and Astronautics}}
 *
 * ============================================================================
 *
 * MAJOR CHANGELOG:
 * 2016.06.28, Mike Hamer: Initial version
 *
 */

#include "estimator_kalman.h"
#include "outlierFilter.h"
// use printf
#include <stdio.h>

#include "stm32f4xx.h"

#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"
#include "sensors.h"

#include "log.h"
#include "param.h"

#include "math.h"
#include "cf_math.h"

#include "debug.h"
// [change] include the nn model
#include "dsl_dnn.h"

//#define KALMAN_USE_BARO_UPDA
//#define KALMAN_NAN_CHECK
static bool enable_flow = false;
static bool enable_zrange = false;
static bool enable_UWB = true;

static bool OUTLIER_REJ = false;            // Model based outlier rejection
static bool CHI_SQRUARE = false;             // Chi-square test
static bool THREE_SIGMA = false;            // 3 sigma test
static bool DNN_COM = true;                 // DNN bias compensation for TDoA measurements
static bool ROBUST = true;                  // Use robust Kalman filter
// q_an = [q.w, q.x, q.y, q.z]
// 0912
static float q_an[8][4] ={{-0.32855428,  0.62340112,  0.31318984,  0.63665944},  // 0
                          { 0.59999234,  0.79323699,  0.10362054,  0.00685946},  // 1
                          {0.65082884,   0.29064111, -0.64062828,  0.28556081},   // 2
                          {-0.06297648, -0.09271534,  0.73607473,  0.6675566},  // 3
                          {-0.61563234,  0.78300854, -0.0733227,   0.05018209 },  // 4
                          {0.60832498,  -0.38012147, -0.58610896, -0.3767289},  // 5
                          {-0.14442833,  0.21802605,  0.75479003, -0.60157884},  // 6
                          {0.29906281,   0.62233458, -0.32174032,  0.64787674}   // 7
                                };
/**
 * Primary Kalman filter functions
 *
 * The filter progresses as:
 *  - Predicting the current state forward */
static void stateEstimatorPredict(float thrust, Axis3f *acc, Axis3f *gyro, float dt);
static void stateEstimatorAddProcessNoise(float dt);
static bool first_vicon = true;

/*  - Measurement updates based on sensors */
static void stateEstimatorScalarUpdate(arm_matrix_instance_f32 *Hm, float error, float stdMeasNoise);
static void stateEstimatorUpdateWithAccOnGround(Axis3f *acc);
#ifdef KALMAN_USE_BARO_UPDATE
static void stateEstimatorUpdateWithBaro(baro_t *baro);
#endif

/*  - Finalization to incorporate attitude error into body attitude */
static void stateEstimatorFinalize(sensorData_t *sensors, uint32_t tick);

/*  - Externalization to move the filter's internal state into the external state expected by other modules */
static void stateEstimatorExternalizeState(state_t *state, sensorData_t *sensors, uint32_t tick);


/**
 * Additionally, the filter supports the incorporation of additional sensors into the state estimate
 *
 * This is done via the external functions:
 * - bool estimatorKalmanEnqueueUWBPacket(uwbPacket_t *uwb)
 * - bool estimatorKalmanEnqueuePosition(positionMeasurement_t *pos)
 * - bool estimatorKalmanEnqueueDistance(distanceMeasurement_t *dist)
 *
 * As well as by the following internal functions and datatypes
 */

// Distance-to-point measurements
static xQueueHandle distDataQueue;
#define DIST_QUEUE_LENGTH (10)

// Measurements of a UWB TWR
static void stateEstimatorUpdateWithDistance(distanceMeasurement_t *dist, float dt);

static inline bool stateEstimatorHasDistanceMeasurement(distanceMeasurement_t *dist) {
  return (pdTRUE == xQueueReceive(distDataQueue, dist, 0));
}

// Direct measurements of Crazyflie position
static xQueueHandle posDataQueue;
#define POS_QUEUE_LENGTH (10)

static void stateEstimatorUpdateWithPosition(positionMeasurement_t *pos);

static inline bool stateEstimatorHasPositionMeasurement(positionMeasurement_t *pos) {
  return (pdTRUE == xQueueReceive(posDataQueue, pos, 0));
}

// Direct measurements of Crazyflie position and velocity
static xQueueHandle posvelDataQueue;
#define POSVEL_QUEUE_LENGTH (10)

static void stateEstimatorUpdateWithPosVel(posvelMeasurement_t *posvel, sensorData_t *sensors);

static inline bool stateEstimatorHasPosVelMeasurement(posvelMeasurement_t *posvel) {
  return (pdTRUE == xQueueReceive(posvelDataQueue, posvel, 0));
}

// [CHANGE] yaw estimation. Direct measurements of Crazyflie position, velocity, and yaw
static xQueueHandle posvelyawDataQueue;
#define POSVELYAW_QUEUE_LENGTH (10)

static void stateEstimatorUpdateWithPosVelYaw(posvelyawMeasurement_t *posvelyaw, sensorData_t *sensors);

static inline bool stateEstimatorHasPosVelYawMeasurement(posvelyawMeasurement_t *posvelyaw) {
  return (pdTRUE == xQueueReceive(posvelyawDataQueue, posvelyaw, 0));
}

static void stateEstimatorUpdateWithTDOA(tdoaMeasurement_t *uwb, float dt);

// [CHANGE] Define a new robust update EKF with tdoa measurements (with help functions)
static void robustEstimatorUpdateWithTDOA(tdoaMeasurement_t *uwb);
static void Cholesky_Decomposition(int n, float matrix[n][n],  float lower[n][n]);
static void GM_UWB(float e, float * GM_e);
static void GM_state(float e, float * GM_e);
static void matrixcopy(int ROW, int COLUMN, float destmat[ROW][COLUMN], float srcmat[ROW][COLUMN]);
static void vectorcopy(int DIM, float destVec[DIM], float srcVec[DIM]);

// [CHANGE] help functions for dnn //
static void quat2Rot(float q[4], float R[3][3]);
static void RT_v(float v[3], float C[3][3], float v_b[3]);
static void getAzEl_Angle(float v_cf0[3], float v_cf1[3], float v_an0[3], float v_an1[3], float C_IB[3][3], 
                          float q_IA0[4], float q_IA1[4], float AzEl[8]);

// --------------------------------------------------------------------- //
// Measurements of a UWB Tx/Rx
static xQueueHandle tdoaDataQueue;
#define UWB_QUEUE_LENGTH (10)
static inline bool stateEstimatorHasTDOAPacket(tdoaMeasurement_t *uwb) {
  return (pdTRUE == xQueueReceive(tdoaDataQueue, uwb, 0));
}

// Measurements of flow (dnx, dny)
static xQueueHandle flowDataQueue;
#define FLOW_QUEUE_LENGTH (10)

static void stateEstimatorUpdateWithFlow(flowMeasurement_t *flow, sensorData_t *sensors);

static inline bool stateEstimatorHasFlowPacket(flowMeasurement_t *flow) {
  return (pdTRUE == xQueueReceive(flowDataQueue, flow, 0));
}

// Measurements of TOF from laser sensor
static xQueueHandle tofDataQueue;
#define TOF_QUEUE_LENGTH (10)

static void stateEstimatorUpdateWithTof(tofMeasurement_t *tof);

static inline bool stateEstimatorHasTOFPacket(tofMeasurement_t *tof) {
  return (pdTRUE == xQueueReceive(tofDataQueue, tof, 0));
}

// Absolute height measurement along the room Z
static xQueueHandle heightDataQueue;
#define HEIGHT_QUEUE_LENGTH (10)

static void stateEstimatorUpdateWithAbsoluteHeight(heightMeasurement_t *height);

static inline bool stateEstimatorHasHeightPacket(heightMeasurement_t *height) {
  return (pdTRUE == xQueueReceive(heightDataQueue, height, 0));
}

/**
 * Constants used in the estimator
 */

#define DEG_TO_RAD (PI/180.0f)
#define RAD_TO_DEG (180.0f/PI)

#define GRAVITY_MAGNITUDE (9.81f) // we use the magnitude such that the sign/direction is explicit in calculations
#define CRAZYFLIE_WEIGHT_grams (32.3f)

//thrust is thrust mapped for 65536 <==> 60 GRAMS!
#ifdef CONTROLLER_TYPE_new
#define CONTROL_TO_ACC (1.0f)
#else
#define CONTROL_TO_ACC (GRAVITY_MAGNITUDE*60.0f/CRAZYFLIE_WEIGHT_grams/65536.0f)
#endif

// TODO: Decouple the TDOA implementation from the Kalman filter...
#define METERS_PER_TDOATICK (4.691763979e-3f)
#define SECONDS_PER_TDOATICK (15.650040064e-12f)


/**
 * Tuning parameters
 */
#define PREDICT_RATE RATE_100_HZ // this is slower than the IMU update rate of 500Hz
#define BARO_RATE RATE_25_HZ

// the point at which the dynamics change from stationary to flying
#define IN_FLIGHT_THRUST_THRESHOLD (GRAVITY_MAGNITUDE*0.1f)
#define IN_FLIGHT_TIME_THRESHOLD (500)

// the reversion of pitch and roll to zero
#ifdef LPS_2D_POSITION_HEIGHT
#define ROLLPITCH_ZERO_REVERSION (0.0f)
#else
#define ROLLPITCH_ZERO_REVERSION (0.001f)
#endif

// The bounds on the covariance, these shouldn't be hit, but sometimes are... why?
#define MAX_COVARIANCE (100)
#define MIN_COVARIANCE (1e-6f)

// The bounds on states, these shouldn't be hit...
#define MAX_POSITION (100) //meters
#define MAX_VELOCITY (10) //meters per second

// Initial variances, uncertain of position, but know we're stationary and roughly flat
// [Change]: we are certain about the initial position (for outlier rejection)
static const float stdDevInitialPosition_xy = 100;
static const float stdDevInitialPosition_z = 1;
static const float stdDevInitialVelocity = 0.01;
static const float stdDevInitialAttitude_rollpitch = 0.01;
static const float stdDevInitialAttitude_yaw = 0.01;

static float procNoiseAcc_xy = 2.0f;
static float procNoiseAcc_z = 2.0f;
static float procNoiseVel = 0;
static float procNoisePos = 0;
static float procNoiseAtt = 0;
//static float measNoiseBaro = 2.0f; // meters
static float measNoiseGyro_rollpitch = 0.1f; // radians per second
static float measNoiseGyro_yaw = 0.1f; // radians per second

static float initialX = 1.5f;
static float initialY = 0.0f;
static float initialZ = 0.0f;

//static float dragXY = 0.19f;
//static float dragZ = 0.05f;
static float dragXY = 0.0f;
static float dragZ = 0.00f;
// We track a TDOA skew as part of the Kalman filter
static const float stdDevInitialSkew = 0.1;
//static float procNoiseSkew = 10e-6f; // seconds per second^2 (is multiplied by dt to give skew noise)
static float log_yaw = 0.0f;
// Chi-square test debug
static float log_dm = 0.0f;
static float log_errAbs = 0.0f;

static float log_bias = 0.0f;
// static float log_new1[6] = {0};
// static float log_new2[6] = {0};
// static float log_new3[6] = {0};
/**
 * Quadrocopter State
 *
 * The internally-estimated state is:
 * - X, Y, Z: the quad's position in the global frame
 * - PX, PY, PZ: the quad's velocity in its body frame
 * - D0, D1, D2: attitude error
 * - SKEW: the skew from anchor system clock to quad clock
 *
 * For more information, refer to the paper
 */

// The quad's state, stored as a column vector
typedef enum
{
  STATE_X, STATE_Y, STATE_Z, STATE_PX, STATE_PY, STATE_PZ, STATE_D0, STATE_D1, STATE_D2, STATE_DIM
} stateIdx_t;

static float S[STATE_DIM];

// The quad's attitude as a quaternion (w,x,y,z)
// We store as a quaternion to allow easy normalization (in comparison to a rotation matrix),
// while also being robust against singularities (in comparison to euler angles)
static float q[4] = {1,0,0,0};

// The quad's attitude as a rotation matrix (used by the prediction, updated by the finalization)
static float R[3][3] = {{1,0,0},{0,1,0},{0,0,1}};

// The covariance matrix
static float P[STATE_DIM][STATE_DIM];
static arm_matrix_instance_f32 Pm = {STATE_DIM, STATE_DIM, (float *)P};


/**
 * Internal variables. Note that static declaration results in default initialization (to 0)
 */

static bool isInit = false;
static bool resetEstimation = true;
static int32_t lastPrediction;
static int32_t lastBaroUpdate;
static int32_t lastPNUpdate;
static Axis3f accAccumulator;
static float thrustAccumulator;
static Axis3f gyroAccumulator;
static baro_t baroAccumulator;
static uint32_t accAccumulatorCount;
static uint32_t thrustAccumulatorCount;
static uint32_t gyroAccumulatorCount;
static uint32_t baroAccumulatorCount;
static bool quadIsFlying = false;
static int32_t lastTDOAUpdate;
//static float stateSkew;
static float varSkew;
static uint32_t lastFlightCmd;
static uint32_t takeoffTime;
static uint32_t tdoaCount;
static float twrDist;
static int anchorID;
static float yaw_logback;
static float yaw_error_logback;

//debug
static float tdoaDist;
static int tdoaID;
static float logzrange = 0.0f;

/**
 * Supporting and utility functions
 */

static inline void mat_trans(const arm_matrix_instance_f32 * pSrc, arm_matrix_instance_f32 * pDst)
{ configASSERT(ARM_MATH_SUCCESS == arm_mat_trans_f32(pSrc, pDst)); }
static inline void mat_inv(const arm_matrix_instance_f32 * pSrc, arm_matrix_instance_f32 * pDst)
{ configASSERT(ARM_MATH_SUCCESS == arm_mat_inverse_f32(pSrc, pDst)); }
static inline void mat_mult(const arm_matrix_instance_f32 * pSrcA, const arm_matrix_instance_f32 * pSrcB, arm_matrix_instance_f32 * pDst)
{ configASSERT(ARM_MATH_SUCCESS == arm_mat_mult_f32(pSrcA, pSrcB, pDst)); }
static inline float arm_sqrt(float32_t in)
{ float pOut = 0; arm_status result = arm_sqrt_f32(in, &pOut); configASSERT(ARM_MATH_SUCCESS == result); return pOut; }

static inline void mat_add(const arm_matrix_instance_f32 * pSrcA, const arm_matrix_instance_f32 * pSrcB, arm_matrix_instance_f32 * pDst)
{ configASSERT(ARM_MATH_SUCCESS == arm_mat_add_f32(pSrcA, pSrcB, pDst)); }
static inline void mat_scale(const arm_matrix_instance_f32 * pSrcA, float32_t scale, arm_matrix_instance_f32 * pDst)
{ configASSERT(ARM_MATH_SUCCESS == arm_mat_scale_f32(pSrcA, scale, pDst)); }


#ifdef KALMAN_NAN_CHECK
static void stateEstimatorAssertNotNaN() {
  if ((isnan(S[STATE_X])) ||
      (isnan(S[STATE_Y])) ||
      (isnan(S[STATE_Z])) ||
      (isnan(S[STATE_PX])) ||
      (isnan(S[STATE_PY])) ||
      (isnan(S[STATE_PZ])) ||
      (isnan(S[STATE_D0])) ||
      (isnan(S[STATE_D1])) ||
      (isnan(S[STATE_D2])) ||
      (isnan(q[0])) ||
      (isnan(q[1])) ||
      (isnan(q[2])) ||
      (isnan(q[3]))) { resetEstimation = true; }

  for(int i=0; i<STATE_DIM; i++) {
    for(int j=0; j<STATE_DIM; j++)
    {
      if (isnan(P[i][j]))
      {
        resetEstimation = true;
      }
    }
  }
}
#else
static void stateEstimatorAssertNotNaN()
{
  return;
}
#endif

#ifdef KALMAN_DECOUPLE_XY
// Reset a state to 0 with max covariance
// If called often, this decouples the state to the rest of the filter
static void decoupleState(stateIdx_t state)
{
  // Set all covariance to 0
  for(int i=0; i<STATE_DIM; i++) {
    P[state][i] = 0;
    P[i][state] = 0;
  }
  // Set state variance to maximum
  P[state][state] = MAX_COVARIANCE;
  // set state to zero
  S[state] = 0;
}
#endif

// ----------------------- main function --------------------------- //

void estimatorKalman(state_t *state, sensorData_t *sensors, control_t *control, const uint32_t tick)
{
    // ------------------ debug testing ------------------------- //
    // // --- Cholesky is correct! ---//
    // // --- vector/matrixcopy are correct! ---//
    // // --- GM_weight function is correct --- //

    //-----------------------------------------------------------//

    // If the client (via a parameter update) triggers an estimator reset:
    if (resetEstimation) { estimatorKalmanInit(); resetEstimation = false; }

    // Tracks whether an update to the state has been made, and the state therefore requires finalization
    bool doneUpdate = false;

    uint32_t osTick = xTaskGetTickCount(); // would be nice if this had a precision higher than 1ms...

    #ifdef KALMAN_DECOUPLE_XY
    // Decouple position states
    decoupleState(STATE_X);
    decoupleState(STATE_PX);
    decoupleState(STATE_Y);
    decoupleState(STATE_PY);
    #endif

    // Average the last IMU measurements. We do this because the prediction loop is
    // slower than the IMU loop, but the IMU information is required externally at
    // a higher rate (for body rate control).
    if (sensorsReadAcc(&sensors->acc)) {
        accAccumulator.x += GRAVITY_MAGNITUDE*sensors->acc.x; // accelerometer is in Gs
        accAccumulator.y += GRAVITY_MAGNITUDE*sensors->acc.y; // but the estimator requires ms^-2
        accAccumulator.z += GRAVITY_MAGNITUDE*sensors->acc.z;
        accAccumulatorCount++;
    }

    if (sensorsReadGyro(&sensors->gyro)) {
        gyroAccumulator.x += sensors->gyro.x * DEG_TO_RAD; // gyro is in deg/sec
        gyroAccumulator.y += sensors->gyro.y * DEG_TO_RAD; // but the estimator requires rad/sec
        gyroAccumulator.z += sensors->gyro.z * DEG_TO_RAD;
        gyroAccumulatorCount++;
    }

    if (sensorsReadMag(&sensors->mag)) {
        // Currently the magnetometer doesn't play a part in the estimation
    }

    // Average the thrust command from the last timestep, generated externally by the controller
    thrustAccumulator += control->thrust * CONTROL_TO_ACC; // thrust is in grams, we need ms^-2
    thrustAccumulatorCount++;

    // Run the system dynamics to predict the state forward.
    if ((osTick-lastPrediction) >= configTICK_RATE_HZ/PREDICT_RATE // update at the PREDICT_RATE
        && gyroAccumulatorCount > 0
        && accAccumulatorCount > 0
        && thrustAccumulatorCount > 0)
    {
        gyroAccumulator.x /= gyroAccumulatorCount;
        gyroAccumulator.y /= gyroAccumulatorCount;
        gyroAccumulator.z /= gyroAccumulatorCount;

        accAccumulator.x /= accAccumulatorCount;
        accAccumulator.y /= accAccumulatorCount;
        accAccumulator.z /= accAccumulatorCount;

        thrustAccumulator /= thrustAccumulatorCount;

        float dt = (float)(osTick-lastPrediction)/configTICK_RATE_HZ;
        stateEstimatorPredict(thrustAccumulator, &accAccumulator, &gyroAccumulator, dt); // prediction

        if (!quadIsFlying) { // accelerometers give us information about attitude on slanted ground
            stateEstimatorUpdateWithAccOnGround(&accAccumulator);
        }

        lastPrediction = osTick;

        accAccumulator = (Axis3f){.axis={0}};
        accAccumulatorCount = 0;
        gyroAccumulator = (Axis3f){.axis={0}};
        gyroAccumulatorCount = 0;
        thrustAccumulator = 0;
        thrustAccumulatorCount = 0;

        doneUpdate = true;
    }


    /**
     * Add process noise every loop, rather than every prediction
     */
    stateEstimatorAddProcessNoise((float)(osTick-lastPNUpdate)/configTICK_RATE_HZ);
    lastPNUpdate = osTick;


    /**
     * Update the state estimate with the barometer measurements
     */
    // Accumulate the barometer measurements
    if (sensorsReadBaro(&sensors->baro)) {
        #ifdef KALMAN_USE_BARO_UPDATE
            
            baroAccumulator.asl += sensors->baro.asl;
            baroAccumulatorCount++;
        }

        if ((osTick-lastBaroUpdate) >= configTICK_RATE_HZ/BARO_RATE // update at BARO_RATE
            && baroAccumulatorCount > 0)
        {
            baroAccumulator.asl /= baroAccumulatorCount;

            stateEstimatorUpdateWithBaro(&sensors->baro);

            baroAccumulator.asl = 0;
            baroAccumulatorCount = 0;
            lastBaroUpdate = osTick;
            doneUpdate = true;
        #endif
    }

    /**
     * Sensor measurements can come in sporadically and faster than the stabilizer loop frequency,
     * we therefore consume all measurements since the last loop, rather than accumulating
     */

    tofMeasurement_t tof;
    while (stateEstimatorHasTOFPacket(&tof))
    {
        stateEstimatorUpdateWithTof(&tof);
        doneUpdate = true;
    }

    heightMeasurement_t height;
    while (stateEstimatorHasHeightPacket(&height))
    {
        stateEstimatorUpdateWithAbsoluteHeight(&height);
        doneUpdate = true;
    }

    // [Change]
    float dt_uwb = (float)(osTick-lastPrediction)/configTICK_RATE_HZ;     
    distanceMeasurement_t dist;
    while (stateEstimatorHasDistanceMeasurement(&dist)){
        stateEstimatorUpdateWithDistance(&dist,dt_uwb);
        doneUpdate = true;
    }

    positionMeasurement_t pos;
    while (stateEstimatorHasPositionMeasurement(&pos))
    {
        if (first_vicon){
        resetEstimation = true;
        first_vicon = false;

        }else{
        stateEstimatorUpdateWithPosition(&pos);
        doneUpdate = true;
        }

    }

    posvelMeasurement_t posvel;
    while(stateEstimatorHasPosVelMeasurement(&posvel)){
        stateEstimatorUpdateWithPosVel(&posvel,sensors);
        doneUpdate = true;
    }

    // [CHANGE] yaw estimation
    posvelyawMeasurement_t posvelyaw;
    while(stateEstimatorHasPosVelYawMeasurement(&posvelyaw)){
        stateEstimatorUpdateWithPosVelYaw(&posvelyaw,sensors);
        doneUpdate = true;
    }
    // [CHANGE] robust ekf for uwb tdoa measurements
    tdoaMeasurement_t tdoa;

    while (stateEstimatorHasTDOAPacket(&tdoa))
    {
        // [Change] Select EKF tdoa update method: EKF + Chi-square or robust EKF with M-estimation  
        if (ROBUST){   
            robustEstimatorUpdateWithTDOA(&tdoa);
            }
        else{
            stateEstimatorUpdateWithTDOA(&tdoa, dt_uwb);
        }
        doneUpdate = true;
    }

    flowMeasurement_t flow;
    while (stateEstimatorHasFlowPacket(&flow))
    {
        stateEstimatorUpdateWithFlow(&flow, sensors);
        doneUpdate = true;
    }

    /**
     * If an update has been made, the state is finalized:
     * - the attitude error is moved into the body attitude quaternion,
     * - the body attitude is converted into a rotation matrix for the next prediction, and
     * - correctness of the covariance matrix is ensured
     */

    if (doneUpdate)
    {
        stateEstimatorFinalize(sensors, osTick);
        stateEstimatorAssertNotNaN();
    }
    /**
     * Finally, the internal state is externalized.
     * This is done every round, since the external state includes some sensor data
     */
    stateEstimatorExternalizeState(state, sensors, osTick);
    stateEstimatorAssertNotNaN();
}

static void stateEstimatorPredict(float cmdThrust, Axis3f *acc, Axis3f *gyro, float dt)
{
  /* Here we discretize (euler forward) and linearise the quadrocopter dynamics in order
   * to push the covariance forward.
   *
   * QUADROCOPTER DYNAMICS (see paper):
   *
   * \dot{x} = R(I + [[d]])p
   * \dot{p} = f/m * e3 - [[\omega]]p - g(I - [[d]])R^-1 e3 //drag negligible
   * \dot{d} = \omega
   *
   * where [[.]] is the cross-product matrix of .
   *       \omega are the gyro measurements
   *       e3 is the column vector [0 0 1]'
   *       I is the identity
   *       R is the current attitude as a rotation matrix
   *       f/m is the mass-normalized motor force (acceleration in the body's z direction)
   *       g is gravity
   *       x, p, d, skew are the quad's states
   * note that d (attitude error) is zero at the beginning of each iteration,
   * since error information is incorporated into R after each Kalman update.
   */

  // The linearized update matrix
  static float A[STATE_DIM][STATE_DIM];
  static arm_matrix_instance_f32 Am = { STATE_DIM, STATE_DIM, (float *)A}; // linearized dynamics for covariance update;

  // Temporary matrices for the covariance updates
  static float tmpNN1d[STATE_DIM * STATE_DIM];
  static arm_matrix_instance_f32 tmpNN1m = { STATE_DIM, STATE_DIM, tmpNN1d};

  static float tmpNN2d[STATE_DIM * STATE_DIM];
  static arm_matrix_instance_f32 tmpNN2m = { STATE_DIM, STATE_DIM, tmpNN2d};

  float dt2 = dt*dt;

  // ====== DYNAMICS LINEARIZATION ======
  // Initialize as the identity
  A[STATE_X][STATE_X] = 1;
  A[STATE_Y][STATE_Y] = 1;
  A[STATE_Z][STATE_Z] = 1;

  A[STATE_PX][STATE_PX] = 1;
  A[STATE_PY][STATE_PY] = 1;
  A[STATE_PZ][STATE_PZ] = 1;

  A[STATE_D0][STATE_D0] = 1;
  A[STATE_D1][STATE_D1] = 1;
  A[STATE_D2][STATE_D2] = 1;

  // position from body-frame velocity
  A[STATE_X][STATE_PX] = R[0][0]*dt;
  A[STATE_Y][STATE_PX] = R[1][0]*dt;
  A[STATE_Z][STATE_PX] = R[2][0]*dt;

  A[STATE_X][STATE_PY] = R[0][1]*dt;
  A[STATE_Y][STATE_PY] = R[1][1]*dt;
  A[STATE_Z][STATE_PY] = R[2][1]*dt;

  A[STATE_X][STATE_PZ] = R[0][2]*dt;
  A[STATE_Y][STATE_PZ] = R[1][2]*dt;
  A[STATE_Z][STATE_PZ] = R[2][2]*dt;

  // position from attitude error
  A[STATE_X][STATE_D0] = (S[STATE_PY]*R[0][2] - S[STATE_PZ]*R[0][1])*dt;
  A[STATE_Y][STATE_D0] = (S[STATE_PY]*R[1][2] - S[STATE_PZ]*R[1][1])*dt;
  A[STATE_Z][STATE_D0] = (S[STATE_PY]*R[2][2] - S[STATE_PZ]*R[2][1])*dt;

  A[STATE_X][STATE_D1] = (- S[STATE_PX]*R[0][2] + S[STATE_PZ]*R[0][0])*dt;
  A[STATE_Y][STATE_D1] = (- S[STATE_PX]*R[1][2] + S[STATE_PZ]*R[1][0])*dt;
  A[STATE_Z][STATE_D1] = (- S[STATE_PX]*R[2][2] + S[STATE_PZ]*R[2][0])*dt;

  A[STATE_X][STATE_D2] = (S[STATE_PX]*R[0][1] - S[STATE_PY]*R[0][0])*dt;
  A[STATE_Y][STATE_D2] = (S[STATE_PX]*R[1][1] - S[STATE_PY]*R[1][0])*dt;
  A[STATE_Z][STATE_D2] = (S[STATE_PX]*R[2][1] - S[STATE_PY]*R[2][0])*dt;

  // body-frame velocity from body-frame velocity
  A[STATE_PX][STATE_PX] = 1 - 2*dragXY*S[STATE_PX]*dt; //drag term for new controller
  A[STATE_PY][STATE_PX] =-gyro->z*dt;
  A[STATE_PZ][STATE_PX] = gyro->y*dt;

  A[STATE_PX][STATE_PY] = gyro->z*dt;
  A[STATE_PY][STATE_PY] = 1 - 2*dragXY*S[STATE_PY]*dt; //drag term for new controller
  A[STATE_PZ][STATE_PY] =-gyro->x*dt;

  A[STATE_PX][STATE_PZ] =-gyro->y*dt;
  A[STATE_PY][STATE_PZ] = gyro->x*dt;
  A[STATE_PZ][STATE_PZ] = 1 - 2*dragZ*S[STATE_PZ]*dt; //drag term for new controller

  // body-frame velocity from attitude error
  A[STATE_PX][STATE_D0] =  0;
  A[STATE_PY][STATE_D0] = -GRAVITY_MAGNITUDE*R[2][2]*dt;
  A[STATE_PZ][STATE_D0] =  GRAVITY_MAGNITUDE*R[2][1]*dt;

  A[STATE_PX][STATE_D1] =  GRAVITY_MAGNITUDE*R[2][2]*dt;
  A[STATE_PY][STATE_D1] =  0;
  A[STATE_PZ][STATE_D1] = -GRAVITY_MAGNITUDE*R[2][0]*dt;

  A[STATE_PX][STATE_D2] = -GRAVITY_MAGNITUDE*R[2][1]*dt;
  A[STATE_PY][STATE_D2] =  GRAVITY_MAGNITUDE*R[2][0]*dt;
  A[STATE_PZ][STATE_D2] =  0;

  // attitude error from attitude error
  /**
   * At first glance, it may not be clear where the next values come from, since they do not appear directly in the
   * dynamics. In this prediction step, we skip the step of first updating attitude-error, and then incorporating the
   * new error into the current attitude (which requires a rotation of the attitude-error covariance). Instead, we
   * directly update the body attitude, however still need to rotate the covariance, which is what you see below.
   *
   * This comes from a second order approximation to:
   * Sigma_post = exps(-d) Sigma_pre exps(-d)'
   *            ~ (I + [[-d]] + [[-d]]^2 / 2) Sigma_pre (I + [[-d]] + [[-d]]^2 / 2)'
   * where d is the attitude error expressed as Rodriges parameters, ie. d0 = 1/2*gyro.x*dt under the assumption that
   * d = [0,0,0] at the beginning of each prediction step and that gyro.x is constant over the sampling period
   *
   * As derived in "Covariance Correction Step for Kalman Filtering with an Attitude"
   * http://arc.aiaa.org/doi/abs/10.2514/1.G000848
   */
  float d0 = gyro->x*dt/2;
  float d1 = gyro->y*dt/2;
  float d2 = gyro->z*dt/2;

  A[STATE_D0][STATE_D0] =  1 - d1*d1/2 - d2*d2/2;
  A[STATE_D0][STATE_D1] =  d2 + d0*d1/2;
  A[STATE_D0][STATE_D2] = -d1 + d0*d2/2;

  A[STATE_D1][STATE_D0] = -d2 + d0*d1/2;
  A[STATE_D1][STATE_D1] =  1 - d0*d0/2 - d2*d2/2;
  A[STATE_D1][STATE_D2] =  d0 + d1*d2/2;

  A[STATE_D2][STATE_D0] =  d1 + d0*d2/2;
  A[STATE_D2][STATE_D1] = -d0 + d1*d2/2;
  A[STATE_D2][STATE_D2] = 1 - d0*d0/2 - d1*d1/2;


  // ====== COVARIANCE UPDATE ======
  mat_mult(&Am, &Pm, &tmpNN1m); // A P
  mat_trans(&Am, &tmpNN2m); // A'
  mat_mult(&tmpNN1m, &tmpNN2m, &Pm); // A P A'
  // Process noise is added after the return from the prediction step

  // ====== PREDICTION STEP ======
  // The prediction depends on whether we're on the ground, or in flight.
  // When flying, the accelerometer directly measures thrust (hence is useless to estimate body angle while flying)

  // TODO: Find a better check for whether the quad is flying
  // Assume that the flight begins when the thrust is large enough and for now we never stop "flying".
  if (cmdThrust > IN_FLIGHT_THRUST_THRESHOLD) {
    lastFlightCmd = xTaskGetTickCount();
    if (!quadIsFlying) {
      takeoffTime = lastFlightCmd;
    }
  }
  quadIsFlying = (xTaskGetTickCount()-lastFlightCmd) < IN_FLIGHT_TIME_THRESHOLD;

  float dx, dy, dz;
  float tmpSPX, tmpSPY, tmpSPZ;
  float zacc;

  if (quadIsFlying) // only acceleration in z direction
  {
    // TODO: In the next lines, can either use cmdThrust/mass, or acc->z. Need to test which is more reliable.
    // cmdThrust's error comes from poorly calibrated mass, and inexact cmdThrust -> thrust map
    // acc->z's error comes from measurement noise and accelerometer scaling
    // float zacc = cmdThrust;
    zacc = acc->z;

    // position updates in the body frame (will be rotated to inertial frame)
    dx = S[STATE_PX] * dt;
    dy = S[STATE_PY] * dt;
    dz = S[STATE_PZ] * dt + zacc * dt2 / 2.0f; // thrust can only be produced in the body's Z direction

    // position update
    S[STATE_X] += R[0][0] * dx + R[0][1] * dy + R[0][2] * dz;
    S[STATE_Y] += R[1][0] * dx + R[1][1] * dy + R[1][2] * dz;
    S[STATE_Z] += R[2][0] * dx + R[2][1] * dy + R[2][2] * dz - GRAVITY_MAGNITUDE * dt2 / 2.0f;

    // keep previous time step's state for the update
    tmpSPX = S[STATE_PX];
    tmpSPY = S[STATE_PY];
    tmpSPZ = S[STATE_PZ];

    // body-velocity update: accelerometers - gyros cross velocity - gravity in body frame
     S[STATE_PX] += dt * (gyro->z * tmpSPY - gyro->y * tmpSPZ - GRAVITY_MAGNITUDE * R[2][0]);
     S[STATE_PY] += dt * (-gyro->z * tmpSPX + gyro->x * tmpSPZ - GRAVITY_MAGNITUDE * R[2][1]);
     S[STATE_PZ] += dt * (zacc + gyro->y * tmpSPX - gyro->x * tmpSPY - GRAVITY_MAGNITUDE * R[2][2]);

    //drag term for new controller
    //    S[STATE_PX] += dt * (gyro->z * tmpSPY - gyro->y * tmpSPZ - GRAVITY_MAGNITUDE * R[2][0]) + dragXY*tmpSPX*tmpSPX*dt*(tmpSPX<0?1:-1);
    //    S[STATE_PY] += dt * (-gyro->z * tmpSPX + gyro->x * tmpSPZ - GRAVITY_MAGNITUDE * R[2][1]) + dragXY*tmpSPY*tmpSPY*dt*(tmpSPY<0?1:-1);
    //    S[STATE_PZ] += dt * (zacc + gyro->y * tmpSPX - gyro->x * tmpSPY - GRAVITY_MAGNITUDE * R[2][2]) + dragZ*tmpSPZ*tmpSPZ*dt*(tmpSPZ<0?1:-1);
  }
  else // Acceleration can be in any direction, as measured by the accelerometer. This occurs, eg. in freefall or while being carried.
  {
    // position updates in the body frame (will be rotated to inertial frame)
    dx = S[STATE_PX] * dt + acc->x * dt2 / 2.0f;
    dy = S[STATE_PY] * dt + acc->y * dt2 / 2.0f;
    dz = S[STATE_PZ] * dt + acc->z * dt2 / 2.0f; // thrust can only be produced in the body's Z direction

    // position update
    S[STATE_X] += R[0][0] * dx + R[0][1] * dy + R[0][2] * dz;
    S[STATE_Y] += R[1][0] * dx + R[1][1] * dy + R[1][2] * dz;
    S[STATE_Z] += R[2][0] * dx + R[2][1] * dy + R[2][2] * dz - GRAVITY_MAGNITUDE * dt2 / 2.0f;

    // keep previous time step's state for the update
    tmpSPX = S[STATE_PX];
    tmpSPY = S[STATE_PY];
    tmpSPZ = S[STATE_PZ];

    // body-velocity update: accelerometers - gyros cross velocity - gravity in body frame
    S[STATE_PX] += dt * (acc->x + gyro->z * tmpSPY - gyro->y * tmpSPZ - GRAVITY_MAGNITUDE * R[2][0]);
    S[STATE_PY] += dt * (acc->y - gyro->z * tmpSPX + gyro->x * tmpSPZ - GRAVITY_MAGNITUDE * R[2][1]);
    S[STATE_PZ] += dt * (acc->z + gyro->y * tmpSPX - gyro->x * tmpSPY - GRAVITY_MAGNITUDE * R[2][2]);
  }

  // attitude update (rotate by gyroscope), we do this in quaternions
  // this is the gyroscope angular velocity integrated over the sample period
  float dtwx = dt*gyro->x;
  float dtwy = dt*gyro->y;
  float dtwz = dt*gyro->z;

  // compute the quaternion values in [w,x,y,z] order
  float angle = arm_sqrt(dtwx*dtwx + dtwy*dtwy + dtwz*dtwz);
  float ca = arm_cos_f32(angle/2.0f);
  float sa = arm_sin_f32(angle/2.0f);
  float dq[4] = {ca , sa*dtwx/angle , sa*dtwy/angle , sa*dtwz/angle};

  float tmpq0;
  float tmpq1;
  float tmpq2;
  float tmpq3;

  if (quadIsFlying) {
    // rotate the quad's attitude by the delta quaternion vector computed above
    tmpq0 = (dq[0]*q[0] - dq[1]*q[1] - dq[2]*q[2] - dq[3]*q[3]);
    tmpq1 = (1.0f)*(dq[1]*q[0] + dq[0]*q[1] + dq[3]*q[2] - dq[2]*q[3]);
    tmpq2 = (1.0f)*(dq[2]*q[0] - dq[3]*q[1] + dq[0]*q[2] + dq[1]*q[3]);
    tmpq3 = (1.0f)*(dq[3]*q[0] + dq[2]*q[1] - dq[1]*q[2] + dq[0]*q[3]);
  } else {
    // rotate the quad's attitude by the delta quaternion vector computed above
    tmpq0 = (dq[0]*q[0] - dq[1]*q[1] - dq[2]*q[2] - dq[3]*q[3]);
    tmpq1 = (1.0f-ROLLPITCH_ZERO_REVERSION)*(dq[1]*q[0] + dq[0]*q[1] + dq[3]*q[2] - dq[2]*q[3]);
    tmpq2 = (1.0f-ROLLPITCH_ZERO_REVERSION)*(dq[2]*q[0] - dq[3]*q[1] + dq[0]*q[2] + dq[1]*q[3]);
    tmpq3 = (1.0f-ROLLPITCH_ZERO_REVERSION)*(dq[3]*q[0] + dq[2]*q[1] - dq[1]*q[2] + dq[0]*q[3]);
  }


  // normalize and store the result
  float norm = arm_sqrt(tmpq0*tmpq0 + tmpq1*tmpq1 + tmpq2*tmpq2 + tmpq3*tmpq3);
  q[0] = tmpq0/norm; q[1] = tmpq1/norm; q[2] = tmpq2/norm; q[3] = tmpq3/norm;
  stateEstimatorAssertNotNaN();
}

static void stateEstimatorAddProcessNoise(float dt)
{
  if (dt>0)
  {
    P[STATE_X][STATE_X] += powf(procNoiseAcc_xy*dt*dt + procNoiseVel*dt + procNoisePos, 2);  // add process noise on position
    P[STATE_Y][STATE_Y] += powf(procNoiseAcc_xy*dt*dt + procNoiseVel*dt + procNoisePos, 2);  // add process noise on position
    P[STATE_Z][STATE_Z] += powf(procNoiseAcc_z*dt*dt + procNoiseVel*dt + procNoisePos, 2);  // add process noise on position

    P[STATE_PX][STATE_PX] += powf(procNoiseAcc_xy*dt + procNoiseVel, 2); // add process noise on velocity
    P[STATE_PY][STATE_PY] += powf(procNoiseAcc_xy*dt + procNoiseVel, 2); // add process noise on velocity
    P[STATE_PZ][STATE_PZ] += powf(procNoiseAcc_z*dt + procNoiseVel, 2); // add process noise on velocity

    P[STATE_D0][STATE_D0] += powf(measNoiseGyro_rollpitch * dt + procNoiseAtt, 2);
    P[STATE_D1][STATE_D1] += powf(measNoiseGyro_rollpitch * dt + procNoiseAtt, 2);
    P[STATE_D2][STATE_D2] += powf(measNoiseGyro_yaw * dt + procNoiseAtt, 2);
  }

  for (int i=0; i<STATE_DIM; i++) {
    for (int j=i; j<STATE_DIM; j++) {
      float p = 0.5f*P[i][j] + 0.5f*P[j][i];
      if (isnan(p) || p > MAX_COVARIANCE) {
        P[i][j] = P[j][i] = MAX_COVARIANCE;
      } else if ( i==j && p < MIN_COVARIANCE ) {
        P[i][j] = P[j][i] = MIN_COVARIANCE;
      } else {
        P[i][j] = P[j][i] = p;
      }
    }
  }

  stateEstimatorAssertNotNaN();
}


static void stateEstimatorScalarUpdate(arm_matrix_instance_f32 *Hm, float error, float stdMeasNoise)
{
  // The Kalman gain as a column vector
  static float K[STATE_DIM];
  static arm_matrix_instance_f32 Km = {STATE_DIM, 1, (float *)K};

  // Temporary matrices for the covariance updates
  static float tmpNN1d[STATE_DIM * STATE_DIM];
  static arm_matrix_instance_f32 tmpNN1m = {STATE_DIM, STATE_DIM, tmpNN1d};

  static float tmpNN2d[STATE_DIM * STATE_DIM];
  static arm_matrix_instance_f32 tmpNN2m = {STATE_DIM, STATE_DIM, tmpNN2d};

  static float tmpNN3d[STATE_DIM * STATE_DIM];
  static arm_matrix_instance_f32 tmpNN3m = {STATE_DIM, STATE_DIM, tmpNN3d};

  static float HTd[STATE_DIM * 1];
  static arm_matrix_instance_f32 HTm = {STATE_DIM, 1, HTd};

  static float PHTd[STATE_DIM * 1];
  static arm_matrix_instance_f32 PHTm = {STATE_DIM, 1, PHTd};

  configASSERT(Hm->numRows == 1);
  configASSERT(Hm->numCols == STATE_DIM);

  // ====== INNOVATION COVARIANCE ======
  mat_trans(Hm, &HTm);
  mat_mult(&Pm, &HTm, &PHTm); // PH'
  float R = stdMeasNoise*stdMeasNoise;
  float HPHR = R; // HPH' + R
  for (int i=0; i<STATE_DIM; i++) { // Add the element of HPH' to the above
    HPHR += Hm->pData[i]*PHTd[i]; // this obviously only works if the update is scalar (as in this function)
  }
  configASSERT(!isnan(HPHR));
  // check the Chi-sqaure in original Update function
  float err_abs = (float)fabs(error);
  float err_sqr = error * error;
  float d_m = err_sqr/HPHR;     // Chi-square
  log_dm = d_m;
  log_errAbs = err_abs;

  bool Chi_square_label = true;  bool three_sigma_flag = true;

    // ****************** Chi-squared test *********************//
    if(CHI_SQRUARE && (d_m >= 5.0f)){   // tuning param
        Chi_square_label = false;
    }
    //
    if(THREE_SIGMA && (err_abs > 3.0f*stdMeasNoise)){
        three_sigma_flag = false;
    }

    if(Chi_square_label & three_sigma_flag){
        // ====== MEASUREMENT UPDATE ======
        // Calculate the Kalman gain and perform the state update
        for (int i=0; i<STATE_DIM; i++) {
            K[i] = PHTd[i]/HPHR; // kalman gain = (PH' (HPH' + R )^-1)
            S[i] = S[i] + K[i] * error; // state update
        }
        stateEstimatorAssertNotNaN();

        // ====== COVARIANCE UPDATE ======
        mat_mult(&Km, Hm, &tmpNN1m); // KH
        for (int i=0; i<STATE_DIM; i++) { tmpNN1d[STATE_DIM*i+i] -= 1; } // KH - I
        mat_trans(&tmpNN1m, &tmpNN2m); // (KH - I)'
        mat_mult(&tmpNN1m, &Pm, &tmpNN3m); // (KH - I)*P
        mat_mult(&tmpNN3m, &tmpNN2m, &Pm); // (KH - I)*P*(KH - I)'
        stateEstimatorAssertNotNaN();
        // add the measurement variance and ensure boundedness and symmetry
        // TODO: Why would it hit these bounds? Needs to be investigated.
        for (int i=0; i<STATE_DIM; i++) {
            for (int j=i; j<STATE_DIM; j++) {
            float v = K[i] * R * K[j];
            float p = 0.5f*P[i][j] + 0.5f*P[j][i] + v; // add measurement noise
            if (isnan(p) || p > MAX_COVARIANCE) {
                P[i][j] = P[j][i] = MAX_COVARIANCE;
            } else if ( i==j && p < MIN_COVARIANCE ) {
                P[i][j] = P[j][i] = MIN_COVARIANCE;
            } else {
                P[i][j] = P[j][i] = p;
            }
            }
        }
        stateEstimatorAssertNotNaN();
    }
}

static void stateEstimatorUpdateWithAccOnGround(Axis3f *acc)
{
  // The following code is disabled due to the function not being complete (and that we aim for zero warnings).
    #if 0
    // This update only makes sense on the ground, when no thrust is being produced,
    // since the accelerometers can then directly measure the direction of gravity
    float accMag = sqrtf(acc->x*acc->x + acc->y*acc->y + acc->z*acc->z);

    // Only do the update if the quad isn't flying, and if the accelerometers
    // are close enough to gravity that we can assume it is the only force
    if(!quadIsFlying && fabs(1-accMag/GRAVITY_MAGNITUDE) < 0.01) {
        float h[STATE_DIM] = {0};
        arm_matrix_instance_f32 H = {1, STATE_DIM, h};

        float gravityInBodyX = GRAVITY_MAGNITUDE * R[2][0];
        float gravityInBodyY = GRAVITY_MAGNITUDE * R[2][1];
        float gravityInBodyZ = GRAVITY_MAGNITUDE * R[2][2];

        // TODO: What are the update equations?
    }
    #endif
}

#ifdef KALMAN_USE_BARO_UPDATE
static void stateEstimatorUpdateWithBaro(baro_t *baro)
{
  static float baroReferenceHeight = 0;

  float h[STATE_DIM] = {0};
  arm_matrix_instance_f32 H = {1, STATE_DIM, h};

  h[STATE_Z] = 1;

  if (!quadIsFlying || baroReferenceHeight < 1) {
    //TODO: maybe we could track the zero height as a state. Would be especially useful if UWB anchors had barometers.
    baroReferenceHeight = baro->asl;
  }

  float meas = (baro->asl - baroReferenceHeight);
  stateEstimatorScalarUpdate(&H, meas - S[STATE_Z], measNoiseBaro);
}
#endif

static void stateEstimatorUpdateWithAbsoluteHeight(heightMeasurement_t* height) {
  float h[STATE_DIM] = {0};
  arm_matrix_instance_f32 H = {1, STATE_DIM, h};
  h[STATE_Z] = 1;
  stateEstimatorScalarUpdate(&H, height->height - S[STATE_Z], height->stdDev);
}

static void stateEstimatorUpdateWithPosition(positionMeasurement_t *xyz)
{
  // a direct measurement of states x, y, and z
  // do a scalar update for each state, since this should be faster than updating all together
  for (int i=0; i<3; i++) {
    float h[STATE_DIM] = {0};
    arm_matrix_instance_f32 H = {1, STATE_DIM, h};
    h[STATE_X+i] = 1;
    stateEstimatorScalarUpdate(&H, xyz->pos[i] - S[STATE_X+i], xyz->stdDev);
  }
}

static void stateEstimatorUpdateWithPosVel(posvelMeasurement_t *posvel, sensorData_t *sensors){
	// a direct measurement of states x, y, and z
	  // do a scalar update for each state, since this should be faster than updating all together
	  for (int i=0; i<3; i++) {
	    float h[STATE_DIM] = {0};
	    arm_matrix_instance_f32 H = {1, STATE_DIM, h};
	    h[STATE_X+i] = 1;             //  what's the function for this line??
	    stateEstimatorScalarUpdate(&H, posvel->pos[i] - S[STATE_X+i], posvel->stdDev_pos);
	  }
	  // Measurement model of velocity as measured in world frame
	  // v_w = R(P + omega^x x)

	  for (int i=0; i<3; i++) {
	    float h[STATE_DIM] = {0};
	    arm_matrix_instance_f32 H = {1, STATE_DIM, h};

	    h[STATE_PX] = R[i][0];
	    h[STATE_PY] = R[i][1];
	    h[STATE_PZ] = R[i][2];
	    float pred_vel_w = R[i][0] * S[STATE_PX] + R[i][1] * S[STATE_PY] + R[i][2] * S[STATE_PZ];

	    stateEstimatorScalarUpdate(&H, posvel->vel[i] - pred_vel_w, posvel->stdDev_vel);
	  }
}

// [CHANGE] yaw estimation
static void stateEstimatorUpdateWithPosVelYaw(posvelyawMeasurement_t *posvelyaw, sensorData_t *sensors){
	  // a direct measurement of states x, y, and z
	  // do a scalar update for each state, since this should be faster than updating all together
	  for (int i=0; i<3; i++) {
	    float h[STATE_DIM] = {0};
	    arm_matrix_instance_f32 H = {1, STATE_DIM, h};
	    h[STATE_X+i] = 1;
	    stateEstimatorScalarUpdate(&H, posvelyaw->pos[i] - S[STATE_X+i], posvelyaw->stdDev_pos);
	  }

	  // Measurement model of velocity as measured in world frame
	  // v_w = R(P + omega^x x)
	  for (int i=0; i<3; i++) {
	    float h[STATE_DIM] = {0};
	    arm_matrix_instance_f32 H = {1, STATE_DIM, h};

	    h[STATE_PX] = R[i][0];
	    h[STATE_PY] = R[i][1];
	    h[STATE_PZ] = R[i][2];
	    float pred_vel_w = R[i][0] * S[STATE_PX] + R[i][1] * S[STATE_PY] + R[i][2] * S[STATE_PZ];

	    stateEstimatorScalarUpdate(&H, posvelyaw->vel[i] - pred_vel_w, posvelyaw->stdDev_vel);
	  }

	  // direct measurement of yaw (yaw error STATE_D2)
	  float h[STATE_DIM] = {0};
	  arm_matrix_instance_f32 H = {1, STATE_DIM, h};
	  h[STATE_D2] = 1; // the Jacobian
	  float pred_yaw = atan2f(2*(q[1]*q[2]+q[0]*q[3]) , q[0]*q[0] + q[1]*q[1] - q[2]*q[2] - q[3]*q[3]);
	  float yaw_error = posvelyaw->yaw - pred_yaw;
	  yaw_error_logback = yaw_error;

	  // wrap yaw_error between (-PI, PI]
	  while (yaw_error > PI){
		  yaw_error -= (float) 2.0 * PI;
	  }

	  while (yaw_error <= -PI){
		  yaw_error += (float) 2.0 * PI;
	  }

	  // Add yaw measurement to Kalman filter if yaw estimate is valid (i.e., in (-PI, PI])
	  if ((posvelyaw->yaw > -PI) && (posvelyaw->yaw <= PI)){
		  stateEstimatorScalarUpdate(&H, yaw_error, posvelyaw->stdDev_yaw);
	  }
}

//TWR
// implement TWR-trilateration before fusing into EKF
static void stateEstimatorUpdateWithDistance(distanceMeasurement_t *d, float dt)
{
	  // a measurement of distance to point (x, y, z)
	  float h[STATE_DIM] = {0};
	  arm_matrix_instance_f32 H = {1, STATE_DIM, h};
	  // d->x,y,z is the anchor's position
	  float dx = S[STATE_X] - d->x;
	  float dy = S[STATE_Y] - d->y;
	  float dz = S[STATE_Z] - d->z;
	  float measuredDistance = 0.0f;

	  measuredDistance = d->distance;

	  //do not fuse "0" measurement
	  if (measuredDistance >= 0.001f)
	  {
		  float predictedDistance = arm_sqrt(powf(dx, 2) + powf(dy, 2) + powf(dz, 2));
		  if (predictedDistance != 0.0f)
		  {
		  // The measurement is: z = sqrt(dx^2 + dy^2 + dz^2). The derivative dz/dX gives h.
			  h[STATE_X] = dx/predictedDistance;
			  h[STATE_Y] = dy/predictedDistance;
			  h[STATE_Z] = dz/predictedDistance;
		  }
		  else
		  {
		  // Avoid divide by zero
			  h[STATE_X] = 1.0f;
			  h[STATE_Y] = 0.0f;
			  h[STATE_Z] = 0.0f;
		  }
		  // Extra logging variables
		  twrDist = d->distance;
		  anchorID = d->anchor_ID;
		  if (OUTLIER_REJ){
			  float vx = S[STATE_PX];
			  float vy = S[STATE_PY];
			  float vz = S[STATE_PZ];
			  float Vpr = arm_sqrt(powf(vx, 2) + powf(vy, 2) + powf(vz, 2));    // prior velocity
			  float T_max = 40.0;
			  float F_max[3][1] ={{0.0},{0.0},{(float)4.0*T_max* GRAVITY_MAGNITUDE}};
			  float g_body[3][1] = {{R[2][0]*GRAVITY_MAGNITUDE},{R[2][1]*GRAVITY_MAGNITUDE},{R[2][2]*GRAVITY_MAGNITUDE}};  // R^T [0;0;g]
			  float ACC_max[3][1] = {{F_max[0][0]-g_body[0][0]},{F_max[1][0]-g_body[1][0]},{F_max[2][0]-g_body[2][0]}};    //F_max - R^T [0;0;g]
			  float a_max = arm_sqrt(powf(ACC_max[0][0], 2) + powf(ACC_max[1][0], 2) + powf(ACC_max[2][0], 2));
			  float r_max = Vpr * dt + (float)0.5*a_max*dt*dt;
			  if((enable_UWB) && (fabsf(measuredDistance-predictedDistance) <= r_max)){
		  	      stateEstimatorScalarUpdate(&H, measuredDistance-predictedDistance, d->stdDev);
			  }
		  }else{
			  if(enable_UWB){
				  stateEstimatorScalarUpdate(&H, measuredDistance-predictedDistance, d->stdDev);
			  }
		  }
	  }
}

//TDoA
static void stateEstimatorUpdateWithTDOA(tdoaMeasurement_t *tdoa, float dt)
{
  if (tdoaCount >= 100)
  {
    /**
     * Measurement equation:
     * dR = dT + d1 - d0
     */
	float measurement=0.0f;

    measurement = tdoa->distanceDiff;
    // predict based on current state
    float x = S[STATE_X];
    float y = S[STATE_Y];
    float z = S[STATE_Z];

    float x1 = tdoa->anchorPosition[1].x, y1 = tdoa->anchorPosition[1].y, z1 = tdoa->anchorPosition[1].z;
    float x0 = tdoa->anchorPosition[0].x, y0 = tdoa->anchorPosition[0].y, z0 = tdoa->anchorPosition[0].z;

    float dx1 = x - x1;   float dy1 = y - y1;   float dz1 = z - z1;
    float dx0 = x - x0;   float dy0 = y - y0;   float dz0 = z - z0;

    float d1 = sqrtf(powf(dx1, 2) + powf(dy1, 2) + powf(dz1, 2));
    float d0 = sqrtf(powf(dx0, 2) + powf(dy0, 2) + powf(dz0, 2));

    float predicted = d1 - d0;
    // DNN bias compensation
    if(DNN_COM){
            float v_an0[3] = { dx0,  dy0,  dz0};    float v_an1[3] = { dx1,  dy1,  dz1};
            float v_cf0[3] = {-dx0, -dy0, -dz0};    float v_cf1[3] = {-dx1, -dy1, -dz1};
            // AzEl[8] = {cf_Az0, cf_Ele0, cf_Az1, cf_Ele1, An_Az0, An_Ele0, An_Az1, An_Ele1}
            float AzEl[8]= {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
            // index the anchor pose based on measurement ID
            int ID[8] ={0,1,2,3,4,5,6,7};
            float q_IA0[4] = {0}; float q_IA1[4] ={0};

            if (tdoa->anchor_id == 0){
                vectorcopy(4, q_IA0, q_an[7]);
                vectorcopy(4, q_IA1, q_an[0]);
            }else{
                vectorcopy(4, q_IA0, q_an[ID[tdoa->anchor_id - 1]]);
                vectorcopy(4, q_IA1, q_an[ID[tdoa->anchor_id]]);
            }
            // get the Azimuth and Elevation angles
            getAzEl_Angle(v_cf0, v_cf1, v_an0, v_an1, R, q_IA0, q_IA1, AzEl);
            // feature vector
            float feature_tdoa[14] = { dx0,   dy0,   dz0,  dx1,   dy1,   dz1,
                                       AzEl[0],  AzEl[1],  AzEl[2],  AzEl[3],
                                       AzEl[4],  AzEl[5],  AzEl[6],  AzEl[7] };

            // debug testing -- feature vector {3.0,    3.0,   1.0,    2.0,    3.0,    1.0, 
            //                                  100.0,  30.0,  100.0,  40.0,
            //                                  150.0,  28.0,  130.0,  48.0 }
            // feature_tdoa[0] = 23.0;    feature_tdoa[1] = 3.0; feature_tdoa[2] = 13.0;   feature_tdoa[3] = 2.0;
            // feature_tdoa[4] = 7.0;    feature_tdoa[5] = 1.0; feature_tdoa[6] = 120.0; feature_tdoa[7] = 30.0;
            // feature_tdoa[8] = 100.0;  feature_tdoa[9] = 45.0; feature_tdoa[10] = 150.0; feature_tdoa[11] = 28.0;
            // feature_tdoa[12] = 30.0; feature_tdoa[13] = 48.0;
            // ------------------ get feature normalization range --------------------- //
            float uwb_feature_max_tdoa[14]={0};   float uwb_feature_min_tdoa[14] ={0};
            float uwb_err_max_tdoa = 0;           float uwb_err_min_tdoa = 0;
            getErrMax(&uwb_err_max_tdoa);          getErrMin(&uwb_err_min_tdoa);
            getFeatureMax(uwb_feature_max_tdoa);  getFeatureMin(uwb_feature_min_tdoa);
            // ----------------------------------------------------------------------- //
            for(int idx=0; idx<14; idx++){
			  feature_tdoa[idx] = scaler_normalize(feature_tdoa[idx], uwb_feature_min_tdoa[idx], uwb_feature_max_tdoa[idx]);
		    }
            // DNN inference
            float bias = nn_inference(feature_tdoa, 14);
            // denormalization
            float Bias = scaler_denormalize(bias, uwb_err_min_tdoa, uwb_err_max_tdoa);
            // debug setting
            Bias = 0.0f;
            // measurements after bias compensation
            measurement = tdoa->distanceDiff + Bias;
    }else{
    	  // without DNN bias compensation
          measurement = tdoa->distanceDiff;
    }

    // innovation term 
    float error = measurement - predicted;

    float h[STATE_DIM] = {0};
    arm_matrix_instance_f32 H = {1, STATE_DIM, h};

    if ((d0 != 0.0f) && (d1 != 0.0f)) {
        h[STATE_X] = (dx1 / d1 - dx0 / d0);
        h[STATE_Y] = (dy1 / d1 - dy0 / d0);
        h[STATE_Z] = (dz1 / d1 - dz0 / d0);

        vector_t jacobian = {
            .x = h[STATE_X],
            .y = h[STATE_Y],
            .z = h[STATE_Z],
        };

        point_t estimatedPosition = {
            .x = S[STATE_X],
            .y = S[STATE_Y],
            .z = S[STATE_Z],
        };
        
        // Outlier rejection labels
        bool sampleIsGood = outlierFilterVaildateTdoaSteps(tdoa, error, &jacobian, &estimatedPosition);
        if(sampleIsGood &&enable_UWB){
            stateEstimatorScalarUpdate(&H, error, tdoa->stdDev);
        }
        // reset flags
        // Model_based_label = true;    Chi_square_label = true; three_sigma_flag = true;
        tdoaID = tdoa->anchor_id;
        tdoaDist = tdoa->distanceDiff;     
    }
  }
  tdoaCount++;
}

// [CHANGE] Robust Extended Kalman Filter
// Cholesky Decomposition for a nxn psd matrix(from scratch)
// A function for a dynamic dim. of matrix can be implement for other cases.
static void Cholesky_Decomposition(int n, float matrix[n][n],  float lower[n][n]){
    // Decomposing a matrix into Lower Triangular 
    for (int i = 0; i < n; i++) { 
        for (int j = 0; j <= i; j++) { 
            float sum = 0.0; 
            if (j == i) // summation for diagnols 
            { 
                for (int k = 0; k < j; k++) 
                    sum += powf(lower[j][k], 2); 
                lower[j][j] = sqrtf(matrix[j][j] - sum); 
            } else { 
                // Evaluating L(i, j) using L(j, j) 
                for (int k = 0; k < j; k++) 
                    sum += (lower[i][k] * lower[j][k]); 
                lower[i][j] = (matrix[i][j] - sum) / lower[j][j]; 
            } 
        } 
    }
} 
// [Help function] copy float matrix
static void matrixcopy(int ROW, int COLUMN, float destmat[ROW][COLUMN], float srcmat[ROW][COLUMN]){
    for (int i=0; i<ROW; i++){
        for(int j=0; j<COLUMN; j++){
            destmat[i][j] = srcmat[i][j];
        }
    }
}
static void vectorcopy(int DIM, float destVec[DIM], float srcVec[DIM]){
    for (int i=0; i<DIM; i++){
        destVec[i] = srcVec[i];
    }
}
// Weight function for GM Robust cost function
static void GM_UWB(float e, float * GM_e){
    float sigma = 2.5;
    float GM_dn = sigma + e*e;
    *GM_e = (sigma * sigma)/(GM_dn * GM_dn);
}

static void GM_state(float e, float * GM_e){
    float sigma = 1.5;
    float GM_dn = sigma + e*e;
    *GM_e = (sigma * sigma)/(GM_dn * GM_dn);
}

static void quat2Rot(float q[4], float R[3][3]){
    // convert quaternion to rotation matrix
    R[0][0] = q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3];
    R[0][1] = 2 * q[1] * q[2] - 2 * q[0] * q[3];
    R[0][2] = 2 * q[1] * q[3] + 2 * q[0] * q[2];

    R[1][0] = 2 * q[1] * q[2] + 2 * q[0] * q[3];
    R[1][1] = q[0] * q[0] - q[1] * q[1] + q[2] * q[2] - q[3] * q[3];
    R[1][2] = 2 * q[2] * q[3] - 2 * q[0] * q[1];

    R[2][0] = 2 * q[1] * q[3] - 2 * q[0] * q[2];
    R[2][1] = 2 * q[2] * q[3] + 2 * q[0] * q[1];
    R[2][2] = q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3];
}

static void RT_v(float v[3], float C[3][3], float v_b[3]){
    // matrix computation: v_b = C.T.dot(v)
    v_b[0] = C[0][0]*v[0] + C[1][0]*v[1] + C[2][0]*v[2];
    v_b[1] = C[0][1]*v[0] + C[1][1]*v[1] + C[2][1]*v[2];
    v_b[2] = C[0][2]*v[0] + C[1][2]*v[1] + C[2][2]*v[2];
}

static void getAzEl_Angle(float v_cf0[3], float v_cf1[3], float v_an0[3], float v_an1[3], float C_IB[3][3], 
                          float q_IA0[4], float q_IA1[4], float AzEl[8]){
    // AzEl[8] = {cf_Az0, cf_Ele0, cf_Az1, cf_Ele1, An_Az0, An_Ele0, An_Az1, An_Ele1}
    float v_cf0_b[3]={0};            float v_cf1_b[3]={0};
    RT_v(v_cf0, C_IB, v_cf0_b);
    AzEl[0] = atan2f(v_cf0_b[1], v_cf0_b[0]) * RAD_TO_DEG;
    AzEl[1] = asinf(v_cf0_b[2]/ (sqrtf(powf(v_cf0_b[0], 2) + powf(v_cf0_b[1], 2) + powf(v_cf0_b[2], 2))) ) * RAD_TO_DEG;

    RT_v(v_cf1, C_IB, v_cf1_b);
    AzEl[2] = atan2f(v_cf1_b[1], v_cf1_b[0]) * RAD_TO_DEG;
    AzEl[3] = asinf(v_cf1_b[2]/ (sqrtf(powf(v_cf1_b[0], 2) + powf(v_cf1_b[1], 2) + powf(v_cf1_b[2], 2))) ) * RAD_TO_DEG;

    float C_IA0[3][3] = {0};        float C_IA1[3][3] = {0};
    quat2Rot(q_IA0, C_IA0);         quat2Rot(q_IA1, C_IA1);
    float v_an0_b[3]={0};           float v_an1_b[3]={0};
    RT_v(v_an0, C_IA0, v_an0_b);
    AzEl[4] = atan2f(v_an0_b[1], v_an0_b[0]) * RAD_TO_DEG;
    AzEl[5] = asinf(v_an0_b[2]/ (sqrtf(powf(v_an0_b[0], 2) + powf(v_an0_b[1], 2) + powf(v_an0_b[2], 2))) )* RAD_TO_DEG;

    RT_v(v_an1, C_IA1, v_an1_b);
    AzEl[6] = atan2f(v_an1_b[1], v_an1_b[0]) * RAD_TO_DEG;
    AzEl[7] = asinf(v_an1_b[2]/ (sqrtf(powf(v_an1_b[0], 2) + powf(v_an1_b[1], 2) + powf(v_an1_b[2], 2))) )* RAD_TO_DEG;
}

// Robust EKF update for TDoA measurements
static void robustEstimatorUpdateWithTDOA(tdoaMeasurement_t *tdoa)
{
    if (tdoaCount >= 100)
  {
    /**
     * Measurement equation:
     * dR = dT + d1 - d0
     */
	float measurement = 0.0f;
    // x prior (x_check)
    float x = S[STATE_X];   float y = S[STATE_Y];   float z = S[STATE_Z];

    float x1 = tdoa->anchorPosition[1].x, y1 = tdoa->anchorPosition[1].y, z1 = tdoa->anchorPosition[1].z;
    float x0 = tdoa->anchorPosition[0].x, y0 = tdoa->anchorPosition[0].y, z0 = tdoa->anchorPosition[0].z;

    float dx1 = x - x1;     float dy1 = y - y1;     float dz1 = z - z1;
    float dx0 = x - x0;     float dy0 = y - y0;     float dz0 = z - z0;

    float d1 = sqrtf(powf(dx1, 2) + powf(dy1, 2) + powf(dz1, 2));
    float d0 = sqrtf(powf(dx0, 2) + powf(dy0, 2) + powf(dz0, 2));
    // if measurements make sense and enable UWB
    if ((d0 != 0.0f) && (d1 != 0.0f) && (enable_UWB)) {
        
        float predicted = d1 - d0;
        // [Add DNN]
        if(DNN_COM){
            // --------------------- DNN features unit test -------------------- //
            // dx0 = 1.5;    dy0 = 0.85;   dz0 = 1.2;
            // dx1 = 0.6;    dy1 = 1.13;   dz1 = 0.5;
            // R[0][0] = 1.0;  R[0][1] = 0.0;  R[0][2] = 0.0;
            // R[1][0] = 0.0;  R[1][1] = 1.0;  R[1][2] = 0.5;
            // R[2][0] =-0.5;  R[2][1] = 0.0;  R[2][2] = 1.0;
            // --------------------- DNN features unit test -------------------- //
            float v_an0[3] = { dx0,  dy0,  dz0};    float v_an1[3] = { dx1,  dy1,  dz1};
            float v_cf0[3] = {-dx0, -dy0, -dz0};    float v_cf1[3] = {-dx1, -dy1, -dz1};
            // AzEl[8] = {cf_Az0, cf_Ele0, cf_Az1, cf_Ele1, An_Az0, An_Ele0, An_Az1, An_Ele1}
            float AzEl[8]= {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
            // index the anchor pose based on measurement ID
            int ID[8] ={0,1,2,3,4,5,6,7};
            float q_IA0[4] = {0}; float q_IA1[4] ={0};
            // Initialize a dummy anchor quaternion struct     
            // anchorPose q_an={
            //     .anchorQuaternion = {{0}}
            //     };   
            // get anchor quaternion
            // getQan(&q_an);
            // ------------ unit test ------------ //
            // tdoa->anchor_id = 7;
            // ----------------------------------- //
            if (tdoa->anchor_id == 0){
                vectorcopy(4, q_IA0, q_an[7]);
                vectorcopy(4, q_IA1, q_an[0]);
            }else{
                vectorcopy(4, q_IA0, q_an[ID[tdoa->anchor_id - 1]]);
                vectorcopy(4, q_IA1, q_an[ID[tdoa->anchor_id]]);
            }
            // get the Azimuth and Elevation angles: AzEl
            getAzEl_Angle(v_cf0, v_cf1, v_an0, v_an1, R, q_IA0, q_IA1, AzEl);
            // feature vector
            float feature_tdoa[14] = { dx0,   dy0,   dz0,  dx1,   dy1,   dz1,
                                       AzEl[0],  AzEl[1],  AzEl[2],  AzEl[3],
                                       AzEl[4],  AzEl[5],  AzEl[6],  AzEl[7] };
            // ---------------------- DNN unit test --> feature vector {...} ---------------------- //
            // feature_tdoa[0] = 3.0;    feature_tdoa[1] = 2.0; feature_tdoa[2] = 1.0;   feature_tdoa[3] = 2.0;
            // feature_tdoa[4] = -2.0;    feature_tdoa[5] = 1.0; feature_tdoa[6] = 120.0; feature_tdoa[7] = 30.0;
            // feature_tdoa[8] = 60.0;  feature_tdoa[9] = 45.0; feature_tdoa[10] = 75.0; feature_tdoa[11] = 28.0;
            // feature_tdoa[12] = 30.0; feature_tdoa[13] = 48.0;
            // ------------------ get feature normalization range --------------------- //
            float uwb_feature_max_tdoa[14]={0};    float uwb_feature_min_tdoa[14] ={0};
            float uwb_err_max_tdoa = 0;            float uwb_err_min_tdoa = 0;
            getErrMax(&uwb_err_max_tdoa);          getErrMin(&uwb_err_min_tdoa);
            getFeatureMax(uwb_feature_max_tdoa);   getFeatureMin(uwb_feature_min_tdoa);
            // ----------------------------------------------------------------------- //
            for(int idx=0; idx<14; idx++){
			  feature_tdoa[idx] = scaler_normalize(feature_tdoa[idx], uwb_feature_min_tdoa[idx], uwb_feature_max_tdoa[idx]);
		    }
            // DNN inference
            float bias = nn_inference(feature_tdoa, 14);
            // denormalization
            float Bias = scaler_denormalize(bias, uwb_err_min_tdoa, uwb_err_max_tdoa);
            log_bias = Bias;
            // debug setting
            // Bias = 0.0f;
            // measurements after bias compensation
            measurement = tdoa->distanceDiff + Bias;
        }else{
            // without DNN bias compensation
            measurement = tdoa->distanceDiff;
        }
        // innovation term based on x_check
        float error_check = measurement - predicted;    // error_check

        // ------------------------ unit test ------------------------ //
        // measurement = 2.45f;
        // error_check = 0.12f;
        // S[0]=1.5f;   S[1]=0.0f; S[2]=0.0f; S[3]=0.0f; S[4]=0.0f; S[5]=0.0f; S[6]=0.0f; S[7]=0.0f; S[8]=0.0f;
        // static float P[9][9]={0};
        // P[0][0] = 1.0f;  P[1][1] = 1.0f;  P[2][2] = 1.0f;
        // P[3][3] = 0.1f;  P[4][4] = 0.1f;  P[5][5] = 0.1f;
        // P[6][6] = 0.1f;  P[7][7] = 0.1f;  P[8][8] = 0.1f;
        // ---------------------- matrix defination ----------------------------- //
            static float P_chol[STATE_DIM][STATE_DIM]; 
            static arm_matrix_instance_f32 Pc_m = {STATE_DIM, STATE_DIM, (float *)P_chol};
            // Pc.T
            static float Pc_tran[STATE_DIM][STATE_DIM];
            static arm_matrix_instance_f32 Pc_tran_m = {STATE_DIM, STATE_DIM, (float *)Pc_tran};

            float h[STATE_DIM] = {0};
            arm_matrix_instance_f32 H = {1, STATE_DIM, h};    // H is a row vector

            static float Kw[STATE_DIM];           // The Kalman gain as a column vector
            static arm_matrix_instance_f32 Kwm = {STATE_DIM, 1, (float *)Kw};

            // float error_x[STATE_DIM]={0}; // is not useful
            // arm_matrix_instance_f32 error_x_mat = {STATE_DIM, 1, error_x};
            
            static float e_x[STATE_DIM];
            static arm_matrix_instance_f32 e_x_m = {STATE_DIM, 1, e_x};
            
            static float Pc_inv[STATE_DIM][STATE_DIM];
            static arm_matrix_instance_f32 Pc_inv_m = {STATE_DIM, STATE_DIM, (float *)Pc_inv};
            
            // rescale matrix
            static float wx_inv[STATE_DIM][STATE_DIM];
            static arm_matrix_instance_f32 wx_invm = {STATE_DIM, STATE_DIM, (float *)wx_inv};
            // tmp matrix for P_chol inverse
            static float tmp1[STATE_DIM][STATE_DIM];
            static arm_matrix_instance_f32 tmp1m = {STATE_DIM, STATE_DIM, (float *)tmp1};

            static float Pc_w_inv[STATE_DIM][STATE_DIM];
            static arm_matrix_instance_f32 Pc_w_invm = {STATE_DIM, STATE_DIM, (float *)Pc_w_inv};

            static float P_w[STATE_DIM][STATE_DIM];
            static arm_matrix_instance_f32 P_w_m = {STATE_DIM, STATE_DIM, (float *)P_w};

            // Temporary matrices for the covariance updates (one way to walk around)
            static float tmpNN1d[STATE_DIM][STATE_DIM];
            static arm_matrix_instance_f32 tmpNN1m = {STATE_DIM, STATE_DIM, (float *)tmpNN1d};

            // static float tmpNN2d[STATE_DIM * STATE_DIM];
            // static arm_matrix_instance_f32 tmpNN2m = {STATE_DIM, STATE_DIM, tmpNN2d};

            // static float tmpNN3d[STATE_DIM * STATE_DIM];
            // static arm_matrix_instance_f32 tmpNN3m = {STATE_DIM, STATE_DIM, tmpNN3d};

            static float HTd[STATE_DIM];
            static arm_matrix_instance_f32 HTm = {STATE_DIM, 1, HTd};

            static float PHTd[STATE_DIM];
            static arm_matrix_instance_f32 PHTm = {STATE_DIM, 1, PHTd};
        // ------------------- Some initialization -----------------------//
        // float xpr[STATE_DIM] = {0.0};                // x prior (error state), set to be zeros 
        static float x_err[STATE_DIM] = {0.0};          // x_err comes from the KF update is the state of error state Kalman filter, set to be zero initially
        static arm_matrix_instance_f32 x_errm = {STATE_DIM, 1, x_err};
        static float X_state[STATE_DIM] = {0.0};
        float P_iter[STATE_DIM][STATE_DIM];
        matrixcopy(STATE_DIM, STATE_DIM, P_iter,P);     // init P_iter as P_prior
        // construct Q
        float Q_iter = tdoa->stdDev * tdoa->stdDev;
        vectorcopy(STATE_DIM, X_state, S);             // copy Xpr to X_State and then update in each iterations

        // --------------------------------- Start iteration --------------------------------- //
        // maximum iteration is 4. Setting iter to 5 leads to a problem of timer.c
        // consider the execution time of DNN, set iter to 3
        // matrix definations are not in the loop
        for (int iter = 0; iter < 2; iter++){
            // apply cholesky decomposition for the prior covariance matrix 
            Cholesky_Decomposition(STATE_DIM, P_iter, P_chol);               // P_chol is a lower triangular matrix
            mat_trans(&Pc_m, &Pc_tran_m);
            // test cholesky --> cholesky is correct !!!!
            //----------------------------------------------//
            // decomposition for measurement covariance (scalar case)
            // only true in scalar case
            float Q_chol = sqrtf(Q_iter);           
            // construct H matrix
            // X_state updates in each iteration
            float x_iter = X_state[STATE_X];   float y_iter = X_state[STATE_Y];   float z_iter = X_state[STATE_Z];

            float x1 = tdoa->anchorPosition[1].x, y1 = tdoa->anchorPosition[1].y, z1 = tdoa->anchorPosition[1].z;
            float x0 = tdoa->anchorPosition[0].x, y0 = tdoa->anchorPosition[0].y, z0 = tdoa->anchorPosition[0].z;     

            float dx1 = x_iter - x1;  float dy1 = y_iter - y1;  float dz1 = z_iter - z1;
            float dx0 = x_iter - x0;  float dy0 = y_iter - y0;  float dz0 = z_iter - z0;
            // ---------------- debug inputs ---------------- //
            // dx1 = x_iter + 3.0f;  dy1 = y_iter - 1.0f;  dz1 = z_iter - 2.0f;
            // dx0 = x_iter - 3.0f;  dy0 = y_iter - 1.0f;  dz0 = z_iter - 2.0f;

            float d1 = sqrtf(powf(dx1, 2) + powf(dy1, 2) + powf(dz1, 2));
            float d0 = sqrtf(powf(dx0, 2) + powf(dy0, 2) + powf(dz0, 2));
            float predicted_iter = d1 - d0;                         // predicted measurements in each iteration based on X_state
            float error_iter = measurement - predicted_iter;        // innovation term based on X_state

            // debug
            // error_iter = 0.1f;
            float e_y = error_iter;
            if ((d0 != 0.0f) && (d1 != 0.0f)){
                h[STATE_X] = (dx1 / d1 - dx0 / d0);  
                h[STATE_Y] = (dy1 / d1 - dy0 / d0); 
                h[STATE_Z] = (dz1 / d1 - dz0 / d0);
                // e_y = linalg.inv(Q_c).dot(err_uwb)
                if (fabsf(Q_chol - 0.0f) < 0.0001f){
                    e_y = error_iter / 0.0001f;
                }
                else{
                    e_y = error_iter / Q_chol;}
                // e_x = inv(Ppr_c) * (error_x), here error_x = x_err
                // Problem: after deon mat_inv, Pc matrix becomes eye(9) !!!
                // Reason: arm_mat_inverse_f32() overwrites the source matrix !!!
                // https://community.arm.com/developer/tools-software/tools/f/keil-forum/32946/cmsis-dsp-matrix-inverse-problem
                matrixcopy(STATE_DIM, STATE_DIM, tmp1, P_chol);
                // in order to keep P_chol
                mat_inv(&tmp1m, &Pc_inv_m);                          // Pc_inv_m = inv(Pc_m) = inv(P_chol)

                mat_mult(&Pc_inv_m, &x_errm, &e_x_m);                  // e_x_m = Pc_inv_m.dot(x_errm) 

                // compute w_x, w_y --> weighting matrix
                // Since w_x is diagnal matrix, directly compute the inverse
                for (int state_k = 0; state_k < STATE_DIM; state_k++){
                    GM_state(e_x[state_k], &wx_inv[state_k][state_k]);
                    wx_inv[state_k][state_k] = (float)1.0 / wx_inv[state_k][state_k];
                }

                // get w_y
                float w_y=0.0;
                GM_UWB(e_y, &w_y);

                // rescale covariance matrix P and Q
                mat_mult(&Pc_m, &wx_invm, &Pc_w_invm);       // Pc_w_invm = P_c.dot(linalg.inv(w_x))

                // rescale P matrix
                mat_mult(&Pc_w_invm, &Pc_tran_m, &P_w_m);        // P_w_m = Pc_w_invm.dot(Pc_tran_m) = P_c.dot(linalg.inv(w_x)).dot(P_c.T)
                // debug
                // matrixcopy(STATE_DIM, STATE_DIM, P_w, P);
                // rescale Q matrix
                float Q_w = 0.0f;
                if (fabsf(w_y - 0.0f) < 0.0001f){
                    Q_w = (Q_chol * Q_chol) / 0.0001f;
                }
                else{
                    Q_w = (Q_chol * Q_chol) / w_y;
                }
                // ====== INNOVATION COVARIANCE ====== //
                // debug
                // Q_w = tdoa->stdDev * tdoa->stdDev;
                // matrixcopy(STATE_DIM, STATE_DIM, P_w, P); 
                // H is a row vector
                mat_trans(&H, &HTm);

                mat_mult(&P_w_m, &HTm, &PHTm);     // PHTm = P_w.dot(H.T). The P is the updated P_w 

                float HPHR = Q_w;                  // HPH' + R.            The Q(R) is the updated Q_w 
                for (int i=0; i<STATE_DIM; i++) {  // Add the element of HPH' to the above
                    HPHR += h[i]*PHTd[i];          // this obviously only works if the update is scalar (as in this function)
                }
                configASSERT(!isnan(HPHR));

                // ====== MEASUREMENT UPDATE ======
                // Calculate the Kalman gain and perform the state update
                for (int i=0; i<STATE_DIM; i++) {
                    Kw[i] = PHTd[i]/HPHR;               // rescaled kalman gain = (PH' (HPH' + R )^-1) with the updated P_w and Q_w
                    //[Note]: The 'error' here is the innovation term based on x_check
                    x_err[i] = Kw[i] * error_check;           // error state for next iteration
                    X_state[i] = S[i] + x_err[i];  // convert to nominal state
                }

                // update P_iter matrix and Q matrix for next iteration
                matrixcopy(STATE_DIM, STATE_DIM, P_iter, P_w);
                Q_iter = Q_w;
                stateEstimatorAssertNotNaN();
            }
        }

        // ---------------------------------- After n iterations --------------------------------------- //
        // P = P_iter =P_w, arm matrix: Pm = P_w_m
        // Q = Q_iter = Q_w

        for (int i=0; i<STATE_DIM; i++){
            S[i] = S[i] + Kw[i] * error_check;
        }
        
        // ====== COVARIANCE UPDATE ======
        mat_mult(&Kwm, &H, &tmpNN1m);               // KH,  the Kalman Gain and H are the updated Kalman Gain and H 
        // ---------- method 1 ---------- //
        //  I-KH
        mat_scale(&tmpNN1m, -1.0f, &tmpNN1m);
        for (int i=0; i<STATE_DIM; i++) { tmpNN1d[i][i] = 1.0f + tmpNN1d[i][i]; } 
        // the last step matrix multiplication does not work! 
        // mat_mult(&tmpNN1m, &P_w_m, &Pm); (tmpNN1m and P_w_m are both correct)
        // ---------- One way to walk around ---------- //
        float Ppo[STATE_DIM][STATE_DIM]={0};
        arm_matrix_instance_f32 Ppom = {STATE_DIM, STATE_DIM, (float *)Ppo};
        mat_mult(&tmpNN1m, &P_w_m, &Ppom);      // Pm = (I-KH)*P_w_m
        matrixcopy(STATE_DIM, STATE_DIM, P, Ppo);
        // -------- method2 ---------//
        // for (int i=0; i<STATE_DIM; i++) { tmpNN1d[STATE_DIM*i+i] -= 1; } // KH - I
        // mat_trans(&tmpNN1m, &tmpNN2m); // (KH - I)'
        // mat_mult(&tmpNN1m, &P_w_m, &tmpNN3m); // (KH - I)*Pw
        
        // float Ppo[STATE_DIM][STATE_DIM]={0};
        // arm_matrix_instance_f32 Ppom = {STATE_DIM, STATE_DIM, (float *)Ppo};
        // mat_mult(&tmpNN3m, &tmpNN2m, &Ppom); // Ppo = (KH - I)*Pw*(KH - I)'
        // matrixcopy(9,9, P, Ppo);
        stateEstimatorAssertNotNaN();
        // add the measurement variance and ensure boundedness and symmetry
        // TODO: Why would it hit these bounds? Needs to be investigated.
        for (int i=0; i<STATE_DIM; i++) {
            for (int j=i; j<STATE_DIM; j++) {
            // float v = Kw[i] * Q_iter * K[j];
            float p = 0.5f*P[i][j] + 0.5f*P[j][i];// + v; // add measurement noise
            if (isnan(p) || p > MAX_COVARIANCE) {
                P[i][j] = P[j][i] = MAX_COVARIANCE;
            } else if ( i==j && p < MIN_COVARIANCE ) {
                P[i][j] = P[j][i] = MIN_COVARIANCE;
            } else {
                P[i][j] = P[j][i] = p;
            }
            }
        }
        stateEstimatorAssertNotNaN();
    } 
  }
  tdoaCount++;
}

// TODO remove the temporary test variables (used for logging)
static float omegax_b;
static float omegay_b;
static float dx_g;
static float dy_g;
static float z_g;
static float predictedNX;
static float predictedNY;
static float measuredNX;
static float measuredNY;

static void stateEstimatorUpdateWithFlow(flowMeasurement_t *flow, sensorData_t *sensors)
{
  // Inclusion of flow measurements in the EKF done by two scalar updates
  if (enable_flow){    // if enable flow when using flowdeck.
  // ~~~ Camera constants ~~~
  // The angle of aperture is guessed from the raw data register and thankfully look to be symmetric
  float Npix = 30.0;                      // [pixels] (same in x and y)
  //float thetapix = DEG_TO_RAD * 4.0f;     // [rad]    (same in x and y)
  float thetapix = DEG_TO_RAD * 4.2f;
  //~~~ Body rates ~~~
  // TODO check if this is feasible or if some filtering has to be done
  omegax_b = sensors->gyro.x * DEG_TO_RAD;
  omegay_b = sensors->gyro.y * DEG_TO_RAD;

  // ~~~ Moves the body velocity into the global coordinate system ~~~
  // [bar{x},bar{y},bar{z}]_G = R*[bar{x},bar{y},bar{z}]_B
  //
  // \dot{x}_G = (R^T*[dot{x}_B,dot{y}_B,dot{z}_B])\dot \hat{x}_G
  // \dot{x}_G = (R^T*[dot{x}_B,dot{y}_B,dot{z}_B])\dot \hat{x}_G
  //
  // where \hat{} denotes a basis vector, \dot{} denotes a derivative and
  // _G and _B refer to the global/body coordinate systems.

  // Modification 1
  //dx_g = R[0][0] * S[STATE_PX] + R[0][1] * S[STATE_PY] + R[0][2] * S[STATE_PZ];
  //dy_g = R[1][0] * S[STATE_PX] + R[1][1] * S[STATE_PY] + R[1][2] * S[STATE_PZ];


  dx_g = S[STATE_PX];
  dy_g = S[STATE_PY];
  // Saturate elevation in prediction and correction to avoid singularities
  if ( S[STATE_Z] < 0.1f ) {
      z_g = 0.1;
  } else {
      z_g = S[STATE_Z];
  }

  // ~~~ X velocity prediction and update ~~~
  // predicts the number of accumulated pixels in the x-direction
  float omegaFactor = 1.25f;
  float hx[STATE_DIM] = {0};
  arm_matrix_instance_f32 Hx = {1, STATE_DIM, hx};
  predictedNX = (flow->dt * Npix / thetapix ) * ((dx_g * R[2][2] / z_g) - omegaFactor * omegay_b);
  measuredNX = flow->dpixelx;

  // derive measurement equation with respect to dx (and z?)
  hx[STATE_Z] = (Npix * flow->dt / thetapix) * ((R[2][2] * dx_g) / (-z_g * z_g));
  hx[STATE_PX] = (Npix * flow->dt / thetapix) * (R[2][2] / z_g);

  // X update
//  if(S[STATE_Z] < UWB_MAX_HEIGHT){
	  stateEstimatorScalarUpdate(&Hx, measuredNX-predictedNX, flow->stdDevX);
//  }
  // ~~~ Y velocity prediction and update ~~~
  float hy[STATE_DIM] = {0};
  arm_matrix_instance_f32 Hy = {1, STATE_DIM, hy};
  predictedNY = (flow->dt * Npix / thetapix ) * ((dy_g * R[2][2] / z_g) + omegaFactor * omegax_b);
  measuredNY = flow->dpixely;

  // derive measurement equation with respect to dy (and z?)
  hy[STATE_Z] = (Npix * flow->dt / thetapix) * ((R[2][2] * dy_g) / (-z_g * z_g));
  hy[STATE_PY] = (Npix * flow->dt / thetapix) * (R[2][2] / z_g);

//   Y update
//  if(S[STATE_Z] < UWB_MAX_HEIGHT){
  stateEstimatorScalarUpdate(&Hy, measuredNY-predictedNY, flow->stdDevY);
//  }
  }
}

static void stateEstimatorUpdateWithTof(tofMeasurement_t *tof)
{
  // Updates the filter with a measured distance in the z direction
 //  if(tof->distance<ZRANGE_MAX_HEIGHT) {               //modified here, but it does not work
	float h[STATE_DIM] = {0};
	arm_matrix_instance_f32 H = {1, STATE_DIM, h};

  // Only update the filter if the measurement is reliable (\hat{h} -> infty when R[2][2] -> 0)
  if (fabs(R[2][2]) > 0.1 && R[2][2] > 0){
    float angle = fabsf(acosf(R[2][2])) - DEG_TO_RAD * (15.0f / 2.0f);
    if (angle < 0.0f) {
      angle = 0.0f;
    }
    //float predictedDistance = S[STATE_Z] / cosf(angle);
    float predictedDistance = S[STATE_Z] / R[2][2];
    float measuredDistance = tof->distance; // [m]

    //Measurement equation
    //
    // h = z/((R*z_b)\dot z_b) = z/cos(alpha)
    h[STATE_Z] = 1 / R[2][2];
    //h[STATE_Z] = 1 / cosf(angle);
    logzrange = measuredDistance;     // log the zrange
    // Scalar update
    // Z update
    if (enable_zrange){   // if enable zrange when using flowdeck.
    	stateEstimatorScalarUpdate(&H, measuredDistance-predictedDistance, tof->stdDev);
     }
  }
// }
}

static void stateEstimatorFinalize(sensorData_t *sensors, uint32_t tick)
{
  // Matrix to rotate the attitude covariances once updated
  static float A[STATE_DIM][STATE_DIM];
  static arm_matrix_instance_f32 Am = {STATE_DIM, STATE_DIM, (float *)A};

  // Temporary matrices for the covariance updates
  static float tmpNN1d[STATE_DIM * STATE_DIM];
  static arm_matrix_instance_f32 tmpNN1m = {STATE_DIM, STATE_DIM, tmpNN1d};

  static float tmpNN2d[STATE_DIM * STATE_DIM];
  static arm_matrix_instance_f32 tmpNN2m = {STATE_DIM, STATE_DIM, tmpNN2d};

  // Incorporate the attitude error (Kalman filter state) with the attitude
  float v0 = S[STATE_D0];
  float v1 = S[STATE_D1];
  float v2 = S[STATE_D2];

  // Move attitude error into attitude if any of the angle errors are large enough
  if ((fabsf(v0) > 0.1e-3f || fabsf(v1) > 0.1e-3f || fabsf(v2) > 0.1e-3f) && (fabsf(v0) < 10 && fabsf(v1) < 10 && fabsf(v2) < 10))
  {
    float angle = arm_sqrt(v0*v0 + v1*v1 + v2*v2);
    float ca = arm_cos_f32(angle / 2.0f);
    float sa = arm_sin_f32(angle / 2.0f);
    float dq[4] = {ca, sa * v0 / angle, sa * v1 / angle, sa * v2 / angle};

    // rotate the quad's attitude by the delta quaternion vector computed above
    float tmpq0 = dq[0] * q[0] - dq[1] * q[1] - dq[2] * q[2] - dq[3] * q[3];
    float tmpq1 = dq[1] * q[0] + dq[0] * q[1] + dq[3] * q[2] - dq[2] * q[3];
    float tmpq2 = dq[2] * q[0] - dq[3] * q[1] + dq[0] * q[2] + dq[1] * q[3];
    float tmpq3 = dq[3] * q[0] + dq[2] * q[1] - dq[1] * q[2] + dq[0] * q[3];

    // normalize and store the result
    float norm = arm_sqrt(tmpq0 * tmpq0 + tmpq1 * tmpq1 + tmpq2 * tmpq2 + tmpq3 * tmpq3);
    q[0] = tmpq0 / norm;
    q[1] = tmpq1 / norm;
    q[2] = tmpq2 / norm;
    q[3] = tmpq3 / norm;

    /** Rotate the covariance, since we've rotated the body
     *
     * This comes from a second order approximation to:
     * Sigma_post = exps(-d) Sigma_pre exps(-d)'
     *            ~ (I + [[-d]] + [[-d]]^2 / 2) Sigma_pre (I + [[-d]] + [[-d]]^2 / 2)'
     * where d is the attitude error expressed as Rodriges parameters, ie. d = tan(|v|/2)*v/|v|
     *
     * As derived in "Covariance Correction Step for Kalman Filtering with an Attitude"
     * http://arc.aiaa.org/doi/abs/10.2514/1.G000848
     */

    float d0 = v0/2; // the attitude error vector (v0,v1,v2) is small,
    float d1 = v1/2; // so we use a first order approximation to d0 = tan(|v0|/2)*v0/|v0|
    float d2 = v2/2;

    A[STATE_X][STATE_X] = 1;
    A[STATE_Y][STATE_Y] = 1;
    A[STATE_Z][STATE_Z] = 1;

    A[STATE_PX][STATE_PX] = 1;
    A[STATE_PY][STATE_PY] = 1;
    A[STATE_PZ][STATE_PZ] = 1;

    A[STATE_D0][STATE_D0] =  1 - d1*d1/2 - d2*d2/2;
    A[STATE_D0][STATE_D1] =  d2 + d0*d1/2;
    A[STATE_D0][STATE_D2] = -d1 + d0*d2/2;

    A[STATE_D1][STATE_D0] = -d2 + d0*d1/2;
    A[STATE_D1][STATE_D1] =  1 - d0*d0/2 - d2*d2/2;
    A[STATE_D1][STATE_D2] =  d0 + d1*d2/2;

    A[STATE_D2][STATE_D0] =  d1 + d0*d2/2;
    A[STATE_D2][STATE_D1] = -d0 + d1*d2/2;
    A[STATE_D2][STATE_D2] = 1 - d0*d0/2 - d1*d1/2;

    mat_trans(&Am, &tmpNN1m); // A'
    mat_mult(&Am, &Pm, &tmpNN2m); // AP
    mat_mult(&tmpNN2m, &tmpNN1m, &Pm); //APA'
  }

  // convert the new attitude to a rotation matrix, such that we can rotate body-frame velocity and acc
  R[0][0] = q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3];
  R[0][1] = 2 * q[1] * q[2] - 2 * q[0] * q[3];
  R[0][2] = 2 * q[1] * q[3] + 2 * q[0] * q[2];

  R[1][0] = 2 * q[1] * q[2] + 2 * q[0] * q[3];
  R[1][1] = q[0] * q[0] - q[1] * q[1] + q[2] * q[2] - q[3] * q[3];
  R[1][2] = 2 * q[2] * q[3] - 2 * q[0] * q[1];

  R[2][0] = 2 * q[1] * q[3] - 2 * q[0] * q[2];
  R[2][1] = 2 * q[2] * q[3] + 2 * q[0] * q[1];
  R[2][2] = q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3];

  // reset the attitude error
  S[STATE_D0] = 0;
  S[STATE_D1] = 0;
  S[STATE_D2] = 0;

  // constrain the states

  for (int i=0; i<3; i++)
  {
    if (S[STATE_X+i] < -MAX_POSITION) { S[STATE_X+i] = -MAX_POSITION; }
    else if (S[STATE_X+i] > MAX_POSITION) { S[STATE_X+i] = MAX_POSITION; }

    if (S[STATE_PX+i] < -MAX_VELOCITY) { S[STATE_PX+i] = -MAX_VELOCITY; }
    else if (S[STATE_PX+i] > MAX_VELOCITY) { S[STATE_PX+i] = MAX_VELOCITY; }
  }

  // enforce symmetry of the covariance matrix, and ensure the values stay bounded
  for (int i=0; i<STATE_DIM; i++) {
    for (int j=i; j<STATE_DIM; j++) {
      float p = 0.5f*P[i][j] + 0.5f*P[j][i];
      if (isnan(p) || p > MAX_COVARIANCE) {
        P[i][j] = P[j][i] = MAX_COVARIANCE;
      } else if ( i==j && p < MIN_COVARIANCE ) {
        P[i][j] = P[j][i] = MIN_COVARIANCE;
      } else {
        P[i][j] = P[j][i] = p;
      }
    }
  }

  stateEstimatorAssertNotNaN();
}


static void stateEstimatorExternalizeState(state_t *state, sensorData_t *sensors, uint32_t tick)
{

  // position state is already in world frame
  state->position = (point_t){
      .timestamp = tick,
      .x = S[STATE_X],
      .y = S[STATE_Y],
      .z = S[STATE_Z]
  };

  // velocity is in body frame and needs to be rotated to world frame
  state->velocity = (velocity_t){
      .timestamp = tick,
      .x = R[0][0]*S[STATE_PX] + R[0][1]*S[STATE_PY] + R[0][2]*S[STATE_PZ],
      .y = R[1][0]*S[STATE_PX] + R[1][1]*S[STATE_PY] + R[1][2]*S[STATE_PZ],
      .z = R[2][0]*S[STATE_PX] + R[2][1]*S[STATE_PY] + R[2][2]*S[STATE_PZ]
  };

  // Accelerometer measurements are in the body frame and need to be rotated to world frame.
  // Furthermore, the legacy code requires acc.z to be acceleration without gravity.
  // Finally, note that these accelerations are in Gs, and not in m/s^2, hence - 1 for removing gravity
  state->acc = (acc_t){
      .timestamp = tick,
      .x = R[0][0]*sensors->acc.x + R[0][1]*sensors->acc.y + R[0][2]*sensors->acc.z,
      .y = R[1][0]*sensors->acc.x + R[1][1]*sensors->acc.y + R[1][2]*sensors->acc.z,
      .z = R[2][0]*sensors->acc.x + R[2][1]*sensors->acc.y + R[2][2]*sensors->acc.z - 1
  };

  // convert the new attitude into Euler YPR
  float yaw = atan2f(2*(q[1]*q[2]+q[0]*q[3]) , q[0]*q[0] + q[1]*q[1] - q[2]*q[2] - q[3]*q[3]);
  float pitch = asinf(-2*(q[1]*q[3] - q[0]*q[2]));
  float roll = atan2f(2*(q[2]*q[3]+q[0]*q[1]) , q[0]*q[0] - q[1]*q[1] - q[2]*q[2] + q[3]*q[3]);

  // [CHANGE] yaw estimate
  yaw_logback = yaw;

  // Save attitude, adjusted for the legacy CF2 body coordinate system
  state->attitude = (attitude_t){
      .timestamp = tick,
      .roll = roll*RAD_TO_DEG,
      .pitch = -pitch*RAD_TO_DEG,
      .yaw = yaw*RAD_TO_DEG
  };

  log_yaw = yaw*RAD_TO_DEG;  // save yaw angle for logging (testing)

  // Save quaternion, hopefully one day this could be used in a better controller.
  // Note that this is not adjusted for the legacy coordinate system
  state->attitudeQuaternion = (quaternion_t){
      .timestamp = tick,
      .w = q[0],
      .x = q[1],
      .y = q[2],
      .z = q[3]
  };
}


void estimatorKalmanInit(void) {
  if (!isInit)
  {
    distDataQueue = xQueueCreate(DIST_QUEUE_LENGTH, sizeof(distanceMeasurement_t));
    posDataQueue = xQueueCreate(POS_QUEUE_LENGTH, sizeof(positionMeasurement_t));
    posvelDataQueue = xQueueCreate(POSVEL_QUEUE_LENGTH, sizeof(posvelMeasurement_t));
    posvelyawDataQueue = xQueueCreate(POSVELYAW_QUEUE_LENGTH, sizeof(posvelyawMeasurement_t)); // [CHANGE] yaw estimation
    tdoaDataQueue = xQueueCreate(UWB_QUEUE_LENGTH, sizeof(tdoaMeasurement_t));                 // [Note] the tdoa Queue size, each time 10 tdoa meas. are in the queue
    flowDataQueue = xQueueCreate(FLOW_QUEUE_LENGTH, sizeof(flowMeasurement_t));
    tofDataQueue = xQueueCreate(TOF_QUEUE_LENGTH, sizeof(tofMeasurement_t));
    heightDataQueue = xQueueCreate(HEIGHT_QUEUE_LENGTH, sizeof(heightMeasurement_t));
  }
  else
  {
    xQueueReset(distDataQueue);
    xQueueReset(posDataQueue);
    xQueueReset(posvelDataQueue);
    xQueueReset(posvelyawDataQueue); // [CHANGE] yaw estimate
    xQueueReset(tdoaDataQueue);
    xQueueReset(flowDataQueue);
    xQueueReset(tofDataQueue);
  }

  lastPrediction = xTaskGetTickCount();
  lastBaroUpdate = xTaskGetTickCount();
  lastTDOAUpdate = xTaskGetTickCount();
  lastPNUpdate = xTaskGetTickCount();

  accAccumulator = (Axis3f){.axis={0}};
  gyroAccumulator = (Axis3f){.axis={0}};
  thrustAccumulator = 0;
  baroAccumulator.asl = 0;

  accAccumulatorCount = 0;
  gyroAccumulatorCount = 0;
  thrustAccumulatorCount = 0;
  baroAccumulatorCount = 0;

  // Reset all matrices to 0 (like uppon system reset)
  memset(q, 0, sizeof(q));
  memset(R, 0, sizeof(R));
  memset(P, 0, sizeof(S));

  // TODO: Can we initialize this more intelligently?
  S[STATE_X] = initialX;
  S[STATE_Y] = initialY;
  S[STATE_Z] = initialZ;
  S[STATE_PX] = 0;
  S[STATE_PY] = 0;
  S[STATE_PZ] = 0;
  S[STATE_D0] = 0;
  S[STATE_D1] = 0;
  S[STATE_D2] = 0;

  // reset the attitude quaternion
  q[0] = 1; q[1] = 0; q[2] = 0; q[3] = 0;

  // then set the initial rotation matrix to the identity. This only affects
  // the first prediction step, since in the finalization, after shifting
  // attitude errors into the attitude state, the rotation matrix is updated.
  for(int i=0; i<3; i++) { for(int j=0; j<3; j++) { R[i][j] = i==j ? 1 : 0; }}

  for (int i=0; i< STATE_DIM; i++) {
    for (int j=0; j < STATE_DIM; j++) {
      P[i][j] = 0; // set covariances to zero (diagonals will be changed from zero in the next section)
    }
  }

  // initialize state variances
  P[STATE_X][STATE_X]  = powf(stdDevInitialPosition_xy, 2);
  P[STATE_Y][STATE_Y]  = powf(stdDevInitialPosition_xy, 2);
  P[STATE_Z][STATE_Z]  = powf(stdDevInitialPosition_z, 2);

  P[STATE_PX][STATE_PX] = powf(stdDevInitialVelocity, 2);
  P[STATE_PY][STATE_PY] = powf(stdDevInitialVelocity, 2);
  P[STATE_PZ][STATE_PZ] = powf(stdDevInitialVelocity, 2);

  P[STATE_D0][STATE_D0] = powf(stdDevInitialAttitude_rollpitch, 2);
  P[STATE_D1][STATE_D1] = powf(stdDevInitialAttitude_rollpitch, 2);
  P[STATE_D2][STATE_D2] = powf(stdDevInitialAttitude_yaw, 2);

  varSkew = powf(stdDevInitialSkew, 2);

  tdoaCount = 0;
  isInit = true;
}

static bool stateEstimatorEnqueueExternalMeasurement(xQueueHandle queue, void *measurement)
{
  portBASE_TYPE result;
  bool isInInterrupt = (SCB->ICSR & SCB_ICSR_VECTACTIVE_Msk) != 0;

  if (isInInterrupt) {
    portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
    result = xQueueSendFromISR(queue, measurement, &xHigherPriorityTaskWoken);
    if(xHigherPriorityTaskWoken == pdTRUE)
    {
      portYIELD();
    }
  } else {
    result = xQueueSend(queue, measurement, 0);
  }
  return (result==pdTRUE);
}

bool estimatorKalmanEnqueueTDOA(tdoaMeasurement_t *uwb)
{
  ASSERT(isInit);
  return stateEstimatorEnqueueExternalMeasurement(tdoaDataQueue, (void *)uwb);
}

bool estimatorKalmanEnqueuePosition(positionMeasurement_t *pos)
{
  ASSERT(isInit);
  return stateEstimatorEnqueueExternalMeasurement(posDataQueue, (void *)pos);
}

bool estimatorKalmanEnqueuePosVel(posvelMeasurement_t *posvel)
{
  ASSERT(isInit);
  return stateEstimatorEnqueueExternalMeasurement(posvelDataQueue, (void *)posvel);
}

bool estimatorKalmanEnqueuePosVelYaw(posvelyawMeasurement_t *posvelyaw) // [CHANGE] yaw estimation
{
  ASSERT(isInit);
  return stateEstimatorEnqueueExternalMeasurement(posvelyawDataQueue, (void *)posvelyaw);
}

bool estimatorKalmanEnqueueDistance(distanceMeasurement_t *dist)
{
  ASSERT(isInit);
  return stateEstimatorEnqueueExternalMeasurement(distDataQueue, (void *)dist);
}

bool estimatorKalmanEnqueueFlow(flowMeasurement_t *flow)
{
  // A flow measurement (dnx,  dny) [accumulated pixels]
  ASSERT(isInit);
  return stateEstimatorEnqueueExternalMeasurement(flowDataQueue, (void *)flow);
}

bool estimatorKalmanEnqueueTOF(tofMeasurement_t *tof)
{
  // A distance (distance) [m] to the ground along the z_B axis.
  ASSERT(isInit);
  return stateEstimatorEnqueueExternalMeasurement(tofDataQueue, (void *)tof);
}

bool estimatorKalmanEnqueueAsoluteHeight(heightMeasurement_t *height)
{
  // A distance (height) [m] to the ground along the z axis.
  ASSERT(isInit);
  return stateEstimatorEnqueueExternalMeasurement(heightDataQueue, (void *)height);
}

bool estimatorKalmanTest(void)
{
  // TODO: Figure out what we could test?
  return isInit;
}

float estimatorKalmanGetElevation()
{
  // Return elevation, used in the optical flow
  return S[STATE_Z];
}

void estimatorKalmanSetShift(float deltax, float deltay)
{
  // Return elevation, used in the optical flow
  S[STATE_X] -= deltax;
  S[STATE_Y] -= deltay;
}

void estimatorKalmanGetEstimatedPos(point_t* pos) {
  pos->x = S[STATE_X];
  pos->y = S[STATE_Y];
  pos->z = S[STATE_Z];
}

// Temporary development groups

//LOG_GROUP_START(kalman_pred)
//  LOG_ADD(LOG_FLOAT, predNX, &predictedNX)
//  LOG_ADD(LOG_FLOAT, predNY, &predictedNY)
//  LOG_ADD(LOG_FLOAT, measNX, &measuredNX)
//  LOG_ADD(LOG_FLOAT, measNY, &measuredNY)
//LOG_GROUP_STOP(kalman_pred)

LOG_GROUP_START(twr_ekf)
//  LOG_ADD(LOG_FLOAT, distance, &twrDist)
//  LOG_ADD(LOG_UINT8, anchorID, &anchorID)
//  LOG_ADD(LOG_FLOAT, yaw, &log_yaw)
  LOG_ADD(LOG_FLOAT, dx, &measuredNX)
  LOG_ADD(LOG_FLOAT, dy, &measuredNY)
  LOG_ADD(LOG_FLOAT, zrange, &logzrange)
LOG_GROUP_STOP(twr_ekf)

LOG_GROUP_START(tdoa_ekf)
  LOG_ADD(LOG_FLOAT, distance, &tdoaDist)
  LOG_ADD(LOG_UINT8, anchorID, &tdoaID)
LOG_GROUP_STOP(tdoa_ekf)

// Stock log groups
LOG_GROUP_START(kalman)
  LOG_ADD(LOG_UINT8, inFlight, &quadIsFlying)
  LOG_ADD(LOG_FLOAT, stateX, &S[STATE_X])
  LOG_ADD(LOG_FLOAT, stateY, &S[STATE_Y])
  LOG_ADD(LOG_FLOAT, stateZ, &S[STATE_Z])
  LOG_ADD(LOG_FLOAT, statePX, &S[STATE_PX])
  LOG_ADD(LOG_FLOAT, statePY, &S[STATE_PY])
  LOG_ADD(LOG_FLOAT, statePZ, &S[STATE_PZ])
  LOG_ADD(LOG_FLOAT, stateD0, &S[STATE_D0])
  LOG_ADD(LOG_FLOAT, stateD1, &S[STATE_D1])
  LOG_ADD(LOG_FLOAT, stateD2, &S[STATE_D2])
  //LOG_ADD(LOG_FLOAT, stateSkew, &stateSkew)
  LOG_ADD(LOG_FLOAT, varX, &P[STATE_X][STATE_X])
  LOG_ADD(LOG_FLOAT, varY, &P[STATE_Y][STATE_Y])
  LOG_ADD(LOG_FLOAT, varZ, &P[STATE_Z][STATE_Z])
  /*
  LOG_ADD(LOG_FLOAT, varPX, &P[STATE_PX][STATE_PX])
  LOG_ADD(LOG_FLOAT, varPY, &P[STATE_PY][STATE_PY])
  LOG_ADD(LOG_FLOAT, varPZ, &P[STATE_PZ][STATE_PZ])
  LOG_ADD(LOG_FLOAT, varD0, &P[STATE_D0][STATE_D0])
  LOG_ADD(LOG_FLOAT, varD1, &P[STATE_D1][STATE_D1])
  LOG_ADD(LOG_FLOAT, varD2, &P[STATE_D2][STATE_D2])
  */
  //LOG_ADD(LOG_FLOAT, varSkew, &varSkew)
//   LOG_ADD(LOG_FLOAT, q0, &q[0])
//   LOG_ADD(LOG_FLOAT, q1, &q[1])
//   LOG_ADD(LOG_FLOAT, q2, &q[2])
//   LOG_ADD(LOG_FLOAT, q3, &q[3])
//   LOG_ADD(LOG_FLOAT, yaw, &yaw_logback)
//   LOG_ADD(LOG_FLOAT, yaw_error, &yaw_error_logback)
  // Chi-square debug
//   LOG_ADD(LOG_FLOAT, dm,       &log_dm)
//   LOG_ADD(LOG_FLOAT, errAbs,   &log_errAbs)
//   LOG_ADD(LOG_FLOAT, hphr_chi, &log_HPHR_chi)
    // DNN debug
  LOG_ADD(LOG_FLOAT, dnn_bias, &log_bias)

LOG_GROUP_STOP(kalman)

PARAM_GROUP_START(kalman)
  PARAM_ADD(PARAM_FLOAT, init_x, &initialX)
  PARAM_ADD(PARAM_FLOAT, init_y, &initialY)
  PARAM_ADD(PARAM_UINT8, resetEstimation, &resetEstimation)
//  PARAM_ADD(PARAM_UINT8, quadIsFlying, &quadIsFlying)
//  PARAM_ADD(PARAM_FLOAT, pNAcc_xy, &procNoiseAcc_xy)
//  PARAM_ADD(PARAM_FLOAT, pNAcc_z, &procNoiseAcc_z)
//  PARAM_ADD(PARAM_FLOAT, pNVel, &procNoiseVel)
//  PARAM_ADD(PARAM_FLOAT, pNPos, &procNoisePos)
//  PARAM_ADD(PARAM_FLOAT, pNAtt, &procNoiseAtt)
//  PARAM_ADD(PARAM_FLOAT, pNSkew, &procNoiseSkew)
//  PARAM_ADD(PARAM_FLOAT, mNBaro, &measNoiseBaro)
//  PARAM_ADD(PARAM_FLOAT, mNGyro_rollpitch, &measNoiseGyro_rollpitch)
//  PARAM_ADD(PARAM_FLOAT, mNGyro_yaw, &measNoiseGyro_yaw)
PARAM_GROUP_STOP(kalman)
