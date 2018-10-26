#include <errno.h>
#include <string.h>
#include <stdint.h>
#include <math.h>
/* FreeRtos includes */
#include "FreeRTOS.h"
#include "task.h"

#include "crtp.h"
#include "crtp_broadcast_service.h"
#include "crtp_commander.h"
#include "log.h"
#include "param.h"
#include "configblock.h"
#include "estimator_kalman.h"
#include "commander.h"

/**
 * Position data cache
 */
typedef struct
{
  struct CrtpExtPosition currVal[2];
  bool activeSide;
  uint32_t timestamp[2]; // FreeRTOS ticks
  bool new_data;	  // new position data
} ExtPositionCache;

typedef struct{
	float data[500];
	uint16_t index;
	float avg;
	float max;
	float min;
	float stddev;
	uint32_t last_timestamp;
}Sampling;

static ExtPositionCache crtpExtPosCache;
static bool isInit_pos = false;
static bool isInit_cmd = false;

static uint8_t my_id;
static uint8_t bc_id;
static positionMeasurement_t broadcast_pos;
static positionMeasurement_t broadcast_cmd;
static posvelMeasurement_t posvel;
static uint32_t numPacketsReceivedPos = 0, numPacketsReceivedCmd = 0;

static Sampling posRxFreq, cmdRxFreq;

static void bcPosSrvCrtpCB(CRTPPacket* pk);
static void bcCmdSrvCrtpCB(CRTPPacket* pk);

/*
 * Log Variable
 */

static velocity_t curr_vel;
static point_t curr_pos;
static uint32_t last_time;

void bcPosInit()
{
  if (isInit_pos) {
    return;
  }

  crtpInit();
  crtpRegisterPortCB(CRTP_PORT_EXTPOS_BRINGUP, bcPosSrvCrtpCB);
  isInit_pos = true;
  posRxFreq.max = 0.f;
  posRxFreq.min = 150.f;

  cmdRxFreq.max = 0.f;
  cmdRxFreq.min = 150.f;

  uint64_t address = configblockGetRadioAddress();
  my_id = address & 0xFF;
}

void bcCmdInit(void)
{
  if(isInit_cmd) {
    return;
  }
  crtpInit();
  crtpRegisterPortCB(CRTP_PORT_SETPOINT, bcCmdSrvCrtpCB);
  crtpRegisterPortCB(CRTP_PORT_SETPOINT_GENERIC, bcCmdSrvCrtpCB);
  isInit_cmd = true;
}

static void bcPosSrvCrtpCB(CRTPPacket* pk)
{

  crtp_vicon_t* d = ((crtp_vicon_t*) pk->data);
  for (int i=0; i < 2; ++i) {
    bc_id = d->pose[i].id;
    if (d->pose[i].id == my_id) {
    	numPacketsReceivedPos++;
      struct CrtpExtPosition data;
      data.x = position_fix24_to_float(d->pose[i].x);
      data.y = position_fix24_to_float(d->pose[i].y);
      data.z = position_fix24_to_float(d->pose[i].z);


      if(posRxFreq.last_timestamp!=0){
    	  float interval = T2M(xTaskGetTickCount()- posRxFreq.last_timestamp);
    	  float sum = posRxFreq.avg * 500 - posRxFreq.data[posRxFreq.index];
    	  posRxFreq.data[posRxFreq.index] = 1000.f / interval;
    	  posRxFreq.avg = (sum + posRxFreq.data[posRxFreq.index]) / 500.0f;

    	  posRxFreq.max = posRxFreq.max > posRxFreq.data[posRxFreq.index] ? posRxFreq.max : posRxFreq.data[posRxFreq.index];
    	  posRxFreq.min = posRxFreq.min < posRxFreq.data[posRxFreq.index] ? posRxFreq.min : posRxFreq.data[posRxFreq.index];

    	  sum = 0;
    	  for(int i=0; i<500; ++i)
    		  sum += (float) pow((posRxFreq.data[i] - posRxFreq.avg),2);
    	  sum /= 499.f;
    	  posRxFreq.stddev = (float) pow(sum, 0.5);

      }

      posRxFreq.index = (posRxFreq.index + 1)%500;
      posRxFreq.last_timestamp = xTaskGetTickCount();




      if(pow((curr_pos.x - data.x), 2) + pow((curr_pos.y - data.y), 2) + pow((curr_pos.z -data.z), 2) > 25e-10){
    	  float dt = (float) (xTaskGetTickCount() - last_time) / 1000.f;
    	  curr_vel.x = (data.x - curr_pos.x) / dt;
    	  curr_vel.y = (data.y - curr_pos.y) / dt;
    	  curr_vel.z = (data.z - curr_pos.z) / dt;

    	  last_time = xTaskGetTickCount();
    	  curr_pos.x = data.x;
    	  curr_pos.y = data.y;
    	  curr_pos.z = data.z;
      }


      crtpExtPosCache.currVal[!crtpExtPosCache.activeSide] = data;
      crtpExtPosCache.timestamp[!crtpExtPosCache.activeSide] = xTaskGetTickCount();
      crtpExtPosCache.activeSide = !crtpExtPosCache.activeSide;
      crtpExtPosCache.new_data = true;

      }
    }
}

bool getExtPositionBC(state_t *state)
{
  // Only use position information if it's valid and recent
  if ((xTaskGetTickCount() - crtpExtPosCache.timestamp[crtpExtPosCache.activeSide]) < M2T(5)) {
    // Get the updated position from the mocap
    broadcast_pos.x = crtpExtPosCache.currVal[crtpExtPosCache.activeSide].x;
    broadcast_pos.y = crtpExtPosCache.currVal[crtpExtPosCache.activeSide].y;
    broadcast_pos.z = crtpExtPosCache.currVal[crtpExtPosCache.activeSide].z;
    broadcast_pos.stdDev = 0.0005;

    posvelMeasurement_t posvel;
    posvel.x = crtpExtPosCache.currVal[crtpExtPosCache.activeSide].x;
    posvel.y = crtpExtPosCache.currVal[crtpExtPosCache.activeSide].y;
    posvel.z = crtpExtPosCache.currVal[crtpExtPosCache.activeSide].z;

#ifndef PLATFORM_CF1
//    estimatorKalmanEnqueuePosition(&broadcast_pos);
    estimatorKalmanEnqueuePosVel(&posvel);
#endif

    return true;
  }
  return false;
}

bool getExtPosVelBC(state_t *state){

	if(crtpExtPosCache.new_data && crtpExtPosCache.timestamp[!crtpExtPosCache.activeSide] != 0){
		// get position

	    posvel.x = crtpExtPosCache.currVal[crtpExtPosCache.activeSide].x;
	    posvel.y = crtpExtPosCache.currVal[crtpExtPosCache.activeSide].y;
	    posvel.z = crtpExtPosCache.currVal[crtpExtPosCache.activeSide].z;

	    posvel.stdDev_pos = 0.0005;

		float dt = T2M(crtpExtPosCache.timestamp[crtpExtPosCache.activeSide] - crtpExtPosCache.timestamp[!crtpExtPosCache.activeSide])/1000.f;
		if(dt < 7e-3f)
			dt = 7e-3f;
		else if(dt > 0.1f)
			dt = 0.1f;

		// dp/dt

		posvel.vx =  (crtpExtPosCache.currVal[crtpExtPosCache.activeSide].x - crtpExtPosCache.currVal[!crtpExtPosCache.activeSide].x)/dt;
		posvel.vy =  (crtpExtPosCache.currVal[crtpExtPosCache.activeSide].y - crtpExtPosCache.currVal[!crtpExtPosCache.activeSide].y)/dt;
		posvel.vz =  (crtpExtPosCache.currVal[crtpExtPosCache.activeSide].z - crtpExtPosCache.currVal[!crtpExtPosCache.activeSide].z)/dt;

		posvel.stdDev_vel = 4.414e-3;

		#ifndef PLATFORM_CF1
		//    estimatorKalmanEnqueuePosition(&broadcast_pos);
		estimatorKalmanEnqueuePosVel(&posvel);
		#endif
		crtpExtPosCache.new_data = false;

		return true;
	}

	return false;
}

static void bcCmdSrvCrtpCB(CRTPPacket* pk)
{
  static setpoint_t setpoint;

 if (pk->port == CRTP_PORT_SETPOINT_GENERIC && pk->channel == 0) {
	  crtp_setpoint_t* d = ((crtp_setpoint_t*) pk->data);
	  for (int i=0; i < 2; ++i) {
		  if (d->pose[i].id == my_id) {
			  numPacketsReceivedCmd++;

			  bccrtpCommanderGenericDecodeSetpoint(&setpoint, d, i);
			  commanderSetSetpoint(&setpoint, COMMANDER_PRIORITY_CRTP);
			  broadcast_cmd.x = setpoint.attitude.roll;
			  broadcast_cmd.y = setpoint.attitude.pitch;
			  broadcast_cmd.z = setpoint.thrust;

			  if(cmdRxFreq.last_timestamp!=0){
				  float interval = T2M(xTaskGetTickCount()- cmdRxFreq.last_timestamp);
				  float sum = cmdRxFreq.avg * 500 - cmdRxFreq.data[cmdRxFreq.index];
				  cmdRxFreq.data[cmdRxFreq.index] = 1000.f / interval;
				  cmdRxFreq.avg = (sum + cmdRxFreq.data[cmdRxFreq.index]) / 500.0f;

				  cmdRxFreq.max = cmdRxFreq.max > cmdRxFreq.data[cmdRxFreq.index] ? cmdRxFreq.max : cmdRxFreq.data[cmdRxFreq.index];
				  cmdRxFreq.min = cmdRxFreq.min < cmdRxFreq.data[cmdRxFreq.index] ? cmdRxFreq.min : cmdRxFreq.data[cmdRxFreq.index];

				  sum = 0;
				  for(int i=0; i<500; ++i)
					  sum += (float) pow((cmdRxFreq.data[i] - cmdRxFreq.avg),2);
				  sum /=  499.f;
				  cmdRxFreq.stddev = (float) pow(sum, 0.5);
			 }

			 cmdRxFreq.index = (cmdRxFreq.index + 1)%500;
			 cmdRxFreq.last_timestamp = xTaskGetTickCount();
	      }
      }
  }
}

LOG_GROUP_START(vicon)
LOG_ADD(LOG_FLOAT, X, &posvel.x)
LOG_ADD(LOG_FLOAT, Y, &posvel.y)
LOG_ADD(LOG_FLOAT, Z, &posvel.z)

LOG_ADD(LOG_FLOAT, Vx, &posvel.vx)
LOG_ADD(LOG_FLOAT, Vy, &posvel.vy)
LOG_ADD(LOG_FLOAT, Vz, &posvel.vz)
LOG_GROUP_STOP(vicon)

LOG_GROUP_START(broadcast_pos)
LOG_ADD(LOG_FLOAT, X, &broadcast_pos.x)
LOG_ADD(LOG_FLOAT, Y, &broadcast_pos.y)
LOG_ADD(LOG_FLOAT, Z, &broadcast_pos.z)
LOG_GROUP_STOP(broadcast_pos)

LOG_GROUP_START(broadcast_cmd)
LOG_ADD(LOG_FLOAT, X, &broadcast_cmd.x)
LOG_ADD(LOG_FLOAT, Y, &broadcast_cmd.y)
LOG_ADD(LOG_FLOAT, Z, &broadcast_cmd.z)
LOG_ADD(LOG_FLOAT, Thrust, &broadcast_cmd.stdDev)
LOG_GROUP_STOP(broadcast_cmd)

LOG_GROUP_START(broadcast_test)
LOG_ADD(LOG_UINT32, RP, &numPacketsReceivedPos)
LOG_ADD(LOG_UINT32, RC, &numPacketsReceivedCmd)
LOG_ADD(LOG_FLOAT, aRFP, &posRxFreq.avg)
LOG_ADD(LOG_FLOAT, aRFC, &cmdRxFreq.avg)
LOG_ADD(LOG_FLOAT, mxRFP, &posRxFreq.max)
LOG_ADD(LOG_FLOAT, mxRFC, &cmdRxFreq.max)
LOG_ADD(LOG_FLOAT, mnRFP, &posRxFreq.min)
LOG_ADD(LOG_FLOAT, mnRFC, &cmdRxFreq.min)
LOG_ADD(LOG_FLOAT, sRFP, &posRxFreq.stddev)
//LOG_ADD(LOG_FLOAT, sRFC, &cmdRxFreq.stddev)
LOG_GROUP_STOP(broadcast_test)

