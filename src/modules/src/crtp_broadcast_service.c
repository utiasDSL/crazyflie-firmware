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
  uint32_t timestamp; // FreeRTOS ticks
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

static uint32_t numPacketsReceivedPos = 0, numPacketsReceivedCmd = 0;

static Sampling posRxFreq, cmdRxFreq;

static void bcPosSrvCrtpCB(CRTPPacket* pk);
static void bcCmdSrvCrtpCB(CRTPPacket* pk);

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

      crtpExtPosCache.currVal[!crtpExtPosCache.activeSide] = data;
      crtpExtPosCache.activeSide = !crtpExtPosCache.activeSide;
      crtpExtPosCache.timestamp = xTaskGetTickCount();
      }
    }
}

bool getExtPositionBC(state_t *state)
{
  // Only use position information if it's valid and recent
  if ((xTaskGetTickCount() - crtpExtPosCache.timestamp) < M2T(100)) {
    // Get the updated position from the mocap
    broadcast_pos.x = crtpExtPosCache.currVal[crtpExtPosCache.activeSide].x;
    broadcast_pos.y = crtpExtPosCache.currVal[crtpExtPosCache.activeSide].y;
    broadcast_pos.z = crtpExtPosCache.currVal[crtpExtPosCache.activeSide].z;
    broadcast_pos.stdDev = 0.01;
#ifndef PLATFORM_CF1
    estimatorKalmanEnqueuePosition(&broadcast_pos);
#endif

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
			  broadcast_cmd.x = setpoint.position.x;
			  broadcast_cmd.y = setpoint.position.y;
			  broadcast_cmd.z = setpoint.velocity.z;

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

