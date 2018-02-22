#include <errno.h>
#include <string.h>
#include <stdint.h>

/* FreeRtos includes */
#include "FreeRTOS.h"
#include "task.h"

#include "crtp.h"
#include "crtp_broadcast_service.h"
#include "crtp_commander.h"
#include "log.h"
#include "param.h"
#include "packetdef.h"
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

static ExtPositionCache crtpExtPosCache;
static bool isInit_pos = false;
static bool isInit_cmd = false;

static uint8_t my_id;
static uint8_t bc_id;
static positionMeasurement_t broadcast_pos;
static positionMeasurement_t broadcast_cmd;
static setpoint_t decoded_cmd;
static uint16_t flag = 0;
static uint16_t cmd_flag = 0;
static uint16_t numPacketsReceived = 0, numPacketsReceived2 = 0;
static uint16_t numPacketsAccepted = 0, numPacketsAccepted2 = 0;



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

  uint64_t address = configblockGetRadioAddress();
  my_id = address & 0xFF;
  flag = 1;
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
  cmd_flag = 1;
}

static void bcPosSrvCrtpCB(CRTPPacket* pk)
{
  numPacketsReceived++;
  flag = 2;
  struct data_vicon* d = ((struct data_vicon*) pk->data);
  for (int i=0; i < 2; ++i) {
    bc_id = d->pose[i].id;
    if (d->pose[i].id == my_id) {
      numPacketsAccepted++;
      flag = 3;
      struct CrtpExtPosition data;
      data.x = position_fix24_to_float(d->pose[i].x);
      data.y = position_fix24_to_float(d->pose[i].y);
      data.z = position_fix24_to_float(d->pose[i].z);

      crtpExtPosCache.currVal[!crtpExtPosCache.activeSide] = data;
      crtpExtPosCache.activeSide = !crtpExtPosCache.activeSide;
      crtpExtPosCache.timestamp = xTaskGetTickCount();
      }
    }
}

bool getExtPositionBC(state_t *state)
{
  // Only use position information if it's valid and recent
  if ((xTaskGetTickCount() - crtpExtPosCache.timestamp) < M2T(5)) {
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
  numPacketsReceived2++;

  if(pk->port == CRTP_PORT_SETPOINT && pk->channel == 0) {
    struct data_setpoint* d = ((struct data_setpoint*) pk->data);
    for (int i=0; i < 2; ++i) {
      if (d->pose[i].id == my_id) {
        numPacketsAccepted2++;
        cmd_flag = 2;

        struct CommanderCrtpLegacyValues data;
        data.roll = position_fix24_to_float(d->pose[i].x);
        data.pitch = position_fix24_to_float(d->pose[i].y);
        data.thrust = (int) (position_fix24_to_float(d->pose[i].z) * 1000.0f);
        data.yaw = position_fix24_to_float(d->pose[i].yaw) / 3.1415926f * 180.0f;

        //crtpCommanderRpytDecodeSetpoint(&setpoint, &pk);
        crtpCommanderRpytDecodeSetpoint(&setpoint, pk, true, &data);
        commanderSetSetpoint(&setpoint, COMMANDER_PRIORITY_CRTP);
        broadcast_cmd.x = data.roll;
        broadcast_cmd.y = data.pitch;
        broadcast_cmd.z = data.thrust;
        broadcast_cmd.stdDev = data.yaw;
        memcpy(&decoded_cmd, &setpoint, sizeof(setpoint));
        //broadcast_cmd.x = setpoint.position.x;
        //broadcast_cmd.y = setpoint.position.y;
        //broadcast_cmd.z = setpoint.position.z;

        }
      }
  } else if (pk->port == CRTP_PORT_SETPOINT_GENERIC && pk->channel == 0) {
    if (pk->data[0] == my_id){
      numPacketsAccepted2++; 
      crtpCommanderGenericDecodeSetpoint(&setpoint, pk);
      commanderSetSetpoint(&setpoint, COMMANDER_PRIORITY_CRTP); 
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

LOG_GROUP_START(broadcast_flag)
LOG_ADD(LOG_UINT16, pos_Flag, &flag)
LOG_ADD(LOG_UINT16, cmd_Flag, &cmd_flag)
LOG_ADD(LOG_UINT8, pos_ID, &my_id)
LOG_ADD(LOG_UINT8, pos_IDInput, &bc_id)
LOG_GROUP_STOP(broadcast_flag)

LOG_GROUP_START(broadcast_count)
LOG_ADD(LOG_UINT16, Rx, &numPacketsReceived)
LOG_ADD(LOG_UINT16, Acc, &numPacketsAccepted)
LOG_ADD(LOG_UINT16, Rx2, &numPacketsReceived2)
LOG_ADD(LOG_UINT16, Acc2, &numPacketsAccepted2)
LOG_GROUP_STOP(broadcast_count)

LOG_GROUP_START(broadcast_setpoint)
LOG_ADD(LOG_FLOAT, X, &decoded_cmd.position.x)
LOG_ADD(LOG_FLOAT, Y, &decoded_cmd.position.y)
LOG_ADD(LOG_FLOAT, Z, &decoded_cmd.position.z)
LOG_ADD(LOG_FLOAT, VZ, &decoded_cmd.velocity.z)
LOG_ADD(LOG_FLOAT, Roll, &decoded_cmd.attitude.roll)
LOG_ADD(LOG_FLOAT, Pitch, &decoded_cmd.attitude.pitch)
LOG_ADD(LOG_FLOAT, Yaw, &decoded_cmd.attitude.yaw)
LOG_GROUP_STOP(broadcast_setpoint)
