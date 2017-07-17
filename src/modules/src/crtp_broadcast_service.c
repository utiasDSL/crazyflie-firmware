#include <errno.h>
#include <string.h>
#include <stdint.h>

/* FreeRtos includes */
#include "FreeRTOS.h"
#include "task.h"

#include "crtp.h"
#include "crtp_broadcast_service.h"
#include "log.h"
#include "param.h"

#include "configblock.h"
#include "estimator_kalman.h"

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
static bool isInit = false;
static uint8_t my_id;
static uint8_t bc_id;
static positionMeasurement_t broadcast_pos;
static uint16_t flag = 0;




static void broadcastSrvCrtpCB(CRTPPacket* pk);

void broadcastSrvInit()
{
  if (isInit) {
    return;
  }

  crtpRegisterPortCB(CRTP_PORT_EXTPOS_BRINGUP, broadcastSrvCrtpCB);
  isInit = true;

  uint64_t address = configblockGetRadioAddress();
  my_id = address & 0xFF;
  flag = 1;
}

static void broadcastSrvCrtpCB(CRTPPacket* pk)
{
  flag = 2;
  struct data_vicon* d = ((struct data_vicon*) pk->data);
  for (int i=0; i < 2; ++i) {
    if (d->pose[i].id == my_id) {
      flag = 3;
      bc_id = d->pose[i].id;
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

LOG_GROUP_START(broadcast_pos)
LOG_ADD(LOG_FLOAT, X, &broadcast_pos.x)
LOG_ADD(LOG_FLOAT, Y, &broadcast_pos.y)
LOG_ADD(LOG_FLOAT, Z, &broadcast_pos.z)
LOG_GROUP_STOP(broadcast_pos)

LOG_GROUP_START(broadcast)
LOG_ADD(LOG_UINT16, Flag, &flag)
LOG_ADD(LOG_UINT8, ID, &my_id)
LOG_ADD(LOG_UINT8, IDInput, &bc_id)
LOG_GROUP_STOP(broadcast)
