/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie Firmware
 *
 * Copyright (C) 2011-2017 Bitcraze AB
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
#include <stdbool.h>
#include <string.h>

#include "crtp_commander.h"

#include "commander.h"
#include "param.h"
#include "crtp.h"
#include "broadcast_data.h"
#include "num.h"
#include "log.h"
#include "FreeRTOS.h"

/* The generic commander format contains a packet type and data that has to be
 * decoded into a setpoint_t structure. The aim is to make it future-proof
 * by easily allowing the addition of new packets for future use cases.
 *
 * The packet format is:
 * +------+==========================+
 * | TYPE |     DATA                 |
 * +------+==========================+
 *
 * The type is defined bellow together with a decoder function that should take
 * the data buffer in and fill up a setpoint_t structure.
 * The maximum data size is 29 bytes.
 */

/* To add a new packet:
 *   1 - Add a new type in the packetType_e enum.
 *   2 - Implement a decoder function with good documentation about the data
 *       structure and the intent of the packet.
 *   3 - Add the decoder function to the packetDecoders array.
 *   4 - Create a new params group for your handler if necessary
 *   5 - Pull-request your change :-)
 */

typedef void (*packetDecoder_t)(setpoint_t *setpoint, uint8_t type, const void *data, size_t datalen);

/* ---===== 1 - packetType_e enum =====--- */
enum packet_type {
  stopType          = 0,
  velocityWorldType = 1,
  zDistanceType     = 2,
  cppmEmuType       = 3,
  altHoldTypeOld    = 4,
  hoverType         = 5,
  newControllerType = 6,
  posSetType		= 7,
  altHoldType		= 8,
};

/* ---===== 2 - Decoding functions =====--- */
/* The setpoint structure is reinitialized to 0 before being passed to the
 * functions
 */

/* stopDecoder
 * Keeps setpoint to 0: stops the motors and fall
 */
static void stopDecoder(setpoint_t *setpoint, uint8_t type, const void *data, size_t datalen)
{
  return;
}


static void velocityDecoder(setpoint_t *setpoint, uint8_t type, const void *data, size_t datalen)
{
  const struct velocityPacket_s *values = data;

  ASSERT(datalen == sizeof(struct velocityPacket_s));

  setpoint->mode.x = modeVelocity;
  setpoint->mode.y = modeVelocity;
  setpoint->mode.z = modeVelocity;

  setpoint->velocity.x = values->vx;
  setpoint->velocity.y = values->vy;
  setpoint->velocity.z = values->vz;

  setpoint->mode.yaw = modeVelocity;

  setpoint->attitudeRate.yaw = values->yawrate; 
}


static void zDistanceDecoder(setpoint_t *setpoint, uint8_t type, const void *data, size_t datalen)
{
  const struct zDistancePacket_s *values = data;

  ASSERT(datalen == sizeof(struct velocityPacket_s));


  setpoint->mode.z = modeAbs;

  setpoint->position.z = values->zDistance;


  setpoint->mode.yaw = modeVelocity;

  setpoint->attitudeRate.yaw = values->yawrate;


  setpoint->mode.roll = modeAbs;
  setpoint->mode.pitch = modeAbs;

  setpoint->attitude.roll = values->roll;
  setpoint->attitude.pitch = values->pitch; 
}

/* cppmEmuDecoder
 * CRTP packet containing an emulation of CPPM channels
 * Channels have a range of 1000-2000 with a midpoint of 1500
 * Supports the ordinary RPYT channels plus up to MAX_AUX_RC_CHANNELS auxiliary channels.
 * Auxiliary channels are optional and transmitters do not have to transmit all the data
 * unless a given channel is actually in use (numAuxChannels must be set accordingly)
 *
 * Current aux channel assignments:
 * - AuxChannel0: set high to enable self-leveling, low to disable
 */

static float s_CppmEmuRollMaxRateDps = 720.0f; // For rate mode
static float s_CppmEmuPitchMaxRateDps = 720.0f; // For rate mode
static float s_CppmEmuRollMaxAngleDeg = 50.0f; // For level mode
static float s_CppmEmuPitchMaxAngleDeg = 50.0f; // For level mode
static float s_CppmEmuYawMaxRateDps = 400.0f; // Used regardless of flight mode



static inline float getChannelUnitMultiplier(uint16_t channelValue, uint16_t channelMidpoint, uint16_t channelRange)
{
  // Compute a float from -1 to 1 based on the RC channel value, midpoint, and total range magnitude
  return ((float)channelValue - (float)channelMidpoint) / (float)channelRange;
}

static void cppmEmuDecoder(setpoint_t *setpoint, uint8_t type, const void *data, size_t datalen)
{
  bool isSelfLevelEnabled = true;

  ASSERT(datalen >= 9); // minimum 9 bytes expected - 1byte header + four 2byte channels
  const struct cppmEmuPacket_s *values = data;
  ASSERT(datalen == 9 + (2*values->hdr.numAuxChannels)); // Total size is 9 + number of active aux channels

  // Aux channel 0 is reserved for enabling/disabling self-leveling
  // If it's in use, check and see if it's set and enable self-leveling.
  // If aux channel 0 is not in use, default to self-leveling enabled.
  isSelfLevelEnabled = !(values->hdr.numAuxChannels >= 1 && values->channelAux[0] < 1500);

  // Set the modes

  // Position is disabled
  setpoint->mode.x = modeDisable;
  setpoint->mode.y = modeDisable;
  setpoint->mode.z = modeDisable;

  // Yaw is always velocity
  setpoint->mode.yaw = modeVelocity;

  // Roll/Pitch mode is either velocity or abs based on isSelfLevelEnabled
  setpoint->mode.roll = isSelfLevelEnabled ? modeAbs : modeVelocity;
  setpoint->mode.pitch = isSelfLevelEnabled ? modeAbs : modeVelocity;

  // Rescale the CPPM values into angles to build the setpoint packet
  if(isSelfLevelEnabled)
  {
    setpoint->attitude.roll = -1 * getChannelUnitMultiplier(values->channelRoll, 1500, 500) * s_CppmEmuRollMaxAngleDeg; // roll inverted
    setpoint->attitude.pitch = -1 * getChannelUnitMultiplier(values->channelPitch, 1500, 500) * s_CppmEmuPitchMaxAngleDeg; // pitch inverted
  }
  else
  {
    setpoint->attitudeRate.roll = -1 * getChannelUnitMultiplier(values->channelRoll, 1500, 500) * s_CppmEmuRollMaxRateDps; // roll inverted
    setpoint->attitudeRate.pitch = -1 * getChannelUnitMultiplier(values->channelPitch, 1500, 500) * s_CppmEmuPitchMaxRateDps; // pitch inverted
  }

  setpoint->attitudeRate.yaw = -1 * getChannelUnitMultiplier(values->channelYaw, 1500, 500) * s_CppmEmuYawMaxRateDps; // yaw inverted
  setpoint->thrust = getChannelUnitMultiplier(values->channelThrust, 1000, 1000) * (float)UINT16_MAX; // Thrust is positive only - uses the full 1000-2000 range

  // Make sure thrust isn't negative
  if(setpoint->thrust < 0)
  {
    setpoint->thrust = 0;
  } 
}


static void altHoldDecoderOld(setpoint_t *setpoint, uint8_t type, const void *data, size_t datalen)
{
  const struct altHoldPacket_s *values = data;

  ASSERT(datalen == sizeof(struct velocityPacket_s));


  setpoint->mode.z = modeVelocity;

  setpoint->velocity.z = values->zVelocity;


  setpoint->mode.yaw = modeVelocity;

  setpoint->attitudeRate.yaw = values->yawrate;


  setpoint->mode.roll = modeAbs;
  setpoint->mode.pitch = modeAbs;

  setpoint->attitude.roll = values->roll;
  setpoint->attitude.pitch = values->pitch; 
}


static void hoverDecoder(setpoint_t *setpoint, uint8_t type, const void *data, size_t datalen)
{
  const struct hoverPacket_s *values = data;

  ASSERT(datalen == sizeof(struct velocityPacket_s));

  setpoint->mode.z = modeAbs;
  setpoint->position.z = values->zDistance;


  setpoint->mode.yaw = modeVelocity;
  setpoint->attitudeRate.yaw = values->yawrate;


  setpoint->mode.x = modeVelocity;
  setpoint->mode.y = modeVelocity;
  setpoint->velocity.x = values->vx;
  setpoint->velocity.y = values->vy;

  setpoint->velocity_body = true; 
}

/* newControllerDecoder
 * Set the dynamic waypoints for mike hammer's nonlinear controller
 */

static float cmd[3] = {0};

static void newControllerDecoder(setpoint_t *setpoint, uint8_t type, const void *data, size_t datalen)
{
  const struct newControllerPacket_s *values = data;
  cmd[0] =  half2single(values->x[0]);
  cmd[1] =  half2single(values->y[0]);
  cmd[2] =  half2single(values->z[0]);


  ASSERT(datalen == sizeof(struct newControllerPacket_s));
 
  setpoint->setEmergency = values->header.setEmergency;
  setpoint->resetEmergency = values->header.resetEmergency;

  setpoint->xmode = values->header.controlModeX;
  setpoint->ymode = values->header.controlModeY;
  setpoint->zmode = values->header.controlModeZ;

  for(int i = 0; i<3; i++){
    setpoint->x[i] = half2single(values->x[i]);
    setpoint->y[i] = half2single(values->y[i]);
    setpoint->z[i] = half2single(values->z[i]);
    if (i < 2)
      setpoint->yaw[i] = half2single(values->yaw[i]);
  } 
}

static void posSetDecoder(setpoint_t *setpoint, uint8_t idx, const void* data, size_t datalen)
{
	crtp_setpoint_t* d = ((crtp_setpoint_t*) data);
	float pos_z = position_fix24_to_float(d->pose[idx].z);

	if(pos_z > 0.005f){
		setpoint->mode.x = modeAbs;
		setpoint->mode.y = modeAbs;
		setpoint->mode.z = modeAbs;
		setpoint->mode.roll = modeDisable;
		setpoint->mode.pitch = modeDisable;
		setpoint->mode.yaw = modeAbs;

		setpoint->position.x = position_fix24_to_float(d->pose[idx].x);
		setpoint->position.y = position_fix24_to_float(d->pose[idx].y);
		setpoint->position.z = position_fix24_to_float(d->pose[idx].z);

		setpoint->attitude.roll  = 0;
		setpoint->attitude.pitch = 0;
		setpoint->attitude.yaw = position_fix24_to_float(d->pose[idx].yaw) / 3.1415926f * 180.0f;
		setpoint->thrust = 0;
	}
	else{
		// to lock thrust when set point is clost to group
		// This would work since in controller_pid.c,
		// if(sepoint->mode.z == modeDisabled)
		//   actuator thrust = setpoint->thrust = 0;
		setpoint->mode.z = modeDisable;
		setpoint->thrust = 0.0f;
	}


}

static void altHoldDecoder(setpoint_t *setpoint, uint8_t idx, const void* data, size_t datalen)
{
	crtp_setpoint_t* d = ((crtp_setpoint_t*) data);

	setpoint->mode.x = modeDisable;
	setpoint->mode.y = modeDisable;
    setpoint->mode.z = modeDisable;

    setpoint->thrust = position_fix24_to_float(d->pose[idx].z) * 65536;		// pwm

    setpoint->mode.yaw = modeAbs;
    setpoint->attitudeRate.yaw = position_fix24_to_float(d->pose[idx].yaw) / 3.1415926f * 180.0f; //deg

    setpoint->mode.roll = modeAbs;
    setpoint->mode.pitch = modeAbs;

    // pid controller convention
    // x - forward
    // y - opposite to vicon y
    // z - upward
    setpoint->attitude.roll = position_fix24_to_float(d->pose[idx].x) / 3.1415926f * 180.0f;  //deg
    setpoint->attitude.pitch = -position_fix24_to_float(d->pose[idx].y) / 3.1415926f * 180.0f; //deg

}


 /* ---===== 3 - packetDecoders array =====--- */
const static packetDecoder_t packetDecoders[] = {
  [stopType]          = stopDecoder,
  [velocityWorldType] = velocityDecoder,
  [zDistanceType]     = zDistanceDecoder,
  [cppmEmuType]       = cppmEmuDecoder,
  [altHoldTypeOld]    = altHoldDecoderOld,
  [hoverType]         = hoverDecoder,
  [newControllerType] = newControllerDecoder,
  [posSetType]		  = posSetDecoder,
  [altHoldType]       = altHoldDecoder
};

/* Decoder switch */
void crtpCommanderGenericDecodeSetpoint(setpoint_t *setpoint, CRTPPacket *pk)
{
  // The first ybte in pk->data is the CF's address (aka. broadcast id)
  // The second byte is the packet's type
  static int nTypes = -1;

  ASSERT(pk->size > 0);

  if (nTypes<0) {
    nTypes = sizeof(packetDecoders)/sizeof(packetDecoders[0]);
  }

  uint8_t type = pk->data[1]; 
  // should not set zero
  // there is no guarantee that setpoint will be set properly
//  memset(setpoint, 0, sizeof(setpoint_t));

  if (type<nTypes && (packetDecoders[type] != NULL)) {
    packetDecoders[type](setpoint, type, ((char*)pk->data)+2, pk->size-2);
  } 
}

void bccrtpCommanderGenericDecodeSetpoint(setpoint_t *setpoint, crtp_setpoint_t* data, uint8_t idx)
{
	  // The first ybte in pk->data is the CF's address (aka. broadcast id)
	  // The second byte is the packet's type
	  static int nTypes = -1;

//	  ASSERT(pk->size > 0);

	  if (nTypes<0) {
	    nTypes = sizeof(packetDecoders)/sizeof(packetDecoders[0]);
	  }

	  uint8_t type = data->pose[idx].type;

	  if (type<nTypes && (packetDecoders[type] != NULL)) {
	    packetDecoders[type](setpoint, idx, data, 0);
	  }
}

// Params for generic CRTP handlers

// CPPM Emulation commander
PARAM_GROUP_START(cmdrCPPM)
PARAM_ADD(PARAM_FLOAT, rateRoll, &s_CppmEmuRollMaxRateDps)
PARAM_ADD(PARAM_FLOAT, ratePitch, &s_CppmEmuPitchMaxRateDps)
PARAM_ADD(PARAM_FLOAT, rateYaw, &s_CppmEmuYawMaxRateDps)
PARAM_ADD(PARAM_FLOAT, angRoll, &s_CppmEmuRollMaxAngleDeg)
PARAM_ADD(PARAM_FLOAT, angPitch, &s_CppmEmuPitchMaxAngleDeg)
PARAM_GROUP_STOP(cmdrCPPM)

//LOG_GROUP_START(crtpSetpoint)
//LOG_ADD(LOG_FLOAT, cmdX, &cmd[0])
//LOG_ADD(LOG_FLOAT, cmdY, &cmd[1])
//LOG_ADD(LOG_FLOAT, cmdZ, &cmd[2])
//LOG_GROUP_STOP(crtpSetpoint)
