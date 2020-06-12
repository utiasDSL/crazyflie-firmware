/*
    lpsTdoa4Tag.c

    Created on : May,06,2020
        Author : Wenda Zhao
        Email  : wenda.zhao@robotics.utias.utoronto.ca
    TDoA4 firmware code
*/
#include <string.h>
#include <stdlib.h>   // [change] for rand() function 
#include "FreeRTOS.h"
#include "task.h"
#include "libdw1000.h"
#include "mac.h"
// #include "uwb.h"  // replace with locodeck.h
// #include "locodeck.h"  // in lpsTdoa4Tag.h
// #include "cfg.h"   // unknown function
// #include "lpp.h"  // handle lpp short packet, moved to lpsTdoa4Tag
#include "lpsTdoa4Tag.h"
#include "log.h"
#include "debug.h"
#include "estimator_kalman.h"
//[change]
#define TDOA4_RECEIVE_TIMEOUT 10000
// Packet formats
#define PACKET_TYPE_TDOA4 0x30                      // [Change]
#define LPP_HEADER 0
#define LPP_TYPE (LPP_HEADER + 1)
#define LPP_PAYLOAD (LPP_HEADER + 2)
// Useful constants
static const uint8_t base_address[] = {0,0,0,0,0,0,0xcf,0xbc};
// [New]: Gloabl variable for the mode of the Agent. The default is TDoA4 
static int MODE = 4;
// Anchor context
typedef struct {
  uint8_t id;
  bool isUsed;
  uint8_t seqNr;
  uint32_t rxTimeStamp;
  uint32_t txTimeStamp;
  uint16_t distance;
  uint32_t distanceUpdateTime;
  bool isDataGoodForTransmission;

  double clockCorrection;
  int clockCorrectionBucket;
} anchorContext_t;

// This context struct contains all the required global values of the algorithm
static struct ctx_s {
  int anchorId;
  // Information about latest transmitted packet
  uint8_t seqNr;
  uint32_t txTime; // In UWB clock ticks

  // Next transmit time in system clock ticks
  uint32_t nextTxTick;
  int averageTxDelay; // ms

  // List of ids to transmit in remote data section
  uint8_t remoteTxId[REMOTE_TX_MAX_COUNT];
  uint8_t remoteTxIdCount;

  // The list of anchors to transmit and store is updated at regular intervals
  uint32_t nextAnchorListUpdate;

  // Remote anchor data
  uint8_t anchorCtxLookup[ID_COUNT];
  anchorContext_t anchorCtx[ANCHOR_STORAGE_COUNT];
  uint8_t anchorRxCount[ID_COUNT];
} ctx;

//[Change]
static bool rangingOk;
// static tdoaEngineState_t engineState;
typedef struct {
  uint8_t type;
  uint8_t seq;
  uint32_t txTimeStamp;
  uint8_t remoteCount;
} __attribute__((packed)) rangePacketHeader3_t;

typedef struct {
  uint8_t id;
  uint8_t seq;
  uint32_t rxTimeStamp;
  uint16_t distance;
} __attribute__((packed)) remoteAnchorDataFull_t;

typedef struct {
  uint8_t id;
  uint8_t seq;
  uint32_t rxTimeStamp;
} __attribute__((packed)) remoteAnchorDataShort_t;

typedef struct {
  rangePacketHeader3_t header;
  uint8_t remoteAnchorData;
} __attribute__((packed)) rangePacket3_t;

// lppShortAnchorPos_s is defined in locodeck.h, here we define a new msg for TDoA4 
// [New] lpp packet (transmission data): limitation is 11 float num
struct lppShortAnchorPosition_s {
  float position[3];
  float quaternion[4];
  float imu[6];
} __attribute__((packed));
// [New] Define a struct containing the info of remote "anchor" --> agent
// global variable
static struct remoteAgentInfo_s{
    int remoteAgentID;           // source Agent 
    int destAgentID;             // destination Agent
    bool hasDistance;
    struct lppShortAnchorPosition_s Pose;
    double ranging;
}remoteAgentInfo;                //[re-design]
//----------------------------------------------------------------------------------------//
//[change]: log parameter
static int log_range;    // distance is uint16_t

/*--------------------------------------------------------------------*/
int switchAgentMode(){
    return MODE;
}
static anchorContext_t* getContext(uint8_t anchorId) {
  uint8_t slot = ctx.anchorCtxLookup[anchorId];

  if (slot == ID_WITHOUT_CONTEXT) {
    return 0;
  }

  return &ctx.anchorCtx[slot];
}

static void clearAnchorRxCount() {
  memset(&ctx.anchorRxCount, 0, ID_COUNT);
}

static void removeAnchorContextsNotInList(const uint8_t* id, const uint8_t count) {
  for (int i = 0; i < ANCHOR_STORAGE_COUNT; i++) {
    anchorContext_t* anchorCtx = &ctx.anchorCtx[i];
    if (anchorCtx->isUsed) {
      const uint8_t ctxId = anchorCtx->id;
      bool found = false;
      for (int j = 0; j < count; j++) {
        if (id[j] == ctxId) {
          found = true;
          break;
        }
      }

      if (!found) {
        ctx.anchorCtxLookup[ctxId] = ID_WITHOUT_CONTEXT;
        anchorCtx->isUsed = false;
      }
    }
  }
}

static void createAnchorContext(const uint8_t id) {
  if (ctx.anchorCtxLookup[id] != ID_WITHOUT_CONTEXT) {
    // Already has a context, we're done
    return;
  }

  for (uint8_t i = 0; i < ANCHOR_STORAGE_COUNT; i++) {
    anchorContext_t* anchorCtx = &ctx.anchorCtx[i];
    if (!anchorCtx->isUsed) {
      ctx.anchorCtxLookup[id] = i;

      memset(anchorCtx, 0, sizeof(anchorContext_t));
      anchorCtx->id = id;
      anchorCtx->isUsed = true;

      break;
    }
  }
}

static void createAnchorContextsInList(const uint8_t* id, const uint8_t count) {
  for (uint8_t i = 0; i < count; i++) {
    createAnchorContext(id[i]);
  }
}

static void purgeData() {
  uint32_t now = xTaskGetTickCount();
  uint32_t acceptedCreationTime = now - DISTANCE_VALIDITY_PERIOD;

  for (int i = 0; i < ANCHOR_STORAGE_COUNT; i++) {
    anchorContext_t* anchorCtx = &ctx.anchorCtx[i];
    if (anchorCtx->isUsed) {
      if (anchorCtx->distanceUpdateTime < acceptedCreationTime) {
        anchorCtx->distance = 0;

        anchorCtx->clockCorrection = 0.0;
        anchorCtx->clockCorrectionBucket = 0;
      }
    }
  }
}

// This function is called at regular intervals to update lists containing data
// about which anchors to store and add to outgoing messages. This
// update might take some time but this should not be a problem since the TX
// times are randomized anyway. The intention is that we could plug in clever
// algorithms here that optimizes which anchors to use.
static void updateAnchorLists() {
  // Randomize which anchors to use

  static uint8_t availableId[ID_COUNT];
  static bool availableUsed[ID_COUNT];
  memset(availableId, 0, sizeof(availableId));
  memset(availableUsed, 0, sizeof(availableUsed));
  int availableCount = 0;

  static uint8_t ctxts[ANCHOR_STORAGE_COUNT];
  memset(ctxts, 0, sizeof(ctxts));

  // Collect all anchors we have got a message from
  for (int i = 0; i < ID_COUNT; i++) {
    if (ctx.anchorRxCount[i] != 0) {
      availableId[availableCount++] = i;
    }
  }

  // Out of all anchors that we have received messages from, pick two
  // randomized subsets for storage and TX ids
  uint8_t remoteTXIdIndex = 0;
  uint8_t contextIndex = 0;
  for (int i = 0; i < ANCHOR_STORAGE_COUNT; i++) {
    int start = rand() % availableCount;
    // Scan forward until we find an anchor
    for (int j = start; j < (start + availableCount); j++) {
      const int index = j % availableCount;
      if (!availableUsed[index]) {

        const int id = availableId[index];
        if (remoteTXIdIndex < REMOTE_TX_MAX_COUNT) {
          ctx.remoteTxId[remoteTXIdIndex++] = id;
        }
        if (contextIndex < ANCHOR_STORAGE_COUNT) {
          ctxts[contextIndex++] = id;
        }

        availableUsed[index] = true;
        break;
      }
    }
  }

  removeAnchorContextsNotInList(ctxts, contextIndex);
  createAnchorContextsInList(ctxts, contextIndex);

  ctx.remoteTxIdCount = remoteTXIdIndex;

  clearAnchorRxCount();

  // Set the TX rate based on the number of transmitting anchors around us
  // Aim for 400 messages/s. Up to 8 anchors: 50 Hz / anchor
  float freq = SYSTEM_TX_FREQ / (availableCount + 1);
  if (freq > (float) ANCHOR_MAX_TX_FREQ) {  //[change]: add (float)
    freq = ANCHOR_MAX_TX_FREQ;
  }
  if (freq < (float) ANCHOR_MIN_TX_FREQ) { //[change]: add (float)
    freq = ANCHOR_MIN_TX_FREQ;
  }
  ctx.averageTxDelay = 1000.0f / freq;

  purgeData();
}

/* Adjust time for schedule transfer by DW1000 radio. Set 9 LSB to 0 and round up */
static void adjustTxRxTime(dwTime_t *time)
{
  time->full = (time->full & ~((1 << 9) - 1)) + (1 << 9);
}

static dwTime_t findTransmitTimeAsSoonAsPossible(dwDevice_t *dev)
{
  dwTime_t transmitTime = { .full = 0 };
  dwGetSystemTimestamp(dev, &transmitTime);

  // Add guard and preamble time
  transmitTime.full += TDMA_GUARD_LENGTH;
  transmitTime.full += PREAMBLE_LENGTH;

  // And some extra
  transmitTime.full += TDMA_EXTRA_LENGTH;

  // TODO krri Adding randomization on this level adds a long delay, is it worth it?
  // The randomization on OS level is quantized to 1 ms (tick time of the system)
  // Add a high res random to smooth it out
  // uint32_t r = rand();
  // uint32_t delay = r % TDMA_HIGH_RES_RAND;
  // transmitTime.full += delay;

  // DW1000 can only schedule time with 9 LSB at 0, adjust for it
  adjustTxRxTime(&transmitTime);

  return transmitTime;
}

static double calculateClockCorrection(anchorContext_t* anchorCtx, int remoteTxSeqNr, uint32_t remoteTx, uint32_t rx)
{
  double result = 0.0d;

  // Assigning to uint32_t truncates the diffs and takes care of wrapping clocks
  uint32_t tickCountRemote = remoteTx - anchorCtx->txTimeStamp;
  uint32_t tickCountLocal = rx - anchorCtx->rxTimeStamp;

  if (tickCountRemote != 0) {
    result = (double)tickCountLocal / (double)tickCountRemote;
  }

  return result;
}

static uint16_t calculateDistance(anchorContext_t* anchorCtx, int remoteRxSeqNr, uint32_t remoteTx, uint32_t remoteRx, uint32_t rx)
{
  // Check that the remote received seq nr is our latest tx seq nr
  if (remoteRxSeqNr == ctx.seqNr && anchorCtx->clockCorrection > 0.0d) {
    uint32_t localTime = rx - ctx.txTime;
    uint32_t remoteTime = (uint32_t)((double)(remoteTx - remoteRx) * anchorCtx->clockCorrection);
    uint32_t distance = (localTime - remoteTime) / 2;

    return distance & 0xfffful;
  } else {
    return 0;
  }
}

static bool extractFromPacket(const rangePacket3_t* rangePacket, uint32_t* remoteRx, uint8_t* remoteRxSeqNr) {
  const void* anchorDataPtr = &rangePacket->remoteAnchorData;
  // loop over all the remote agents' info
    for (uint8_t i = 0; i < rangePacket->header.remoteCount; i++) {
        remoteAnchorDataFull_t* anchorData = (remoteAnchorDataFull_t*)anchorDataPtr;
        // if the radio packet is sending to this agent --> to twr
        const uint8_t id = anchorData->id;
        if (id == ctx.anchorId) {
        *remoteRxSeqNr = anchorData->seq & 0x7f;
        *remoteRx = anchorData->rxTimeStamp;
        return true;
        }
        // else --> move the pointer away from the distance msg
        // currently the other agents distance msg is not used 
        bool hasDistance = ((anchorData->seq & 0x80) != 0);
        if (hasDistance) {
        anchorDataPtr += sizeof(remoteAnchorDataFull_t);
        } else {
        anchorDataPtr += sizeof(remoteAnchorDataShort_t);
        }
    }
    return false;
}

static void fillClockCorrectionBucket(anchorContext_t* anchorCtx) {
    if (anchorCtx->clockCorrectionBucket < CLOCK_CORRECTION_BUCKET_MAX) {
      anchorCtx->clockCorrectionBucket++;
    }
}

static bool emptyClockCorrectionBucket(anchorContext_t* anchorCtx) {
    if (anchorCtx->clockCorrectionBucket > 0) {
      anchorCtx->clockCorrectionBucket--;
      return false;
    }

    return true;
}

static bool updateClockCorrection(anchorContext_t* anchorCtx, double clockCorrection) {
  const double diff = clockCorrection - anchorCtx->clockCorrection;
  bool sampleIsAccepted = false;

  if (-CLOCK_CORRECTION_ACCEPTED_NOISE < diff && diff < CLOCK_CORRECTION_ACCEPTED_NOISE) {
    // LP filter
    anchorCtx->clockCorrection = anchorCtx->clockCorrection * (1.0d - CLOCK_CORRECTION_FILTER) + clockCorrection * CLOCK_CORRECTION_FILTER;

    fillClockCorrectionBucket(anchorCtx);
    sampleIsAccepted = true;
  } else {
    if (emptyClockCorrectionBucket(anchorCtx)) {
      if (CLOCK_CORRECTION_SPEC_MIN < clockCorrection && clockCorrection < CLOCK_CORRECTION_SPEC_MAX) {
        anchorCtx->clockCorrection = clockCorrection;
      }
    }
  }

  return sampleIsAccepted;
}


//[New]: get the LPP transmitted data
static void handleLppShortPacket(const uint8_t *data, const int length) {
  uint8_t type = data[0];
  if (type == LPP_SHORT_ANCHORPOS) {
    struct lppShortAnchorPosition_s *pose = (struct lppShortAnchorPosition_s*)&data[1];
    // printf("Position data is: (%f,%f,%f) \r\n", pos->x, pos->y, pos->z);
    // save and use the remote angent data
    remoteAgentInfo.Pose.position[0] = pose->position[0];
    remoteAgentInfo.Pose.position[1] = pose->position[1];
    remoteAgentInfo.Pose.position[2] = pose->position[2];
    remoteAgentInfo.Pose.quaternion[0] = pose->quaternion[0];
    remoteAgentInfo.Pose.quaternion[1] = pose->quaternion[1];
    remoteAgentInfo.Pose.quaternion[2] = pose->quaternion[2];
    remoteAgentInfo.Pose.quaternion[3] = pose->quaternion[3];
    }
}
// [New]
static void handleLppPacket(const int dataLength, int rangePacketLength, const packet_t* rxPacket) {
    const int32_t payloadLength = dataLength - MAC802154_HEADER_LENGTH;
    const int32_t startOfLppDataInPayload = rangePacketLength;
    const int32_t lppDataLength = payloadLength - startOfLppDataInPayload;
    const int32_t lppTypeInPayload = startOfLppDataInPayload + 1;
    //   printf("payloadLentgh is %d\r\n",(int)payloadLength);
    //   printf("startOfLppDataInPayload is %d\r\n",(int)startOfLppDataInPayload);
    if (lppDataLength > 0) {
        const uint8_t lppPacketHeader = rxPacket->payload[startOfLppDataInPayload];
        if (lppPacketHeader == LPP_HEADER_SHORT_PACKET) {
            const int32_t lppTypeAndPayloadLength = lppDataLength - 1;
            handleLppShortPacket(&rxPacket->payload[lppTypeInPayload], lppTypeAndPayloadLength);
      }
    }
}

// [New]: Update the remote agent info, also get the rangeDataLength --> for LPP packet
static int updateRemoteAgentData(const void* payload){
    const rangePacket3_t* packet = (rangePacket3_t*)payload;
    const void* anchorDataPtr = &packet->remoteAnchorData;
    // loop over all remote agent packe info, should save the remote agent data
    for(uint8_t i = 0; i<packet->header.remoteCount; i++){
        remoteAnchorDataFull_t* anchorData = (remoteAnchorDataFull_t*)anchorDataPtr;
        /* comment out unused value*/
        // uint8_t remoteId = anchorData->id;
        // int64_t remoteRxTime = anchorData->rxTimeStamp;
        // uint8_t remoteSeqNr = anchorData->seq & 0x7f;

        /* ----------------- store the remote agent info ----------------- */
        // if (isValidTimeStamp(remoteRxTime)) {
        //    tdoaStorageSetRemoteRxTime(anchorCtx, remoteId, remoteRxTime, remoteSeqNr);
        // }
        /* ----------------- --------------------------- ----------------- */
        bool hasDistance = ((anchorData->seq & 0x80) != 0);
        if (hasDistance) {
        /* comment out unused value*/
        // int64_t tof = anchorData->distance;

        /* ----------------- store the remote agent info ----------------- */
        // if (isValidTimeStamp(tof)) {
        //     tdoaStorageSetTimeOfFlight(anchorCtx, remoteId, tof);

        //     uint8_t anchorId = tdoaStorageGetId(anchorCtx);
        //     tdoaStats_t* stats = &engineState.stats;
        //     if (anchorId == stats->anchorId && remoteId == stats->remoteAnchorId) {
        //     stats->tof = (uint16_t)tof;
        //     }
        // }
        /* ----------------- --------------------------- ----------------- */
        anchorDataPtr += sizeof(remoteAnchorDataFull_t);
        } else {
        anchorDataPtr += sizeof(remoteAnchorDataShort_t);
        }
    }
    return (uint8_t*)anchorDataPtr - (uint8_t*)packet;
}

//[note]: get range data from message
static void handleRangePacket(const uint32_t rxTime, const packet_t* rxPacket, const int dataLength)
{
  //[change] packet code is slightly different 
  //     in CF: locoAddress_t sourceAddress =>  uint64_t sourceAddress
  // in anchor: uint8_t sourceAddress[8]
  // similar to destAddress
  const uint8_t remoteAnchorId = rxPacket->sourceAddress;
  ctx.anchorRxCount[remoteAnchorId]++;
  anchorContext_t* anchorCtx = getContext(remoteAnchorId);
  if (anchorCtx) {
    const rangePacket3_t* rangePacket = (rangePacket3_t *)rxPacket->payload;

    uint32_t remoteTx = rangePacket->header.txTimeStamp;
    uint8_t remoteTxSeqNr = rangePacket->header.seq;

    double clockCorrection = calculateClockCorrection(anchorCtx, remoteTxSeqNr, remoteTx, rxTime);
    if (updateClockCorrection(anchorCtx, clockCorrection)) {
      anchorCtx->isDataGoodForTransmission = true;

      uint32_t remoteRx = 0;
      uint8_t remoteRxSeqNr = 0;
      bool dataFound = extractFromPacket(rangePacket, &remoteRx, &remoteRxSeqNr);
      if (dataFound) {
        //[note]: here is the range distance data!
        uint16_t distance = calculateDistance(anchorCtx, remoteRxSeqNr, remoteTx, remoteRx, rxTime);
        // TODO krri Remove outliers in distances
        if (distance > MIN_TOF) {
          anchorCtx->distance = distance;
          anchorCtx->distanceUpdateTime = xTaskGetTickCount();
        //[note]: log range
        log_range = anchorCtx->distance;
        }
      }
    } else {
      anchorCtx->isDataGoodForTransmission = false;
    }
    // [change]
    rangingOk = anchorCtx->isDataGoodForTransmission;
    anchorCtx->rxTimeStamp = rxTime;
    anchorCtx->seqNr = remoteTxSeqNr;
    anchorCtx->txTimeStamp = remoteTx;
    // [New] get transmitted info. Position + quaternion
    int rangeDataLength = updateRemoteAgentData(rangePacket); 
    handleLppPacket(dataLength, rangeDataLength, rxPacket);
  }
}

//[New]: moved from lpp.c from anchor code
void lppHandleShortPacket(uint8_t *data, size_t length)
{
    if (length < 1) return;
    int type  = data[0];
    //   debug("Handling LPP short packet of type %02x, length %d\r\n", type, length);
    //   debug("Raw data: ");
    //   for (int i=0; i<length; i++) {
    //     debug("%02x ", data[i]);
    //   }
    //   debug("\r\n");
  switch(type) {
    case LPP_SHORT_ANCHOR_POSITION:
    {
      // not used now. do nothing
      break;
    }
    case LPP_SHORT_REBOOT:
    { // not used now. do nothing
      break;
    }
    case LPP_SHORT_MODE:
    { // used to switch Agent mode
      struct lppShortMode_s* modeInfo = (struct lppShortMode_s*)&data[1];

      // Set new mode
      if (modeInfo->mode == LPP_SHORT_MODE_TWR) {
        MODE = lpsMode_TWR;
      } else if (modeInfo->mode == LPP_SHORT_MODE_TDOA2) {
        MODE = lpsMode_TDoA2;
      } else if (modeInfo->mode == LPP_SHORT_MODE_TDOA3) {
        MODE = lpsMode_TDoA3;
      }else if (modeInfo->mode == LPP_SHORT_MODE_TDOA4) {
        MODE = lpsMode_TDoA4;
      }
      break;
    }
    case LPP_SHORT_UWB:
    { // not used for now. Do nothing
      break;
    }
    case LPP_SHORT_UWB_MODE:
    { // not used for now. Do nothing
      break;
    }
  }
}

//[note]: main function after receive an uwb message
static void handleRxPacket(dwDevice_t *dev)
{
  static packet_t rxPacket;
  dwTime_t rxTime = { .full = 0 };

  dwGetRawReceiveTimestamp(dev, &rxTime);
  dwCorrectTimestamp(dev, &rxTime);

  int dataLength = dwGetDataLength(dev);
  rxPacket.payload[0] = 0;
  dwGetData(dev, (uint8_t*)&rxPacket, dataLength);

  if (dataLength == 0) {
    return;
  }

  switch(rxPacket.payload[0]) {
  case PACKET_TYPE_TDOA4:       //[change]
    // DEBUG_PRINT("Received TDOA4 message \r\n");
    handleRangePacket(rxTime.low32, &rxPacket, dataLength);   //[note] get range
    break;
  case SHORT_LPP:  //[New]: use SHORT_LPP to change mode
    if (rxPacket.destAddress == ctx.anchorId) {  // the lpp is sent to this Agent 
      lppHandleShortPacket(&rxPacket.payload[1], dataLength - MAC802154_HEADER_LENGTH - 1);
    }
    break;
  default:
    // Do nothing
    break;
  }
}

static void setupRx(dwDevice_t *dev)
{
  dwNewReceive(dev);
  dwSetDefaults(dev);
  dwStartReceive(dev);
}

static int populateTxData(rangePacket3_t *rangePacket)
{
  // rangePacket->header.type already populated
  rangePacket->header.seq = ctx.seqNr;
  rangePacket->header.txTimeStamp = ctx.txTime;

  uint8_t remoteAnchorCount = 0;
  uint8_t* anchorDataPtr = &rangePacket->remoteAnchorData;
  for (uint8_t i = 0; i < ctx.remoteTxIdCount; i++) {
    remoteAnchorDataFull_t* anchorData = (remoteAnchorDataFull_t*) anchorDataPtr;

    uint8_t id = ctx.remoteTxId[i];
    anchorContext_t* anchorCtx = getContext(id);

    if (anchorCtx->isDataGoodForTransmission) {
      anchorData->id = id;
      anchorData->seq = anchorCtx->seqNr;
      anchorData->rxTimeStamp = anchorCtx->rxTimeStamp;

      if (anchorCtx->distance > 0) {
        anchorData->distance = anchorCtx->distance;
        anchorDataPtr += sizeof(remoteAnchorDataFull_t);
        anchorData->seq |= 0x80;
      } else {
        anchorDataPtr += sizeof(remoteAnchorDataShort_t);
      }

      remoteAnchorCount++;
    }
  }
  rangePacket->header.remoteCount = remoteAnchorCount;

  return (uint8_t*)anchorDataPtr - (uint8_t*)rangePacket;
}

// Set TX data in the radio TX buffer for sendind: sourceAddress, destAddress, LPP.position
static void setTxData(dwDevice_t *dev)
{
  static packet_t txPacket;
  static bool firstEntry = true;
  static int lppLength = 0;

  if (firstEntry) {
    MAC80215_PACKET_INIT(txPacket, MAC802154_TYPE_DATA);
    // [change]: add '&' in front of txPacket 
    memcpy(&txPacket.sourceAddress, base_address, 8);
    txPacket.sourceAddress = ctx.anchorId;
    memcpy(&txPacket.destAddress, base_address, 8);
    txPacket.destAddress = 0xff;
    txPacket.payload[0] = PACKET_TYPE_TDOA4;

    firstEntry = false;
  }
    int rangePacketSize = populateTxData((rangePacket3_t *)txPacket.payload);
    // LPP anchor position is currently sent in all packets
    txPacket.payload[rangePacketSize + LPP_HEADER] = SHORT_LPP;
    txPacket.payload[rangePacketSize + LPP_TYPE] = LPP_SHORT_ANCHOR_POSITION;

    struct lppShortAnchorPosition_s *pos = (struct lppShortAnchorPosition_s*) &txPacket.payload[rangePacketSize + LPP_PAYLOAD];
    // test with dummy positions: it works!
    float dummy_pos[3] = {2.0, 2.1, 2.2};
    float dummy_quater[4] = {2.01, 2.02, 2.03, 2.04};
    float dummy_imu[6] = {2.11, 2.22, 2.33, 2.44, 2.55, 2.66};
    memcpy(pos->position, dummy_pos, 3 * sizeof(float));
    memcpy(pos->quaternion, dummy_quater, 4 * sizeof(float));
    memcpy(pos->imu, dummy_imu, 6 * sizeof(float) );
    lppLength = 2 + sizeof(struct lppShortAnchorPosition_s);

  dwSetData(dev, (uint8_t*)&txPacket, MAC802154_HEADER_LENGTH + rangePacketSize + lppLength);
}

// Setup the radio to send a packet
static void setupTx(dwDevice_t *dev)
{
  dwTime_t txTime = findTransmitTimeAsSoonAsPossible(dev);
  ctx.txTime = txTime.low32;
  ctx.seqNr = (ctx.seqNr + 1) & 0x7f;

  setTxData(dev);

  dwNewTransmit(dev);
  dwSetDefaults(dev);
  dwSetTxRxTime(dev, txTime);

  dwStartTransmit(dev);
}

static uint32_t randomizeDelayToNextTx()
{
  const uint32_t interval = 10;

  uint32_t r = rand();
  uint32_t delay = ctx.averageTxDelay + r % interval - interval / 2;

  return delay;
}

static uint32_t startNextEvent(dwDevice_t *dev, uint32_t now)
{
  dwIdle(dev);

  if (ctx.nextTxTick < now) {
    uint32_t newDelay = randomizeDelayToNextTx();
    ctx.nextTxTick = now + M2T(newDelay);

    setupTx(dev);
  } else {
    setupRx(dev);
  }

  return ctx.nextTxTick - now;
}

//// [change]: not used now, comment out
// static void sendTdoaToEstimatorCallback(tdoaMeasurement_t* tdoaMeasurement) {
//   estimatorKalmanEnqueueTDOA(tdoaMeasurement);
// }

//------------------------------------------------------------------//
// Initialize/reset the agorithm
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
// [change]: in CF algorithm, the init only has dwDevice, don't have config
static void tdoa4Init(dwDevice_t *dev)
{
 
  dwSetReceiveWaitTimeout(dev, TDOA4_RECEIVE_TIMEOUT);

  dwCommitConfiguration(dev);

  rangingOk = false;
  // manually set the Agent ID
  ctx.anchorId = 2;   // initialize to be int 0
  ctx.seqNr = 0;
  ctx.txTime = 0;
  ctx.nextTxTick = 0;
  ctx.averageTxDelay = 1000.0 / ANCHOR_MAX_TX_FREQ;
  ctx.remoteTxIdCount = 0;
  ctx.nextAnchorListUpdate = 0;

  memset(&ctx.anchorCtxLookup, ID_WITHOUT_CONTEXT, ID_COUNT);
  for (int i = 0; i < ANCHOR_STORAGE_COUNT; i++) {
    ctx.anchorCtx[i].isUsed = false;
  }

  clearAnchorRxCount();

  srand(ctx.anchorId);
}
//------------------------------------------------------------------//

// Called for each DW radio event
static uint32_t tdoa4UwbEvent(dwDevice_t *dev, uwbEvent_t event)
{
  switch (event) {
    case eventPacketReceived: {
        handleRxPacket(dev);
      }
      break;
    default:
      // Nothing here
      break;
  }

  uint32_t now = xTaskGetTickCount();
  if (now > ctx.nextAnchorListUpdate) {
    updateAnchorLists();
    ctx.nextAnchorListUpdate = now + ANCHOR_LIST_UPDATE_INTERVAL;
  }

  uint32_t timeout_ms = startNextEvent(dev, now);
  return timeout_ms;
}

//-----------------Move the algorithm from lpstdoa3--------------------//
static bool isRangingOk()
{
  return rangingOk;
}

static bool getAnchorPosition(const uint8_t anchorId, point_t* position) {

    // a walk around. For relative ranging ability, we don't need anchor position info.

    return true;
}

// [Note]: How are these two functions called??
// [Change]: Move the updateAnchorLists() function from anchor code
static uint8_t getAnchorIdList(uint8_t unorderedAnchorList[], const int maxListSize) {
    // get the anchor id num that I received msg from.
    // dn not need to use the inputs: uint8_t unorderedAnchorList[], const int maxListSize
    static uint8_t availableId[ID_COUNT];
    static bool availableUsed[ID_COUNT];
    memset(availableId, 0, sizeof(availableId));
    memset(availableUsed, 0, sizeof(availableUsed));
    int availableCount = 0;

    static uint8_t ctxts[ANCHOR_STORAGE_COUNT];
    memset(ctxts, 0, sizeof(ctxts));

    // Collect all anchors we have got a message from
    for (int i = 0; i < ID_COUNT; i++) {
        if (ctx.anchorRxCount[i] != 0) {
        availableId[availableCount++] = i;
        }
    }

    return availableCount;
}

// [Change]: Move the updateAnchorLists() function from anchor code
static uint8_t getActiveAnchorIdList(uint8_t unorderedAnchorList[], const int maxListSize) {
    // get the anchor id num that I received msg from.
    static uint8_t availableId[ID_COUNT];
    static bool availableUsed[ID_COUNT];
    memset(availableId, 0, sizeof(availableId));
    memset(availableUsed, 0, sizeof(availableUsed));
    int availableCount = 0;

    static uint8_t ctxts[ANCHOR_STORAGE_COUNT];
    memset(ctxts, 0, sizeof(ctxts));

    // Collect all anchors we have got a message from
    for (int i = 0; i < ID_COUNT; i++) {
        if (ctx.anchorRxCount[i] != 0) {
        availableId[availableCount++] = i;
        }
    }
    
    uint8_t remoteTXIdIndex = 0;
    uint8_t contextIndex = 0;
    for (int i = 0; i < ANCHOR_STORAGE_COUNT; i++) {
        int start = rand() % availableCount;
        // Scan forward until we find an anchor
        for (int j = start; j < (start + availableCount); j++) {
            const int index = j % availableCount;
            if (!availableUsed[index]) {
                const int id = availableId[index];
                if (remoteTXIdIndex < REMOTE_TX_MAX_COUNT) {
                    ctx.remoteTxId[remoteTXIdIndex++] = id;
                    }
                if (contextIndex < ANCHOR_STORAGE_COUNT) {
                ctxts[contextIndex++] = id;
                }
                availableUsed[index] = true;
                break;
            }
        }
    }
   return contextIndex;
}

// [Note]: The implementation of algorithm on the anchor and on CF are different
// need to check tdoa engine on CF and uwb.c on lps-node-firmware 

uwbAlgorithm_t uwbTdoa4TagAlgorithm = { //[change]: the name changed
  .init = tdoa4Init,   // the config is changed in init func
  .onEvent = tdoa4UwbEvent,
//[change]: The following are needed
  .isRangingOk = isRangingOk,
  .getAnchorPosition = getAnchorPosition,
  .getAnchorIdList = getAnchorIdList,           // return the active id num: uint8_t
  .getActiveAnchorIdList = getActiveAnchorIdList,
};

//[note]: test for log parameter
LOG_GROUP_START(tdoa3)

LOG_ADD(LOG_INT16,Range, &log_range)
LOG_GROUP_STOP(tdoa3)




