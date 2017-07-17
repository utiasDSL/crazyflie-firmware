#ifndef _CRTP_BROADCAST_H_
#define _CRTP_BROADCAST_H_

#include "stabilizer_types.h"
#include "packetdef.h"

struct CrtpExtPosition
{
  float x; // in m
  float y; // in m
  float z; // in m
} __attribute__((packed));

void broadcastSrvInit(void);
// Get the current position from the cache
bool getExtPositionBC(state_t *state);
#endif /* _CRTP_BROADCAST_H_ */
