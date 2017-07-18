/**
 * CRTP commander rpyt packet format
 */
struct CommanderCrtpLegacyValues
{
  float roll;       // deg
  float pitch;      // deg
  float yaw;        // deg
  uint16_t thrust;
} __attribute__((packed));

/* velocityDecoder
 * Set the Crazyflie velocity in the world coordinate system
 */
struct velocityPacket_s {
  float vx;        // m in the world frame of reference
  float vy;        // ...
  float vz;        // ...
  float yawrate;  // deg/s
} __attribute__((packed));

/* zDistanceDecoder
 * Set the Crazyflie absolute height and roll/pitch angles
 */
struct zDistancePacket_s {
  float roll;            // deg
  float pitch;           // ...
  float yawrate;         // deg/s
  float zDistance;        // m in the world frame of reference
} __attribute__((packed));

#define MAX_AUX_RC_CHANNELS 10

struct cppmEmuPacket_s {
  struct {
      uint8_t numAuxChannels : 4;   // Set to 0 through MAX_AUX_RC_CHANNELS
      uint8_t reserved : 4;
  } hdr;
  uint16_t channelRoll;
  uint16_t channelPitch;
  uint16_t channelYaw;
  uint16_t channelThrust;
  uint16_t channelAux[MAX_AUX_RC_CHANNELS];
} __attribute__((packed));

/* altHoldDecoder
 * Set the Crazyflie vertical velocity and roll/pitch angle
 */
struct altHoldPacket_s {
  float roll;            // rad
  float pitch;           // ...
  float yawrate;         // deg/s
  float zVelocity;       // m/s in the world frame of reference
} __attribute__((packed));

/* hoverDecoder
 * Set the Crazyflie absolute height and velocity in the body coordinate system
 */
struct hoverPacket_s {
  float vx;           // m/s in the body frame of reference
  float vy;           // ...
  float yawrate;      // deg/s
  float zDistance;    // m in the world frame of reference
} __attribute__((packed));
