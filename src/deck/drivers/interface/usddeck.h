#ifndef __USDDECK_H__
#define __USDDECK_H__

#include <stdint.h>
#include <stdbool.h>

#define USDLOG_ACC              (0x01)
#define USDLOG_GYRO             (0x02)
#define USDLOG_BARO             (0x04)
#define USDLOG_MAG              (0x08)
#define USDLOG_STABILIZER_POS   (0x10)
#define USDLOG_STABILIZER_VEL   (0x20)
#define USDLOG_STABILIZER_ATT   (0x40)
#define USDLOG_CONTROL_POS      (0x80)
#define USDLOG_CONTROL_VEL      (0x100)
#define USDLOG_CONTROL_ATT      (0x200)
#define USDLOG_VICON_POS        (0x400)
#define USDLOG_VICON_VEL        (0x800)
#define USDLOG_RANGE            (0x1000)

#define USDLOG_ACC_SIZE				3
#define USDLOG_GYRO_SIZE			3
#define USDLOG_BARO_SIZE			3
#define USDLOG_MAG_SIZE				3
#define USDLOG_STABILIZER_POS_SIZE	3
#define USDLOG_STABILIZER_VEL_SIZE	3
#define USDLOG_STABILIZER_ATT_SIZE	3
#define USDLOG_CONTROL_POS_SIZE		3
#define USDLOG_CONTROL_VEL_SIZE		3
#define USDLOG_CONTROL_ATT_SIZE		3
#define USDLOG_VICON_POS_SIZE		3
#define USDLOG_VICON_VEL_SIZE		3
#define USDLOG_CONTROL_THR_SIZE		1
#define USDLOG_COMM_FREQ_SIZE		2
#define	USDLOG_RANGE_SIZE			1

typedef struct usdLogDataPtr_s {
  uint32_t* tick;
  float* floats;
  int* ints;
} usdLogQueuePtr_t;

typedef struct usdLogConfig_s {
  char filename[13];
  uint16_t items;
  uint16_t frequency;
  uint8_t bufferSize;
  uint8_t floatSlots;
  uint8_t intSlots;
} usdLogConfig_t;

#define USD_WRITE(FILE, MESSAGE, BYTES, BYTES_WRITTEN, CRC_VALUE, CRC_FINALXOR, CRC_TABLE) \
  f_write(FILE, MESSAGE, BYTES, BYTES_WRITTEN); \
  CRC_VALUE = crcByByte(MESSAGE, BYTES, CRC_VALUE, CRC_FINALXOR, CRC_TABLE);

#endif //__USDDECK_H__
