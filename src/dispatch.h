#ifndef _DISPATCH_H
#define _DISPATCH_H
#include <stdint.h>

namespace octopus
{

enum
{
  CAN_ID_SYNC                 = 0x100,
  CAN_ID_COMMAND_SENSOR_N     = 0x200,
  CAN_ID_TARGET_STARTER       = 0x300,
  CAN_ID_TARGET_ENDER         = 0x38C,
  CAN_ID_DIAGNOSTIC_1         = 0x3E1,
  CAN_ID_DIAGNOSTIC_2         = 0x3E2,
  CAN_ID_DIAGNOSTIC_3         = 0x3E3,
  CAN_ID_VERSIONS             = 0x3EF,
  CAN_ID_VALID_DETECTIONS     = 0X3FF,
  MODE_RESET                  = 0,
  MODE_LONG                   = 2,
  MODE_MID                    = 3,
  MODE_SHORT                  = 4,
  MODE_BLIND                  = 5    
};

typedef struct
{
  uint8_t sensor_2_mode : 4;
  uint8_t sensor_1_mode : 4;
  uint8_t sensor_4_mode : 4;
  uint8_t sensor_3_mode : 4;
  uint8_t data_channel_msb : 8;
  uint8_t : 4;
  uint8_t msg_counter : 4;
  uint8_t data_channel_lsb : 8;
  uint8_t host_yaw_msb : 8;
  uint8_t host_yaw_lsb : 4;
  uint8_t host_velocity_msb : 4;
  uint8_t host_velocity_lsb : 8;
} MsgSync; // 0x100

typedef struct
{
  uint8_t meas_page_select : 4;
  uint8_t msg_counter : 4;
  uint8_t data_channel_1_msb : 8;
  uint8_t data_channel_1_lsb : 8;
  uint8_t data_channel_2_msb : 8;
  uint8_t data_channel_2_lsb : 8;
  uint8_t sync_msg_content : 8;
  uint8_t : 8;
} MsgCommand; // 0x201, 0x202, 0x203, 0x204

typedef struct
{
  uint8_t range_msb : 8;
  uint8_t range_lsb : 8;
  uint8_t velocity_msb : 8;
  uint8_t velocity_lsb : 8;
  uint8_t bearing_msb : 7;
  uint8_t bearing_sign : 1;
  uint8_t bearing_lsb : 8;
  uint8_t snr : 7;
  uint8_t snr_sign: 1;
  uint8_t flags : 8;
} MsgTargets;  // Start from 0x300, max of 140 detections
// Blank tareget : (0, 0, 0, 0, 80, 0, 0, 0)

typedef struct
{
  uint8_t : 8;
  uint8_t : 8;
  uint8_t : 8;
  uint8_t : 8;
  uint8_t : 8;
  uint8_t sensor_nr : 4;
  uint8_t msg_counter : 4;
  uint8_t : 4;
  uint8_t diag_code_msb : 8;
  uint8_t diag_code_lsb : 8;
} MsgDiagnostics;  // 0x3E1, 0x3E2, 0x3E3, diagnostic code on 0x3E3.
// A diag code below 1000 indicates a serious fault.

typedef struct
{
  uint8_t sw_1 : 8;
  uint8_t sw_2 : 8;
  uint8_t sw_3 : 8;
  uint8_t sw_4 : 8;
  uint8_t sub : 8;
  uint8_t hw_1 : 8;
  uint8_t hw_2 : 8;
  uint8_t hw_3 : 8;
} MsgVersions;  // 0x3EF

typedef struct
{
  uint8_t valid_detections_msb : 8;
  uint8_t valid_detections_lsb : 8;
  uint8_t : 8;
  uint8_t : 8;
  uint8_t : 8;
  uint8_t : 8;
  uint8_t : 8;
  uint8_t : 8;
} MsgDetectionCount;  // 0x3FF



}
#endif 
