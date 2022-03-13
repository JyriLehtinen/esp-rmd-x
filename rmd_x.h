#ifndef __RMD_X_H__
#define __RMD_X_H__

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/twai.h"
#include <string.h>


#ifdef __cplusplus
extern "C" {
#endif

#define RMD_PROTOCOL_VERSION "1.62"

#define RMD_GEAR_RATIO 		6
#define RMD_POS_TO_RADS 	0.00002908882083333
#define RADS_TO_RMD_POS 	34377.467747131
#define RADS_TO_RMD_ENCODER 0.000015979
#define RMD_ENCODER_TO_RADS 62582.270102733
#define RMD_POS_DEADBAND 	60 // 0,1 degrees

#define CAN_ERR_RELOAD_LIM 20
#define RMD_CAN_TX_TIMEOUT 10
#define RMD_WAIT_FOR_RESP 1
#define RMD_TX_ONLY 0


// SN COMMAND NAME COMMAND DATA
#define RMD_READ_PID_DATA					0x30
#define RMD_WRITE_PID_TO_RAM 				0x31
#define RMD_WRITE_PID_TO_ROM 				0x32
#define RMD_READ_ACCELERATION_DATA 			0x33
#define RMD_WRITE_ACCELERATION_DATA_TO_RAM 	0x34
#define RMD_READ_ENCODER_DATA 				0x90
#define RMD_WRITE_ENCODER_OFFSET 			0x91
#define RMD_WRITE_ZERO_POSITION_TO_ROM		0x19
#define RMD_READ_MULTI_TURNS_ANGLE 			0x92
#define RMD_READ_SINGLE_CIRCLE_ANGLE 		0x94
#define RMD_READ_MOTOR_STATUS_1 			0x9A // Includes Error flag
#define RMD_CLEAR_MOTOR_ERROR_FLAG 			0x9B
#define RMD_READ_MOTOR_STATUS_2 			0x9C
#define RMD_READ_MOTOR_STATUS_3 			0x9D
#define RMD_MOTOR_OFF			 			0x80
#define RMD_MOTOR_STOP 						0x81
#define RMD_MOTOR_RESUME 					0x88
#define RMD_TORQUE_CLOSED_LOOP 				0xA1
#define RMD_SPEED_CLOSED_LOOP 				0xA2
#define RMD_POSITION_CLOSED_LOOP_1 			0xA3 // Set absolute position, rotation dir chosen automatically to shortest path
#define RMD_POSITION_CLOSED_LOOP_2 			0xA4 // Multi turn angle with max speed limit, dir chosen automatically
#define RMD_POSITION_CLOSED_LOOP_3 			0xA5 // Set absolute position & spin direction
#define RMD_POSITION_CLOSED_LOOP_4 			0xA6 // Set absolute position, spin direction & max speed

// Macro to avoid ugly if else or switch case
#define RMD_STD_RESP(X) ( X == RMD_READ_MOTOR_STATUS_2) \
                        || ( X == RMD_TORQUE_CLOSED_LOOP) \
                        || ( X == RMD_SPEED_CLOSED_LOOP) \
                        || ( X == RMD_POSITION_CLOSED_LOOP_1) \
                        || ( X == RMD_POSITION_CLOSED_LOOP_2) \
                        || ( X == RMD_POSITION_CLOSED_LOOP_3) \
                        || ( X == RMD_POSITION_CLOSED_LOOP_4)

#define RMD_NULL_RESP(X) ( X == RMD_MOTOR_OFF) \
						|| ( X == RMD_MOTOR_STOP) \
						|| ( X == RMD_MOTOR_RESUME)

#define RMD_PID_RESP(X) ( X == RMD_READ_PID_DATA ) \
						|| ( X == RMD_WRITE_PID_TO_RAM) \
						|| ( X == RMD_WRITE_PID_TO_ROM)




/*********************** VARIABLE TYPES *************************/


typedef struct {
	uint8_t anglePidKp;
	uint8_t anglePidKi;
	uint8_t speedPidKp;
	uint8_t speedPidKi;
	uint8_t iqPidKp;
	uint8_t iqPidKi;

	int8_t motor_temp; // Internal temp in C
	int16_t torque; // Motor current, -2048 to 2048 translates to -33 A to 33 A
	int16_t speed; // DPS, scaled by gear ratio 1:6
	uint16_t encoder_pos; // Encoder position, 16 bit for 360. Gear ratio is 1:6
	int64_t multiturn_pos; // Multiturn position, 0.01 deg / LSB, gear ratio is 1:6

	int16_t torque_ref;
	int32_t speed_ref;
	int32_t position_ref;

	int32_t max_pos;	// Multiturn, maximum CCW angle
	int32_t center_pos; // Center position
	int32_t min_pos; 	// Multiturn, maximum CW angle

	int32_t error_count;
} rmd_status_t;


/*********************** FUNCTION PROTOTYPES ********************/

int8_t rmd_read_pid_data(uint32_t id, rmd_status_t *rmd_h, int8_t tx_only);
int8_t rmd_write_pid_data_to_ram(uint32_t id, rmd_status_t *rmd_h, int8_t tx_only);
int8_t rmd_motor_stop(uint32_t id, rmd_status_t *rmd_h, int8_t tx_only);
int8_t rmd_motor_off(uint32_t id, rmd_status_t *rmd_h, int8_t tx_only);
int8_t rmd_set_position(uint32_t id, rmd_status_t *rmd_h, int32_t new_position, int8_t tx_only);
int8_t rmd_set_position_speedlim(uint32_t id, rmd_status_t *rmd_h, int32_t new_position, uint16_t speed_lim, int8_t tx_only);
int8_t rmd_set_rotation(uint32_t id, rmd_status_t *rmd_h, uint16_t position, uint8_t dir, uint16_t speed_lim, int8_t tx_only);
int8_t rmd_get_position(uint32_t id, rmd_status_t *rmd_h, int8_t tx_only);
int8_t rmd_set_speed(uint32_t id, rmd_status_t *rmd_h, int32_t new_speed, int8_t tx_only);
int8_t rmd_set_torque(uint32_t id, rmd_status_t *rmd_h, int16_t new_torque, int8_t tx_only);

int8_t rmd_find_limits(uint32_t id, rmd_status_t *rmd_h, int16_t max_torque);

void conf_twai(uint8_t can_tx_pin, uint8_t can_rx_pin);
int8_t can_read_alerts(void);

esp_err_t tx_twai_msg(uint32_t id, uint8_t *data, int32_t *err_counter);
esp_err_t rmd_receive_message(uint32_t id, rmd_status_t *rmd_h, uint32_t timeout);
int8_t rmd_parse_response(rmd_status_t *rmd_h, uint8_t *data);

#ifdef __cplusplus
}
#endif

#endif
