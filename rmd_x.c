#include "rmd_x.h" 


#define CAN_RX_PIN 16
#define CAN_TX_PIN 17


int8_t rmd_read_pid_data(uint32_t id, rmd_status_t *rmd_h, int8_t tx_only) {
	uint8_t data[8] = {0};

	data[0] = RMD_READ_PID_DATA;

	esp_err_t err = tx_twai_msg(id, data, &(rmd_h->error_count));
	if (err) {
		return err;
	}
	if (tx_only == RMD_WAIT_FOR_RESP) {
		err = rmd_receive_message(id, rmd_h, 5);
	}

	return err;

}

int8_t rmd_write_pid_data_to_ram(uint32_t id, rmd_status_t *rmd_h, int8_t tx_only) {
	uint8_t data[8] = {0};

	data[0] = RMD_WRITE_PID_TO_RAM;
	data[2] = rmd_h->anglePidKp;
	data[3] = rmd_h->anglePidKi;
	data[4] = rmd_h->speedPidKp;
	data[5] = rmd_h->speedPidKi;
	data[6] = rmd_h->iqPidKp;
	data[7] = rmd_h->iqPidKi;

	esp_err_t err = tx_twai_msg(id, data, &(rmd_h->error_count));
	if (err) {
		return err;
	}
	if (!tx_only) {
		err = rmd_receive_message(id, rmd_h, 5);
	}

	return err;
}

int8_t rmd_motor_stop(uint32_t id, rmd_status_t *rmd_h, int8_t tx_only) {
	uint8_t data[8] = {0};

	data[0] = RMD_MOTOR_STOP;
	//return tx_twai_msg(id, data);
	esp_err_t err = tx_twai_msg(id, data, &(rmd_h->error_count));
	if (err) {
		return err;
	}

	err = rmd_receive_message(id, rmd_h, 5);
	return err;
}

int8_t rmd_motor_off(uint32_t id, rmd_status_t *rmd_h, int8_t tx_only) {
	uint8_t data[8];

	data[0] = RMD_MOTOR_OFF;
	data[1] = 0x00;
	data[2] = 0x00;
	data[3] = 0x00;
	data[4] = 0x00;
	data[5] = 0x00;
	data[6] = 0x00;
	data[7] = 0x00;
	// return tx_twai_msg(id, data);
	esp_err_t err = tx_twai_msg(id, data, &(rmd_h->error_count));
	if (err) {
		return err;
	}

	err = rmd_receive_message(id, rmd_h, 5);
	return err;
}

int8_t rmd_write_encoder_offset(uint32_t id, rmd_status_t *rmd_h, uint16_t offset, int8_t tx_only) {
	uint8_t data[8] = {0};


	data[0] = RMD_WRITE_ENCODER_OFFSET;
	memcpy(&data[6], &offset, sizeof(uint16_t));
	
	// return tx_twai_msg(id, data);
	esp_err_t err = tx_twai_msg(id, data, &(rmd_h->error_count));
	if (err) {
		return err;
	}

	err = rmd_receive_message(id, rmd_h, 5);
	return err;

}

int8_t rmd_set_position(uint32_t id, rmd_status_t *rmd_h, int32_t new_position, int8_t tx_only) {
	
	if( abs(rmd_h->position_ref - new_position) < RMD_POS_DEADBAND) {
		return 0;
	} else {
		if (new_position > rmd_h->max_pos) {
			new_position = rmd_h->max_pos;
		} else if (new_position < rmd_h->min_pos) {
			new_position = rmd_h->min_pos;
		}

		uint8_t data[8] = {0};

		rmd_h->position_ref = new_position;

		data[0] = RMD_POSITION_CLOSED_LOOP_1;
		memcpy(&data[4], &new_position, sizeof(int32_t));
		
		/// return tx_twai_msg(id, data);
		esp_err_t err = tx_twai_msg(id, data, &(rmd_h->error_count));
		if (err) {
			return err;
		}

		err = rmd_receive_message(id, rmd_h, 5);
		return err;
	}
}

int8_t rmd_set_position_speedlim(uint32_t id, rmd_status_t *rmd_h, int32_t new_position, uint16_t speed_lim, int8_t tx_only) {
	uint8_t data[8] = {0};

	rmd_h->position_ref = new_position;

	data[0] = RMD_POSITION_CLOSED_LOOP_2;
	memcpy(&data[2], &speed_lim, sizeof(uint16_t));
	memcpy(&data[4], &new_position, sizeof(int32_t));
	
	// return tx_twai_msg(id, data);
	esp_err_t err = tx_twai_msg(id, data, &(rmd_h->error_count));
	if (err) {
		return err;
	}

	err = rmd_receive_message(id, rmd_h, 5);
	return err;
}

int8_t rmd_set_rotation(uint32_t id, rmd_status_t *rmd_h, uint16_t position, uint8_t dir, uint16_t speed_lim, int8_t tx_only) {
	uint8_t data[8] = {0};

	rmd_h->position_ref = position;

	data[0] = RMD_POSITION_CLOSED_LOOP_4;
	data[1] = dir;
	memcpy(&data[2], &speed_lim, sizeof(uint16_t));
	memcpy(&data[4], &position, sizeof(uint16_t));
	
	// return tx_twai_msg(id, data);
	esp_err_t err = tx_twai_msg(id, data, &(rmd_h->error_count));
	if (err) {
		return err;
	}

	err = rmd_receive_message(id, rmd_h, 5);
	return err;
}


int8_t rmd_get_position(uint32_t id, rmd_status_t *rmd_h, int8_t tx_only) {
	
	uint8_t data[8] = {0};

	data[0] = RMD_READ_MULTI_TURNS_ANGLE;
	
	twai_clear_receive_queue();
	esp_err_t err = tx_twai_msg(id, data, &(rmd_h->error_count));
	if (err) {
		return err;
	}

	err = rmd_receive_message(id, rmd_h, 5);
	return err;
	
}


int8_t rmd_set_speed(uint32_t id, rmd_status_t *rmd_h, int32_t new_speed, int8_t tx_only) {
	uint8_t data[8];

	rmd_h->speed_ref = new_speed;

	data[0] = RMD_SPEED_CLOSED_LOOP;
	memcpy(&data[4], &new_speed, sizeof(int32_t));


	// return tx_twai_msg(id, data);

	esp_err_t err = tx_twai_msg(id, data, &(rmd_h->error_count));
			if (err) {
			return err;
		}

		err = rmd_receive_message(id, rmd_h, 5);
		return err;

}

int8_t rmd_set_torque(uint32_t id, rmd_status_t *rmd_h, int16_t new_torque, int8_t tx_only) {
	uint8_t data[8] = {0};
	
	rmd_h->torque_ref = new_torque;

	data[0] = RMD_TORQUE_CLOSED_LOOP;
	memcpy(&data[4], &new_torque, sizeof(int16_t));

	// return tx_twai_msg(id, data);

	esp_err_t err = tx_twai_msg(id, data, &(rmd_h->error_count));
	if (err) {
		return err;
	}

	err = rmd_receive_message(id, rmd_h, 5);
	return err;
}


int8_t rmd_find_limits(uint32_t id, rmd_status_t *rmd_h, int16_t max_torque) {
	const uint32_t get_pos_wait = 100 / portTICK_PERIOD_MS;
	const int16_t min_speed = 2;
	const int16_t max_speed = 10000;
	const int32_t pos_thresh = 500;
	const int32_t clearance = 1000; // Set the maximum angle this much smaller
	const int low_speed_count_limit = 50;
	const int32_t max_search_angle = 6*180*100; // motor 0.01 deg/LSB, gear ratio 6:1

	int16_t torq = max_torque;
	// FIND MAX POS

	rmd_motor_off(id, rmd_h, RMD_WAIT_FOR_RESP);
	vTaskDelay(get_pos_wait);

	// Positive rotation (Counter clockwise) 
	rmd_set_torque(id, rmd_h, torq, RMD_WAIT_FOR_RESP);
	vTaskDelay(get_pos_wait);
	int low_speed_count = 0;
	do  { 
		rmd_set_torque(id, rmd_h, torq, RMD_WAIT_FOR_RESP);
		rmd_get_position(id, rmd_h, RMD_WAIT_FOR_RESP);
		vTaskDelay(get_pos_wait);
		if (rmd_h->speed < min_speed) {
			low_speed_count++;
		} else {
			low_speed_count=0;
		}
		if(rmd_h->multiturn_pos > max_search_angle ) {
			low_speed_count = (low_speed_count_limit+1);
		}
	} while (low_speed_count < low_speed_count_limit);

/*
	rmd_set_speed(id, rmd_h, 1800);	
	vTaskDelay(get_pos_wait);
	do  { 
		rmd_set_speed(id, rmd_h, 1800);
		rmd_get_position(id, rmd_h); printf("Current position: %lld\n", rmd_h->multiturn_pos);
		vTaskDelay(get_pos_wait);
	} while (rmd_h->torque < max_torque);
*/
	
	rmd_motor_stop(id, rmd_h, RMD_WAIT_FOR_RESP);
	rmd_h->max_pos = rmd_h->multiturn_pos;
	
	vTaskDelay(get_pos_wait*10);

	torq = max_torque;
	// Negative rotation (Clockwise)
	rmd_set_torque(id, rmd_h, -torq, RMD_WAIT_FOR_RESP);
	vTaskDelay(get_pos_wait);

	
	low_speed_count=0;
	do  { 
		rmd_set_torque(id, rmd_h, -torq, RMD_WAIT_FOR_RESP);
		rmd_get_position(id, rmd_h, RMD_WAIT_FOR_RESP);
		vTaskDelay(get_pos_wait);
		if (rmd_h->speed > -min_speed) {
			low_speed_count++;
		} else {
			low_speed_count=0;
		}
		if(rmd_h->multiturn_pos < -max_search_angle ) {
			low_speed_count = (low_speed_count_limit+1);
		}
	} while (low_speed_count < low_speed_count_limit);

/*
	rmd_set_speed(id, rmd_h, -1800);
	vTaskDelay(get_pos_wait);
	do  { 
		rmd_set_speed(id, rmd_h, -1800);
		rmd_get_position(id, rmd_h); printf("Current position: %lld\n", rmd_h->multiturn_pos);
		vTaskDelay(get_pos_wait);
	} while (rmd_h->torque > -max_torque);
*/

	rmd_motor_stop(id, rmd_h, RMD_WAIT_FOR_RESP);
	vTaskDelay(get_pos_wait);
	rmd_h->min_pos = rmd_h->multiturn_pos;
	
	vTaskDelay(get_pos_wait*5);


	rmd_h->center_pos = (rmd_h->max_pos + rmd_h->min_pos)/2; // Assuming symmetrical motion
	
	int32_t diff;
	do {
		diff = rmd_h->multiturn_pos - rmd_h->center_pos;
		rmd_set_speed(id, rmd_h, max_speed, RMD_WAIT_FOR_RESP);
		rmd_get_position(id, rmd_h, RMD_WAIT_FOR_RESP);
		vTaskDelay(10);
	} while (diff < -pos_thresh);

	rmd_set_position(id, rmd_h, rmd_h->center_pos, RMD_WAIT_FOR_RESP);
	//rmd_motor_off(id, rmd_h, RMD_WAIT_FOR_RESP);
	vTaskDelay(get_pos_wait);
	//rmd_get_position(id, rmd_h, RMD_WAIT_FOR_RESP);
	//rmd_set_position(id, rmd_h, rmd_h->multiturn_pos, RMD_WAIT_FOR_RESP);
	
	//rmd_h->center_pos = rmd_h->multiturn_pos;
	diff = rmd_h->max_pos - rmd_h->min_pos;

	rmd_h->min_pos = (rmd_h->center_pos - diff/2)  + clearance;
	rmd_h->max_pos = (rmd_h->center_pos + diff/2)  - clearance;
	

	vTaskDelay(get_pos_wait*10);
	return 0;
	
}

void conf_twai(uint8_t can_tx_pin, uint8_t can_rx_pin) {
    //twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(can_tx_pin, can_rx_pin, TWAI_MODE_NO_ACK);
	twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(can_tx_pin, can_rx_pin, TWAI_MODE_NORMAL);
    g_config.tx_queue_len = 5;
    g_config.rx_queue_len = 5;
	g_config.intr_flags = ESP_INTR_FLAG_LEVEL2;
    g_config.alerts_enabled = TWAI_ALERT_ALL;
    
    twai_timing_config_t t_config = TWAI_TIMING_CONFIG_1MBITS();
    twai_filter_config_t f_config = {.acceptance_code = 0, .acceptance_mask = 0xFFFFFFFF, .single_filter = true};

     //Install TWAI driver
    if (twai_driver_install(&g_config, &t_config, &f_config) == ESP_OK) {
        printf("Driver installed\n");
    } else {
        printf("Failed to install driver\n");
        return;
    }

    //Start TWAI driver
    if (twai_start() == ESP_OK) {
        printf("Driver started\n");
    } else {
        printf("Failed to start driver\n");
        return;
    }
}


int8_t can_read_alerts(void) {
    uint32_t alerts_triggered;
    twai_read_alerts(&alerts_triggered, pdMS_TO_TICKS(0));
    if(alerts_triggered & TWAI_ALERT_ABOVE_ERR_WARN) {
        //	
    }
    twai_status_info_t status_info;
    twai_get_status_info(&status_info);

	return 0; // TODO: This should return something meaningful
}




esp_err_t tx_twai_msg(uint32_t id, uint8_t *data, int32_t *err_counter) {
	// FIXME find a better solution to ensure memory is not freed before TX is complete
	const twai_message_t tx_msg  = {.data_length_code = 8, .identifier = id, .extd = 0}; 

	memcpy((void *)tx_msg.data, data, 8);
	
/*
	for (int i = 0; i < tx_msg.data_length_code; i++) {
		printf("TX Data byte %d = %d\n", i, msg.data[i]);
	}
*/


	//if (twai_transmit(&tx_message, pdMS_TO_TICKS(RMD_CAN_TX_TIMEOUT)) == ESP_OK) {
	esp_err_t err =  twai_transmit((const twai_message_t *) &tx_msg, pdMS_TO_TICKS(0));
	if (err != ESP_OK) {
		printf("Failed to queue message for transmission, err: %d\n", err);
		*err_counter = (*err_counter)+1;
	} else {
		*err_counter = 0;
	}
	
	if (*err_counter > CAN_ERR_RELOAD_LIM) {
		// Recover the CAN bus if errors have piled up
		printf("Initiating TWAI driver reinstall\n");
		*err_counter = 0;
		twai_driver_uninstall();
		vTaskDelay(10);
		conf_twai(CAN_TX_PIN, CAN_RX_PIN);
		twai_initiate_recovery();
		
		
	}
	return err;
}

esp_err_t rmd_receive_message(uint32_t id, rmd_status_t *rmd_h, uint32_t timeout) {
    twai_message_t rx_message;
    esp_err_t err = twai_receive(&rx_message, pdMS_TO_TICKS(timeout)); // FIXME Check if wait is blocking other threads or not

  	if (err == ESP_ERR_TIMEOUT) {
        return err;
    } else if (err != ESP_OK) {
        printf("Failed to receive message, ERR: %d\n", (int)err);
        return err;
    }   

    if (rx_message.extd) {
        printf("Message is in Extended Format\n");
		return -2;
    }
    	
    //printf("ID is %d\n", rx_message.identifier);
    
    if (!(rx_message.rtr)) {
		int8_t rmd_err = rmd_parse_response(rmd_h, rx_message.data);
		return rmd_err;
    } else {
        printf("Received remote frame\n");
        return -2;
    }
    
    return err;
}

int8_t rmd_parse_response(rmd_status_t *rmd_h, uint8_t *data) {
	uint8_t type = data[0];
	if (RMD_STD_RESP(type)) {
                rmd_h->motor_temp 	= data[1];
				rmd_h->torque 		= (int16_t)( (data[2]<<4) + (data[3]<<12) ) / 16;
				rmd_h->speed 		= (int16_t)( data[4] + (data[5]<<8));
				rmd_h->encoder_pos =  data[6] + (data[7]<<8);

				return 0;

        } else if (type == RMD_READ_MULTI_TURNS_ANGLE) {
			int64_t pos = (
                    ((int64_t) data[1]<<8)
                   +((int64_t) data[2]<<16)
                   +((int64_t) data[3]<<24)
                   +((int64_t) data[4]<<32)
                   +((int64_t) data[5]<<40)
                   +((int64_t) data[6]<<48) 
                   +((int64_t) data[7]<<56) 
               );
            
			pos /= 256; // Shift back, but ensure correct sign in any compiler. (Arithmetic vs Logical right shift)
			rmd_h->multiturn_pos = pos;
			return 0;
		} else if(RMD_PID_RESP(type)) {

			rmd_h->anglePidKp 	= data[2];
			rmd_h->anglePidKi 	= data[3];
			rmd_h->speedPidKp 	= data[4];
			rmd_h->speedPidKi 	= data[5];
			rmd_h->iqPidKp 		= data[6];
			rmd_h->iqPidKi 		= data[7];
			return 0;

		} else if (RMD_NULL_RESP(type)) {
			return 0;
		} else {
			return -3;
		}
}
