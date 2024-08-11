#include "stdio.h"
#include "stm32g0xx_hal.h"
#include "main.h"
#include "func_rf.h"
#include "util.h"

// XXX this needs to be tidied up...

// RF Switches
//// Power Measurement
////// CH A
//////// "SW1"
rf_sw_sp3t IC904 = {
	.v1 = {GPIO_PM_A_CTRL_1_GPIO_Port, GPIO_PM_A_CTRL_1_Pin},
	.v2 = {GPIO_PM_A_CTRL_2_GPIO_Port, GPIO_PM_A_CTRL_2_Pin}
};
//////// "SW2"
rf_sw_spdt IC905 = {
	.vctl = {GPIO_PM_A_CTRL_3_GPIO_Port, GPIO_PM_A_CTRL_3_Pin}
};
//////// "SW3"
rf_sw_spdt IC903 = {
	.vctl = {GPIO_PM_A_CTRL_4_GPIO_Port, GPIO_PM_A_CTRL_4_Pin}
};

////// CH B
rf_sw_sp3t IC1004 = {
	.v1 = {GPIO_PM_B_CTRL_1_GPIO_Port, GPIO_PM_B_CTRL_1_Pin},
	.v2 = {GPIO_PM_B_CTRL_2_GPIO_Port, GPIO_PM_B_CTRL_2_Pin}
};
//////// "SW2"
rf_sw_spdt IC1005 = {
	.vctl = {GPIO_PM_B_CTRL_3_GPIO_Port, GPIO_PM_B_CTRL_3_Pin}
};
//////// "SW3"
rf_sw_spdt IC1003 = {
	.vctl = {GPIO_PM_B_CTRL_4_GPIO_Port, GPIO_PM_B_CTRL_4_Pin}
};

//// TDD Switching
////// CH A
rf_sw_spdt IC1501_1502 = {
	.vctl = {GPIO_TDD_CTRL_A_GPIO_Port, GPIO_TDD_CTRL_A_Pin}
};

////// CH B
rf_sw_spdt IC1601_1602 = {
	.vctl = {GPIO_TDD_CTRL_B_GPIO_Port, GPIO_TDD_CTRL_B_Pin}
};


// SPI for the step attenuator
extern SPI_HandleTypeDef hspi3;

////////////////////////////////////////////////////////////////////////

void set_sw_pos_spdt(rf_sw_spdt sw, rf_switch_pos_t pos) {
    switch (pos) {
        case SW_POS_1:
            HAL_GPIO_WritePin(sw.vctl.port, sw.vctl.pin, GPIO_PIN_RESET);
            break;
        case SW_POS_2:
            HAL_GPIO_WritePin(sw.vctl.port, sw.vctl.pin, GPIO_PIN_SET);
            break;
        case SW_POS_3:
        	break;
        case SW_POS_NONE:
        	break;
    }
}

void set_sw_pos_sp3t(rf_sw_sp3t sw, rf_switch_pos_t pos) {
    switch (pos) {
        case SW_POS_1:
            HAL_GPIO_WritePin(sw.v1.port, sw.v1.pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(sw.v2.port, sw.v2.pin, GPIO_PIN_SET);
            break;
        case SW_POS_2:
            HAL_GPIO_WritePin(sw.v1.port, sw.v1.pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(sw.v2.port, sw.v2.pin, GPIO_PIN_RESET);
            break;
        case SW_POS_3:
            HAL_GPIO_WritePin(sw.v1.port, sw.v1.pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(sw.v2.port, sw.v2.pin, GPIO_PIN_SET);
            break;
        case SW_POS_NONE:
            HAL_GPIO_WritePin(sw.v1.port, sw.v1.pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(sw.v2.port, sw.v2.pin, GPIO_PIN_RESET);
            break;
    }
}

void setLNA(rf_channel_t channel, lna_state_t state) {
    // Set the LNA state for the specified channel.
    switch(channel) {
        case RF_CH_A:
            if (state == LNA_ACTIVE) {
                HAL_GPIO_WritePin(GPIO_LNA_A_EN_GPIO_Port, GPIO_LNA_A_EN_Pin, GPIO_PIN_SET);
            } else {
                HAL_GPIO_WritePin(GPIO_LNA_A_EN_GPIO_Port, GPIO_LNA_A_EN_Pin, GPIO_PIN_RESET);
            }
            break;
        case RF_CH_B:
            if (state == LNA_ACTIVE) {
                HAL_GPIO_WritePin(GPIO_LNA_B_EN_GPIO_Port, GPIO_LNA_B_EN_Pin, GPIO_PIN_SET);
            } else {
                HAL_GPIO_WritePin(GPIO_LNA_B_EN_GPIO_Port, GPIO_LNA_B_EN_Pin, GPIO_PIN_RESET);
            }
            break;
        default:
            // Invalid channel, shouldn't get here...
            break;
    }
}

void setPA(rf_channel_t channel, pa_state_t state) {
    // Set the PA state for the specified channel.
    switch(channel) {
        case RF_CH_A:
            if (state == PA_ACTIVE) {
                HAL_GPIO_WritePin(GPIO_PA_A_EN_GPIO_Port, GPIO_PA_A_EN_Pin, GPIO_PIN_SET);
            } else {
                HAL_GPIO_WritePin(GPIO_PA_A_EN_GPIO_Port, GPIO_PA_A_EN_Pin, GPIO_PIN_RESET);
            }
            break;
        case RF_CH_B:
            if (state == PA_ACTIVE) {
                HAL_GPIO_WritePin(GPIO_PA_B_EN_GPIO_Port, GPIO_PA_B_EN_Pin, GPIO_PIN_SET);
            } else {
                HAL_GPIO_WritePin(GPIO_PA_B_EN_GPIO_Port, GPIO_PA_B_EN_Pin, GPIO_PIN_RESET);
            }
            break;
        default:
            // Invalid channel, shouldn't get here...
            break;
    }
}

void setTxInhibit(rf_channel_t channel, rf_state_t state) {
    // Set the Transmit Inhibit state for the specified channel.

    // XXX missing
}

void setRxAttenuation(rf_channel_t channel, int atten_value) {
    // Set the RF attenuation value for the specified channel.

	// prep attenuation word
	// max attenuation value is 31.75, clip there
	if (atten_value > 31.75) {
		atten_value = 31.75;
	}

	uint8_t atten_word = atten_value / 0.25; // F1956 has fixed 0.25 attenuation steps
	atten_word &= 0b01111111; // make sure D7 is always 0

	// prep address word
	// atten a has A0 connected to GND
	// atten b has A0 connected to VCC
	uint8_t addr_word;
	switch(channel) {
		case RF_CH_A:
			addr_word = 0;
		break;
		case RF_CH_B:
			addr_word = 1;
		break;
	}

	// Write 2 bytes
	uint8_t tx[2] = {atten_word, addr_word};
	HAL_SPI_Transmit(&hspi3, tx, sizeof(tx), HAL_MAX_DELAY);
	HAL_Delay(5);

	// Toggle LE pin
	HAL_GPIO_WritePin(GPIO_ATTEN_LE_GPIO_Port, GPIO_ATTEN_LE_Pin, GPIO_PIN_SET);
	HAL_Delay(5);
	HAL_GPIO_WritePin(GPIO_ATTEN_LE_GPIO_Port, GPIO_ATTEN_LE_Pin, GPIO_PIN_RESET);
	HAL_Delay(5);
}


void setRFPowerMeter(rf_channel_t channel, power_meter_state_t state) {
    // Set the RF power meter state for the specified channel.

	rf_sw_sp3t sw1;
	rf_sw_spdt sw2;
	rf_sw_spdt sw3;

	switch (channel) {
		case RF_CH_A:
			//printf("using CH A switches\n\r");
			sw1 = IC904;
			sw2 = IC905;
			sw3 = IC903;
		break;
		case RF_CH_B:
			//printf("using CH B switches\n\r");
			sw1 = IC1004;
			sw2 = IC1005;
			sw3 = IC1003;
		break;
	}

	/* Possible states:
	 * OFF: Inactive and SDR Tx is routed on to the PA
	 * SDR: Active and RF switches set to measure SDR Tx
	 * EXT: Active and RF switches set for external U.FL input (with SDR Tx routed to PA)
	 */
	switch (state) {
		case POWER_METER_OFF:
			// SW1: ICxx4 pos: J2
			set_sw_pos_sp3t(sw1, SW_POS_2);
			// SW2: ICxx5 pos: don't care
			set_sw_pos_spdt(sw2, SW_POS_1);
			// SW3: ICxx3 pos: don't care
			set_sw_pos_spdt(sw3, SW_POS_1);
		break;

		case POWER_METER_SDR:
			// SW1: ICxx4 pos: J3
			set_sw_pos_sp3t(sw1, SW_POS_3);
			// SW2: ICxx5 pos: don't care
			set_sw_pos_spdt(sw2, SW_POS_1);
			// SW3: ICxx3 pos: J2
			set_sw_pos_spdt(sw3, SW_POS_2);
		break;

		case POWER_METER_EXT:
			// SW1: ICxx4 pos: don't care
			set_sw_pos_sp3t(sw1, SW_POS_2);
			// SW2: ICxx5 pos: don't care
			set_sw_pos_spdt(sw2, SW_POS_1);
			// SW3: ICxx3 pos: RF1
			set_sw_pos_spdt(sw3, SW_POS_1);
		break;
	}

}



// XXX: we can't use SPI on RevA1 boards due to a pin swap, see ERRATA.md
uint16_t getRFPowerLevelRawSPI(rf_channel_t channel) {
    // Get the RF power level for the specified channel.

	GPIO_TypeDef* conv_gpio_port;
	GPIO_TypeDef* sck_gpio_port;
	GPIO_TypeDef* miso_gpio_port;
	uint16_t conv_gpio_pin;
	uint16_t sck_gpio_pin;
	uint16_t miso_pin;

	switch (channel) {
		case RF_CH_A:
			//printf("using CH A pins\n\r");
			conv_gpio_port = GPIO_PM_A_CONV_GPIO_Port;
			conv_gpio_pin = GPIO_PM_A_CONV_Pin;
			sck_gpio_port = GPIO_PM_A_SCK_GPIO_Port;
			sck_gpio_pin = GPIO_PM_A_SCK_Pin;
			miso_gpio_port = GPIO_PM_A_MISO_GPIO_Port;
			miso_pin = GPIO_PM_A_MISO_Pin;
		break;
		case RF_CH_B:
			//printf("using CH B pins\n\r");
			conv_gpio_port = GPIO_PM_B_CONV_GPIO_Port;
			conv_gpio_pin = GPIO_PM_B_CONV_Pin;
			sck_gpio_port = GPIO_PM_B_SCK_GPIO_Port;
			sck_gpio_pin = GPIO_PM_B_SCK_Pin;
			miso_gpio_port = GPIO_PM_B_MISO_GPIO_Port;
			miso_pin = GPIO_PM_B_MISO_Pin;
		break;
	}

	// Trigger a conversion
	HAL_GPIO_WritePin(conv_gpio_port, conv_gpio_pin, GPIO_PIN_SET);
	HAL_Delay(10);
	HAL_GPIO_WritePin(conv_gpio_port, conv_gpio_pin, GPIO_PIN_RESET);
	HAL_Delay(20);

	// Read 2 bytes
	uint8_t spi_rx[2] = {};
	for (int i=0; i<2; i++) {
		for (int j=0; j<8; j++) {
			// Assert CLK pin
			HAL_GPIO_WritePin(sck_gpio_port, sck_gpio_pin, GPIO_PIN_SET);
			HAL_Delay(10);
			// Read the MISO pin
			spi_rx[i] <<= 1;
			if (HAL_GPIO_ReadPin(miso_gpio_port, miso_pin) == GPIO_PIN_SET) {
				spi_rx[i] |= 0x01;
			}
			// Clear CLK pin
			HAL_GPIO_WritePin(sck_gpio_port, sck_gpio_pin, GPIO_PIN_RESET);
			HAL_Delay(10);
		}
    }

/*
	printf("spi_rx[0]: ");
	_binp8(spi_rx[0]);
	printf("\n\rspi_rx[1]: ");
	_binp8(spi_rx[1]);
	printf("\n\r");
*/
	// get the raw ADC value
	uint16_t adc_val = spi_rx[0] << 4 | spi_rx[1] >> 4;

	return adc_val;
	//return adc_val * (2.048/4096); // Vref is 2.048V and the ADC is 12 bits
}


// Bitbanged implementation, workaround for RevA1 boards
uint16_t getRFPowerLevelRawBitbang(rf_channel_t channel) {
    // Get the RF power level for the specified channel.

	GPIO_TypeDef * conv_gpio_port;
	uint16_t conv_gpio_pin;
	GPIO_TypeDef * sck_gpio_port;
	uint16_t sck_gpio_pin;
	GPIO_TypeDef * miso_gpio_port;
	uint16_t miso_pin;

	switch (channel) {
		case RF_CH_A:
			//printf("using ch a pins\n\r");
			conv_gpio_port = GPIO_PM_A_CONV_GPIO_Port;
			conv_gpio_pin = GPIO_PM_A_CONV_Pin;
			sck_gpio_port = GPIO_PM_A_SCK_GPIO_Port;
			sck_gpio_pin = GPIO_PM_A_SCK_Pin;
			miso_gpio_port = GPIO_PM_A_MISO_GPIO_Port;
			miso_pin = GPIO_PM_A_MISO_Pin;
		break;
		case RF_CH_B:
			//printf("using ch b pins\n\r");
			conv_gpio_port = GPIO_PM_B_CONV_GPIO_Port;
			conv_gpio_pin = GPIO_PM_B_CONV_Pin;
			sck_gpio_port = GPIO_PM_B_SCK_GPIO_Port;
			sck_gpio_pin = GPIO_PM_B_SCK_Pin;
			miso_gpio_port = GPIO_PM_B_MISO_GPIO_Port;
			miso_pin = GPIO_PM_B_MISO_Pin;
		break;
	}

	// Trigger a conversion
	HAL_GPIO_WritePin(conv_gpio_port, conv_gpio_pin, GPIO_PIN_SET);
	//HAL_Delay(1);
	HAL_GPIO_WritePin(conv_gpio_port, conv_gpio_pin, GPIO_PIN_RESET);
	//HAL_Delay(1);

	// Read 2 bytes
	uint8_t spi_rx[2] = {};
	for (int i=0; i<2; i++) {
		for (int j=0; j<8; j++) {
			// Assert CLK pin
			HAL_GPIO_WritePin(sck_gpio_port, sck_gpio_pin, GPIO_PIN_SET);
			//HAL_Delay(1);

			// Read the MISO pin
			spi_rx[i] <<= 1;
			if (HAL_GPIO_ReadPin(miso_gpio_port, miso_pin) == GPIO_PIN_SET) {
				spi_rx[i] |= 0x01;
			}

			// Clear CLK pin
			HAL_GPIO_WritePin(sck_gpio_port, sck_gpio_pin, GPIO_PIN_RESET);
			//HAL_Delay(1);
		}
    }

/*
	printf("spi_rx[0]: ");
	_binp8(spi_rx[0]);
	printf("\n\rspi_rx[1]: ");
	_binp8(spi_rx[1]);
	printf("\n\r");
*/

	// get the raw ADC value
	uint16_t adc_val = spi_rx[0] << 4 | spi_rx[1] >> 4;

	return adc_val;
	//return adc_val * (2.048/4096); // Vref is 2.048V and the ADC is 12 bits
}


void setTxRxLoopback(rf_channel_t channel, rf_state_t state) {
    // Set the Transmit/Receive Loopback state for the specified channel.

	rf_sw_sp3t sw1;
	rf_sw_spdt sw2;
	rf_sw_spdt sw3;

	switch (channel) {
		case RF_CH_A:
			printf("using CH A switches\n\r");
			sw1 = IC904;
			sw2 = IC905;
			sw3 = IC903;
		break;
		case RF_CH_B:
			printf("using CH B switches\n\r");
			sw1 = IC1004;
			sw2 = IC1005;
			sw3 = IC1003;
		break;
	}

	switch (state) {
		case RF_ACTIVE:
			// SW1: ICxx4 pos: J1
			set_sw_pos_sp3t(sw1, SW_POS_1);
			// SW2: ICxx5 pos: RF1
			set_sw_pos_spdt(sw2, SW_POS_1);
			// SW3: ICxx3 pos: RF1 (Don't care)
			set_sw_pos_spdt(sw3, SW_POS_1);
		break;

		case RF_INACTIVE:
			// SW1: ICxx4 pos: J2 (or J3)
			set_sw_pos_sp3t(sw1, SW_POS_2);
			// SW2: ICxx5 pos: RF2
			set_sw_pos_spdt(sw2, SW_POS_2);
			// SW3: ICxx3 pos: RF1 (Don't care)
			set_sw_pos_spdt(sw3, SW_POS_1);
		break;
	}
}

void rf_reset(rf_channel_t channel) {
    /* Reset the RF channel to its default state:
     *  - LNA inactive (bypass)
     *  - PA inactive (bypass)
     *  - RF attenuator 0dB (bypass)
     *  - RF power measurement OFF
     *  - Tx/Rx loopback inactive
     *  - Transmit inhibit inactive
	*/

    // XXX missing

}
