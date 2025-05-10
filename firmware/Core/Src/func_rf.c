#include "stdio.h"
#include "stm32g0xx_hal.h"
#include "main.h"
#include "func_rf.h"
#include "util.h"

//////////////
///// RF Switches
//// Power Measurement
/// "SW1"
// CH A
rf_sw_sp3t IC904 = {
	.v1 = {PM_A_SW1_V1_GPIO_Port, PM_A_SW1_V1_Pin},
	.v2 = {PM_A_SW1_V2_GPIO_Port, PM_A_SW1_V2_Pin}
};
// CH B
rf_sw_sp3t IC1004 = {
	.v1 = {PM_B_SW1_V1_GPIO_Port, PM_B_SW1_V1_Pin},
	.v2 = {PM_B_SW1_V2_GPIO_Port, PM_B_SW1_V2_Pin}
};

/// "SW2"
// CH A
rf_sw_spdt IC905 = {
	.vctl = {PM_A_SW2_VCTL_GPIO_Port, PM_A_SW2_VCTL_Pin}
};
// CH B
rf_sw_spdt IC1005 = {
	.vctl = {PM_B_SW2_VCTL_GPIO_Port, PM_B_SW2_VCTL_Pin}
};

/// "SW3"
// CH A
rf_sw_spdt IC903 = {
	.vctl = {PM_A_SW3_VCTL_GPIO_Port, PM_A_SW3_VCTL_Pin}
};
// CH B
rf_sw_spdt IC1003 = {
	.vctl = {PM_B_SW3_VCTL_GPIO_Port, PM_B_SW3_VCTL_Pin}
};

//// TDD Switching
// CH A
rf_sw_spdt IC1501_1502 = {
	.vctl = {TDD_MCU_A_GPIO_Port, TDD_MCU_A_Pin}
};
// CH B
rf_sw_spdt IC1601_1602 = {
	.vctl = {TDD_MCU_B_GPIO_Port, TDD_MCU_B_Pin}
};


// SPI for the step attenuator
extern SPI_HandleTypeDef hspi3;

////////////////////////////////////////////////////////////////////////

void set_sw_pos_spdt(rf_sw_spdt sw, rf_switch_pos_t pos) {
/* SKY13286-359LF Truth Table
 * VCTL    RFC to J1        RFC to J2
 *  0    Insertion loss     Isolation
 *  1      Isolation      Insertion loss
 */
    switch (pos) {
		case SW_POS_NONE: // not valid for this part
			break;
        case SW_POS_1:    // RFC to J1
            HAL_GPIO_WritePin(sw.vctl.port, sw.vctl.pin, GPIO_PIN_RESET);
            break;
        case SW_POS_2:    // RFC to J2
            HAL_GPIO_WritePin(sw.vctl.port, sw.vctl.pin, GPIO_PIN_SET);
            break;
        case SW_POS_3:    // not valid for this part
            break;
    }
}

void set_sw_pos_sp3t(rf_sw_sp3t sw, rf_switch_pos_t pos) {
/*
 * SKY13588-460LF Truth Table1
 * V1  V2   State
 * 0   0    Shutdown
 * 1   0    RFC to J2
 * 0   1    RFC to J1
 * 1   1    RFC to J3
 */

    switch (pos) {
		case SW_POS_NONE:  // Shutdown
			HAL_GPIO_WritePin(sw.v1.port, sw.v1.pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(sw.v2.port, sw.v2.pin, GPIO_PIN_RESET);
			break;
        case SW_POS_1:     // RFC to J1
            HAL_GPIO_WritePin(sw.v1.port, sw.v1.pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(sw.v2.port, sw.v2.pin, GPIO_PIN_SET);
            break;
        case SW_POS_2:     // RFC to J2
            HAL_GPIO_WritePin(sw.v1.port, sw.v1.pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(sw.v2.port, sw.v2.pin, GPIO_PIN_RESET);
            break;
        case SW_POS_3:     // RFC to J3
            HAL_GPIO_WritePin(sw.v1.port, sw.v1.pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(sw.v2.port, sw.v2.pin, GPIO_PIN_SET);
            break;
    }
}

void set_lna(rf_channel_t channel, lna_state_t state) {
    // Set the LNA state for the specified channel.
    switch(channel) {
        case RF_CH_A:
            if (state == LNA_ACTIVE) {
                HAL_GPIO_WritePin(LNA_A_EN_GPIO_Port, LNA_A_EN_Pin, GPIO_PIN_RESET);
            } else {
                HAL_GPIO_WritePin(LNA_A_EN_GPIO_Port, LNA_A_EN_Pin, GPIO_PIN_SET);
            }
            break;
        case RF_CH_B:
            if (state == LNA_ACTIVE) {
                HAL_GPIO_WritePin(LNA_B_EN_GPIO_Port, LNA_B_EN_Pin, GPIO_PIN_RESET);
            } else {
                HAL_GPIO_WritePin(LNA_B_EN_GPIO_Port, LNA_B_EN_Pin, GPIO_PIN_SET);
            }
            break;
    }
}

void set_pa(rf_channel_t channel, pa_state_t state) {
    // Set the PA state for the specified channel.
    switch(channel) {
        case RF_CH_A:
            if (state == PA_ACTIVE) {
                HAL_GPIO_WritePin(PA_A_EN_GPIO_Port, PA_A_EN_Pin, GPIO_PIN_RESET);
            } else {
                HAL_GPIO_WritePin(PA_A_EN_GPIO_Port, PA_A_EN_Pin, GPIO_PIN_SET);
            }
            break;
        case RF_CH_B:
            if (state == PA_ACTIVE) {
                HAL_GPIO_WritePin(PA_B_EN_GPIO_Port, PA_B_EN_Pin, GPIO_PIN_RESET);
            } else {
                HAL_GPIO_WritePin(PA_B_EN_GPIO_Port, PA_B_EN_Pin, GPIO_PIN_SET);
            }
            break;
    }
}

void set_rx_atten(rf_channel_t channel, double atten_value) {
    // Set the RF attenuation value for the specified channel.

	const double step = 0.25;
	uint8_t atten_word, addr_word;

	// prep attenuation word
	// max attenuation value is 31.75, clip there
	if (atten_value > 31.75) {
		atten_value = 31.75;
	}

	atten_word = (uint8_t)(atten_value / step); // F1956 has fixed 0.25 attenuation steps
	atten_word &= 0b01111111; // make sure D7 is always 0

	// prep address word
	// atten a has A0 connected to GND
	// atten b has A0 connected to VCC
	switch(channel) {
		case RF_CH_A:
			addr_word = 0;
		break;
		case RF_CH_B:
			addr_word = 1;
		break;
	}

	// F1956 wants data LSB first, if SPI3 config is MSB first flip in software here
	//atten_word = reverse_bits(atten_word);
	//addr_word = reverse_bits(addr_word);

	HAL_GPIO_WritePin(ATTEN_LE_GPIO_Port, ATTEN_LE_Pin, GPIO_PIN_RESET);
	HAL_Delay(0.1);

	// Write 2 bytes
	uint8_t tx_buf[2] = {atten_word, addr_word};
	//HAL_SPI_Transmit(&hspi3, tx, sizeof(tx), HAL_MAX_DELAY);
	HAL_SPI_Transmit(&hspi3, tx_buf, sizeof(tx_buf), HAL_MAX_DELAY);
	HAL_Delay(0.1);

	// LE high to latch data
	// It is recommended that Latch enable be left high
	// when the device is not being programmed
	HAL_GPIO_WritePin(ATTEN_LE_GPIO_Port, ATTEN_LE_Pin, GPIO_PIN_SET);

}


void set_pm_config(rf_channel_t channel, power_meter_state_t state) {
    // Set the RF power meter state for the specified channel.

	rf_sw_sp3t sw1;
	rf_sw_spdt sw2;
	rf_sw_spdt sw3;

	switch (channel) {
		case RF_CH_A:
			sw1 = IC904;
			sw2 = IC905;
			sw3 = IC903;
		break;
		case RF_CH_B:
			sw1 = IC1004;
			sw2 = IC1005;
			sw3 = IC1003;
		break;
	}

	/* Possible states:
	 * SDR: Active and RF switches set to measure SDR Tx
	 * EXT: Active and RF switches set for external U.FL input (with SDR Tx routed to PA)
	 */
	switch (state) {
		case POWER_METER_OFF:
			// SW1: ICxx4 pos: J2
			set_sw_pos_sp3t(sw1, SW_POS_2);  // SDR_TX -> TX_OUT
			// SW2: ICxx5 pos: J2
			set_sw_pos_spdt(sw2, SW_POS_2);  // SDR_RX -> RX_IN
			// SW3: ICxx3 pos: don't care
			set_sw_pos_spdt(sw3, SW_POS_1);  // u.FL port (Ext. Pwr.) -> Power detector
		break;

		case POWER_METER_SDR_TX:
			// SW1: ICxx4 pos: J3
			set_sw_pos_sp3t(sw1, SW_POS_3); // SDR_TX -> SW1 RF2
			// SW2: ICxx5 pos: don't care
			set_sw_pos_spdt(sw2, SW_POS_2); // RX_IN -> SDR_RX
			// SW3: ICxx3 pos: J2
			set_sw_pos_spdt(sw3, SW_POS_2); // SDR_TX, from SW1 -> Power detector
		break;

		case POWER_METER_EXT:
			// SW1: ICxx4 pos: don't care (for power detector)
			set_sw_pos_sp3t(sw1, SW_POS_2); // SDR_TX -> TX_OUT
			// SW2: ICxx5 pos: don't care (for power detector)
			set_sw_pos_spdt(sw2, SW_POS_2); // RX_IN -> SDR_RX
			// SW3: ICxx3 pos: RF1
			set_sw_pos_spdt(sw3, SW_POS_1); // u.FL port (Ext. Pwr.) -> Power detector
		break;
	}

}

uint16_t get_power_level_raw_spi(rf_channel_t channel) {
    // Get the RF power level for the specified channel.
	// XXX: we can't use SPI on RevA1 boards due to a pin swap, see ERRATA.md

	GPIO_TypeDef* conv_gpio_port;
	GPIO_TypeDef* sck_gpio_port;
	GPIO_TypeDef* miso_gpio_port;
	uint16_t conv_gpio_pin;
	uint16_t sck_gpio_pin;
	uint16_t miso_pin;

	switch (channel) {
		case RF_CH_A:
			conv_gpio_port = PM_A_CONV_GPIO_Port;
			conv_gpio_pin = PM_A_CONV_Pin;
			sck_gpio_port = SPI1_SCK_PM_A_SCK_GPIO_Port;
			sck_gpio_pin = SPI1_SCK_PM_A_SCK_Pin;
			miso_gpio_port = SPI1_MISO_PM_A_SDO_GPIO_Port;
			miso_pin = SPI1_MISO_PM_A_SDO_Pin;
		break;
		case RF_CH_B:
			conv_gpio_port = PM_B_CONV_GPIO_Port;
			conv_gpio_pin = PM_B_CONV_Pin;
			sck_gpio_port = SPI2_SCK_PM_B_SCK_GPIO_Port;
			sck_gpio_pin = SPI2_SCK_PM_B_SCK_Pin;
			miso_gpio_port = SPI2_MISO_PM_B_SDO_GPIO_Port;
			miso_pin = SPI2_MISO_PM_B_SDO_Pin;
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

	// get the raw ADC value
	uint16_t adc_val = spi_rx[0] << 4 | spi_rx[1] >> 4;

	return adc_val;
}


/*
// Bitbanged implementation, workaround for RevA1 boards
uint16_t get_power_level_raw_bitbang(rf_channel_t channel) {
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

	//printf("spi_rx[0]: ");
	//_binp8(spi_rx[0]);
	//printf("\n\rspi_rx[1]: ");
	//_binp8(spi_rx[1]);
	//printf("\n\r");

	// get the raw ADC value
	uint16_t adc_val = spi_rx[0] << 4 | spi_rx[1] >> 4;

	return adc_val;
	//return adc_val * (2.048/4096); // Vref is 2.048V and the ADC is 12 bits
}
*/

void set_tdd_mode(rf_channel_t channel, rf_state_t state) {
    // Set the Transmit Inhibit state for the specified channel for TDD operation.
	// This is controlled either via the MCU or an external signal input

	// XXX: Currently we are just driving this from the MCU, we should determine
	// how we'd like to utilise the external signal, then implement it here.
	// Maybe we just make this be externally driven? TBD...

    rf_sw_spdt sw;

	switch (channel) {
		case RF_CH_A:
			sw = IC1501_1502;
		break;
		case RF_CH_B:
			sw = IC1601_1602;
		break;
	}

	switch (state) {
		case RF_ACTIVE:
			// TRX_IO connected to SDR_RX
			// SDR_TX isolated
			set_sw_pos_spdt(sw, SW_POS_1);
		break;

		case RF_INACTIVE:
			// TRX_IO connected to SDR_TX
			// SDR_RX connected to RX_IN
			set_sw_pos_spdt(sw, SW_POS_2);
		break;
	}
}

void set_tx_rx_loopback(rf_channel_t channel, rf_state_t state) {
    // Set the Transmit/Receive Loopback state for the specified channel.

	rf_sw_sp3t sw1;
	rf_sw_spdt sw2;
	rf_sw_spdt sw3;

	switch (channel) {
		case RF_CH_A:
			sw1 = IC904;
			sw2 = IC905;
			sw3 = IC903;
		break;
		case RF_CH_B:
			sw1 = IC1004;
			sw2 = IC1005;
			sw3 = IC1003;
		break;
	}

	switch (state) {
		case RF_ACTIVE:
			// SW1: ICxx4 pos: J1
			set_sw_pos_sp3t(sw1, SW_POS_1);  // SDR_TX -> SW2 J1
			// SW2: ICxx5 pos: RF1
			set_sw_pos_spdt(sw2, SW_POS_1);  // (SDR_TX, from SW2) -> SDR_RX
			// SW3: ICxx3 pos: don't care
			set_sw_pos_spdt(sw3, SW_POS_1);  // u.FL port (Ext. Pwr.) -> Power detector
		break;

		case RF_INACTIVE:
			// SW1: ICxx4 pos: J2 (or J3)
			set_sw_pos_sp3t(sw1, SW_POS_2);  // SDR_TX -> TX_OUT
			// SW2: ICxx5 pos: RF2
			set_sw_pos_spdt(sw2, SW_POS_2);  // RX_IN -> SDR_RX
			// SW3: ICxx3 pos: don't care
			set_sw_pos_spdt(sw3, SW_POS_1);  // u.FL port (Ext. Pwr.) -> Power detector
		break;
	}
}


void rf_reset(rf_channel_t channel) {

	set_rx_atten(channel, 0.00);

	set_lna(channel, RF_INACTIVE);

	set_pa(channel, RF_INACTIVE);

	HAL_Delay(100);

	set_tdd_mode(channel, RF_INACTIVE);

	set_tx_rx_loopback(channel, RF_INACTIVE);

}



/*
void rf_reset_manual(rf_channel_t channel) {
    // Reset the RF channel to its default state:
    //  - LNA inactive (bypass)
    //  - PA inactive (bypass)
    //  - RF attenuator 0dB (bypass)
    //  - RF power measurement OFF
    //  - Tx/Rx loopback inactive
    //  - Transmit inhibit inactive

	switch (channel) {

		case RF_CH_A:
			// Disable TDD: TRX_IO -> SDR_TX, SDR_RX -> RX_IN
		    HAL_GPIO_WritePin(TDD_MCU_A_GPIO_Port, TDD_MCU_A_Pin, GPIO_PIN_SET);

		    // Set LNA to bypass mode
		    set_lna(RF_CH_A, LNA_BYPASS);

		    // Set SW2 control pin SDR_RX ->RX_IN
		    HAL_GPIO_WritePin(PM_A_SW2_VCTL_GPIO_Port, PM_A_SW2_VCTL_Pin, GPIO_PIN_SET);

		    // Set attenuation to 0dB
		    set_rx_atten(RF_CH_A, 0.00);

		    // TX
		    // Set PA to bypass mode
		    set_pa(RF_CH_A, PA_BYPASS);

		    // PM block SW1 to Pos 2, SDR_TX -> TX_OUT
		    // XXXXX this degrades RX A IN -> RX A OUT HOW?!?!?!
		    //HAL_GPIO_WritePin(PM_A_SW1_V1_GPIO_Port, PM_A_SW1_V1_Pin, GPIO_PIN_SET);   // PB15
		    //HAL_GPIO_WritePin(PM_A_SW1_V2_GPIO_Port, PM_A_SW1_V2_Pin, GPIO_PIN_RESET); // PA8
		break;
		case RF_CH_B:
			// Disable TDD: TRX_IO -> SDR_TX, SDR_RX -> RX_IN
		    HAL_GPIO_WritePin(TDD_MCU_B_GPIO_Port, TDD_MCU_B_Pin, GPIO_PIN_SET);

		    // Set LNA to bypass mode
		    set_lna(RF_CH_B, LNA_BYPASS);

		    // Set SW2 control pin SDR_RX ->RX_IN
		    HAL_GPIO_WritePin(PM_B_SW2_VCTL_GPIO_Port, PM_B_SW2_VCTL_Pin, GPIO_PIN_SET);

		    // Set attenuation to 0dB
		    set_rx_atten(RF_CH_B, 0.00);

		    // TX
		    // Set PA to bypass mode
		    set_pa(RF_CH_B, PA_BYPASS);

		    // PM block SW1 to Pos 2, SDR_TX -> TX_OUT
		    //??????????
		    //HAL_GPIO_WritePin(PM_B_SW1_V1_GPIO_Port, PM_B_SW1_V1_Pin, GPIO_PIN_SET);
		    //HAL_GPIO_WritePin(PM_B_SW1_V2_GPIO_Port, PM_B_SW1_V2_Pin, GPIO_PIN_RESET);
		break;

	}
}
*/
