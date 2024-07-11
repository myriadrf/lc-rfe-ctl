#include "stm32g0xx_hal.h"
#include "main.h"
#include "func_rf.h"

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

void setRxAttenuation(rf_channel_t channel, int value) {
    // Set the RF attenuation value for the specified channel.

    // XXX missing
}

void setRFPowerMeter(rf_channel_t channel, power_meter_state_t state) {
    // Set the RF power meter state for the specified channel.

    // XXX missing

}

int getRFPowerLevelRaw(rf_channel_t channel) {
    // Get the RF power level for the specified channel.

    // XXX missing
    return 0;
}

void setTxRxLoopback(rf_channel_t channel, rf_state_t state) {
    // Set the Transmit/Receive Loopback state for the specified channel.

    // XXX missing
}

void rf_reset(rf_channel_t channel) {
    // Reset the RF channel to its default state.

    // XXX missing
}
