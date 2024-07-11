typedef enum { RF_CH_A, RF_CH_B } rf_channel_t;
typedef enum { LNA_BYPASS, LNA_ACTIVE } lna_state_t;
typedef enum { PA_BYPASS, PA_ACTIVE} pa_state_t;
typedef enum { POWER_METER_OFF, POWER_METER_SDR, POWER_METER_EXT } power_meter_state_t;
typedef enum { RF_INACTIVE, RF_ACTIVE } rf_state_t;

/**
 * @brief Set the LNA state for the specified channel.
 *
 * @param channel The channel number.
 * @param state The desired state ("active" or "bypassed").
 */
void setLNA(rf_channel_t channel, lna_state_t state);

/**
 * @brief Set the PA state for the specified channel.
 *
 * @param channel The channel number.
 * @param state The desired state ("active" or "bypassed").
 */
void setPA(rf_channel_t channel, pa_state_t state);

/**
 * @brief Set the Transmit Inhibit state for the specified channel.
 *
 * @param channel The channel number.
 * @param state The desired state ("active" or "inactive").
 */
void setTxInhibit(rf_channel_t channel, rf_state_t state);


/**
 * @brief Get the LNA state for the specified channel.
 *
 * @param channel The channel number.
 * @return The LNA state as a string ("active" or "bypassed").
 */
lna_state_t getLNA(rf_channel_t channel);

/**
 * @brief Set the RF attenuation value for the specified channel.
 *
 * @param channel The channel number.
 * @param value The desired attenuation value in dB.
 */
void setRxAttenuation(rf_channel_t channel, int value);

/**
 * @brief Set the RF power meter state for the specified channel.
 *
 * @param channel The channel number.
 * @param state The desired state ("OFF", "SDR", or "EXT").
 */
void setRFPowerMeter(rf_channel_t channel, power_meter_state_t state);

/**
 * @brief Get the RF power level for the specified channel.
 *
 * @param channel The channel number.
 * @return The RAW ADC reading for RF power level.
 */
int getRFPowerLevelRaw(rf_channel_t channel);

/**
 * @brief Set the Transmit/Receive Loopback state for the specified channel.
 *
 * @param channel The channel number.
 * @param state The desired state ("active" or "inactive").
 */
void setTxRxLoopback(rf_channel_t channel, rf_state_t state);

/**
 * @brief Reset everything to the default state.
 */
void rf_reset(rf_channel_t channel);
