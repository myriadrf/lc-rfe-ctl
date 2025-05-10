typedef enum { RF_CH_A, RF_CH_B } rf_channel_t;
typedef enum { LNA_BYPASS, LNA_ACTIVE } lna_state_t;
typedef enum { PA_BYPASS, PA_ACTIVE} pa_state_t;
typedef enum { POWER_METER_OFF, POWER_METER_SDR_TX, POWER_METER_EXT } power_meter_state_t;
typedef enum { RF_INACTIVE, RF_ACTIVE } rf_state_t;
typedef enum { SW_POS_NONE, SW_POS_1, SW_POS_2, SW_POS_3 } rf_switch_pos_t;

typedef struct {
    GPIO_TypeDef* port;
    uint16_t pin;
} gpio;

typedef struct {
    gpio vctl;
} rf_sw_spdt;

typedef struct {
    gpio v1;
    gpio v2;
} rf_sw_sp3t;

/**
 * @brief Set the state of a single pole double throw (SPDT) switch.
 * 
 * @param sw The switch to control.
 * @param pos The desired switch position.
 */
void set_sw_pos_spdt(rf_sw_spdt sw, rf_switch_pos_t pos);

/**
 * @brief Set the state of a single pole triple throw (SP3T) switch.
 * 
 * @param sw The switch to control.
 * @param pos The desired switch position.
 */
void set_sw_pos_sp3t(rf_sw_sp3t sw, rf_switch_pos_t pos);

/**
 * @brief Set the LNA state for the specified channel.
 *
 * @param channel The channel number.
 * @param state The desired state ("active" or "bypassed").
 */
void set_lna(rf_channel_t channel, lna_state_t state);

/**
 * @brief Set the PA state for the specified channel.
 *
 * @param channel The channel number.
 * @param state The desired state ("active" or "bypassed").
 */
void set_pa(rf_channel_t channel, pa_state_t state);

/**
 * @brief Set the RF attenuation value for the specified channel.
 *
 * @param channel The channel number.
 * @param value The desired attenuation value in dB.
 */
void set_rx_atten(rf_channel_t channel, double atten_value);

/**
 * @brief Set the RF power meter state for the specified channel.
 *
 * @param channel The channel number.
 * @param state The desired state ("SDR", or "EXT").
 */
void set_pm_config(rf_channel_t channel, power_meter_state_t state);

/**
 * @brief Get the RF power level for the specified channel.
 *
 * @param channel The channel number.
 * @return The RAW ADC reading for RF power level.
 */
uint16_t get_power_level_raw_spi(rf_channel_t channel);
//uint16_t get_power_level_raw_bitbang(rf_channel_t channel);

/**
 * @brief Set the Transmit Inhibit state for the specified channel.
 *
 * @param channel The channel number.
 * @param state The desired state ("active" or "inactive").
 */
void set_tdd_mode(rf_channel_t channel, rf_state_t state);

/**
 * @brief Set the Transmit/Receive Loopback state for the specified channel.
 *
 * @param channel The channel number.
 * @param state The desired state ("active" or "inactive").
 */
void set_tx_rx_loopback(rf_channel_t channel, rf_state_t state);

/**
 * @brief Reset everything to the default state.
 */
void rf_reset(rf_channel_t channel);
