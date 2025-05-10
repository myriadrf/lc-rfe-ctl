#ifndef TEST_MODES_H
#define TEST_MODES_H

/**
 * @brief Test 1 - RX Path, LNA Bypass, 0dB attenuation
 * 
 * Configures the RF frontend for RX path testing with:
 * - LNA in bypass mode
 * - 0dB attenuation
 * - TDD MCU pin enabled
 * - SW2 control enabled
 * 
 * Note: Check VNA ports - TX and RX has different directions!
 */
void test_mode_1();

/**
 * @brief Test 2 - RX Path, LNA Enable, 0dB attenuation
 * 
 * Configures the RF frontend for RX path testing with:
 * - LNA in active mode
 * - 0dB attenuation
 * - TDD MCU pin enabled
 * - SW2 control enabled
 * 
 * Note: Check VNA ports - TX and RX has different directions!
 */
void test_mode_2();

/**
 * @brief Test 3 - TX Path, PA Bypass
 * 
 * Configures the RF frontend for TX path testing with:
 * - PA in bypass mode
 * - TDD MCU pin enabled
 * - SW1 control pins configured (V1=high, V2=low)
 * 
 * Note: Check VNA ports - TX and RX has different directions!
 */
void test_mode_3();

/**
 * @brief Test 4 - TX Path, PA Enable
 * 
 * Configures the RF frontend for TX path testing with:
 * - PA in active mode
 * - TDD MCU pin enabled
 * - SW1 control pins configured (V1=high, V2=low)
 * 
 * Note: Check VNA ports - TX and RX has different directions!
 */
void test_mode_4();

/**
 * @brief Test 5 - RX Path, Variable Attenuation
 * 
 * Tests the step attenuator by sweeping through different attenuation levels:
 * - LNA in bypass mode
 * - TDD MCU pin enabled
 * - SW2 control enabled
 * - Attenuation swept from 0dB to 31.75dB in 0.25dB steps
 * - 100ms delay between steps
 * 
 * Note: Check SMA connectors!
 */
void test_mode_5();

#endif // TEST_MODES_H