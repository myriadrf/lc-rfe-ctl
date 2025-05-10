#include "main.h"
#include "func_rf.h"
#include "test_modes.h"

////////////////////
// Test 1 - RX Path, LNA Bypass, 0dB attenuation
void test_mode_1() {
    // Set TDD pins
	// ext pin is on the connector on this board so can't set it. mcu pin drives OR gate
	// TRX_IO -> SDR_TX, SDR_RX -> RX_IN
    HAL_GPIO_WritePin(TDD_MCU_A_GPIO_Port, TDD_MCU_A_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(TDD_MCU_B_GPIO_Port, TDD_MCU_B_Pin, GPIO_PIN_SET);
    
    // Set LNA to bypass mode
    set_lna(RF_CH_A, LNA_BYPASS);
    set_lna(RF_CH_B, LNA_BYPASS);
    
    // Set SW2 control pin SDR_RX ->RX_IN
    HAL_GPIO_WritePin(PM_A_SW2_VCTL_GPIO_Port, PM_A_SW2_VCTL_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(PM_B_SW2_VCTL_GPIO_Port, PM_B_SW2_VCTL_Pin, GPIO_PIN_SET);
    
    // Set attenuation to 0dB
    set_rx_atten(RF_CH_A, 0.00);
    set_rx_atten(RF_CH_B, 0.00);
}

////////////////////
// Test 2 - RX Path, LNA Enable, 0dB attenuation
void test_mode_2() {
    // Set TDD pins
	// ext pin is on the connector on this board so can't set it. mcu pin drives OR gate
	// TRX_IO -> SDR_TX, SDR_RX -> RX_IN
    HAL_GPIO_WritePin(TDD_MCU_A_GPIO_Port, TDD_MCU_A_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(TDD_MCU_B_GPIO_Port, TDD_MCU_B_Pin, GPIO_PIN_SET);

    // Set LNA to bypass mode
    set_lna(RF_CH_A, LNA_ACTIVE);
    set_lna(RF_CH_B, LNA_ACTIVE);
    
    // Set SW2 control pin SDR_RX ->RX_IN
    HAL_GPIO_WritePin(PM_A_SW2_VCTL_GPIO_Port, PM_A_SW2_VCTL_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(PM_B_SW2_VCTL_GPIO_Port, PM_B_SW2_VCTL_Pin, GPIO_PIN_SET);
    
    // Set attenuation to 0dB
    set_rx_atten(RF_CH_A, 0.0);
    set_rx_atten(RF_CH_B, 0.0);
}

////////////////////
// Test 3 - TX Path, PA Bypass
void test_mode_3() {
    // Set TDD pins
	// ext pin is on the connector on this board so can't set it. mcu pin drives OR gate
	// TRX_IO -> SDR_TX, SDR_RX -> RX_IN
    HAL_GPIO_WritePin(TDD_MCU_A_GPIO_Port, TDD_MCU_A_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(TDD_MCU_B_GPIO_Port, TDD_MCU_B_Pin, GPIO_PIN_SET);
    
    // Set PA to bypass mode
    set_pa(RF_CH_A, PA_BYPASS);
    set_pa(RF_CH_B, PA_BYPASS);
    
    // Set SW1 control pins
    // SW1 to pos2, SDR_TX -> TX_OUT
    HAL_GPIO_WritePin(PM_A_SW1_V1_GPIO_Port, PM_A_SW1_V1_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(PM_A_SW1_V2_GPIO_Port, PM_A_SW1_V2_Pin, GPIO_PIN_RESET);

    HAL_GPIO_WritePin(PM_B_SW1_V1_GPIO_Port, PM_B_SW1_V1_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(PM_B_SW1_V2_GPIO_Port, PM_B_SW1_V2_Pin, GPIO_PIN_RESET);
}

////////////////////
// Test 4 - TX Path, PA Enable
void test_mode_4() {
    // Set TDD pins
	// ext pin is on the connector on this board so can't set it. mcu pin drives OR gate
	// TRX_IO -> SDR_TX, SDR_RX -> RX_IN
    HAL_GPIO_WritePin(TDD_MCU_A_GPIO_Port, TDD_MCU_A_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(TDD_MCU_B_GPIO_Port, TDD_MCU_B_Pin, GPIO_PIN_SET);
    
    // Set PA to bypass mode
    set_pa(RF_CH_A, PA_ACTIVE);
    set_pa(RF_CH_B, PA_ACTIVE);
    
    // Set SW1 control pins
    // SW1 to pos2, SDR_TX -> TX_OUT
    HAL_GPIO_WritePin(PM_A_SW1_V1_GPIO_Port, PM_A_SW1_V1_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(PM_A_SW1_V2_GPIO_Port, PM_A_SW1_V2_Pin, GPIO_PIN_RESET);

    HAL_GPIO_WritePin(PM_B_SW1_V1_GPIO_Port, PM_B_SW1_V1_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(PM_B_SW1_V2_GPIO_Port, PM_B_SW1_V2_Pin, GPIO_PIN_RESET);
}

////////////////////
// Test 5 - RX Path, variable attenuation
void test_mode_5() {
    double cur = 0.0;
    const double step = 0.25;
    const double max_atten = 31.75;
    
    // Set TDD pins
	// ext pin is on the connector on this board so can't set it. mcu pin drives OR gate
	// TRX_IO -> SDR_TX, SDR_RX -> RX_IN
    HAL_GPIO_WritePin(TDD_MCU_A_GPIO_Port, TDD_MCU_A_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(TDD_MCU_B_GPIO_Port, TDD_MCU_B_Pin, GPIO_PIN_SET);
    
    // Set LNA to bypass mode
    set_lna(RF_CH_A, LNA_BYPASS);
    set_lna(RF_CH_A, LNA_BYPASS);
    
    // Set SW2 control pin SDR_RX ->RX_IN
    HAL_GPIO_WritePin(PM_A_SW2_VCTL_GPIO_Port, PM_A_SW2_VCTL_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(PM_B_SW2_VCTL_GPIO_Port, PM_B_SW2_VCTL_Pin, GPIO_PIN_SET);
    
    // Loop through attenuation values
    while (cur <= max_atten) {
        set_rx_atten(RF_CH_A, cur);
        HAL_Delay(10);
        set_rx_atten(RF_CH_B, cur);
        HAL_Delay(100);  // 100ms delay
        cur += step;
    }
}
