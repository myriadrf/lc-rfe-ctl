#include "stm32g0xx_hal.h"
#include "main.h"

void led_on()
{
	HAL_GPIO_WritePin(GPIO_LED_GPIO_Port, GPIO_LED_Pin, GPIO_PIN_SET);
}

void led_off()
{
	HAL_GPIO_WritePin(GPIO_LED_GPIO_Port, GPIO_LED_Pin, GPIO_PIN_RESET);
}

void pwr_5v_on()
{
	HAL_GPIO_WritePin(GPIO_5V_EN_GPIO_Port, GPIO_5V_EN_Pin, GPIO_PIN_SET);
}

void pwr_5v_off()
{
	HAL_GPIO_WritePin(GPIO_5V_EN_GPIO_Port, GPIO_5V_EN_Pin, GPIO_PIN_RESET);
}

bool pwr_5v_pg()
{
	//power-good monitor output that asserts low if the FB voltage is not within the specified window thresholds.
	if(HAL_GPIO_ReadPin(GPIO_5V_PG_GPIO_Port, GPIO_5V_PG_Pin) == 1){
		return true;
	} else {
		return false;
	}
}

void pwr_12v_on()
{
	HAL_GPIO_WritePin(GPIO_12V_EN_GPIO_Port, GPIO_12V_EN_Pin, GPIO_PIN_SET);
}

void pwr_12v_off()
{
	HAL_GPIO_WritePin(GPIO_12V_EN_GPIO_Port, GPIO_12V_EN_Pin, GPIO_PIN_RESET);
}

bool pwr_12v_pg()
{
	//power-good monitor output that asserts low if the FB voltage is not within the specified window thresholds.
	if(HAL_GPIO_ReadPin(GPIO_12V_PG_GPIO_Port, GPIO_12V_PG_Pin) == 1){
		return true;
	} else {
		return false;
	}
}

void relay_port1_on()
{
	HAL_GPIO_WritePin(GPIO_PORT1_EN_GPIO_Port, GPIO_PORT1_EN_Pin, GPIO_PIN_SET);
}

void relay_port1_off()
{
	HAL_GPIO_WritePin(GPIO_PORT1_EN_GPIO_Port, GPIO_PORT1_EN_Pin, GPIO_PIN_RESET);
}

void relay_port2_on()
{
	HAL_GPIO_WritePin(GPIO_PORT2_EN_GPIO_Port, GPIO_PORT2_EN_Pin, GPIO_PIN_SET);
}

void relay_port2_off()
{
	HAL_GPIO_WritePin(GPIO_PORT2_EN_GPIO_Port, GPIO_PORT2_EN_Pin, GPIO_PIN_RESET);
}
