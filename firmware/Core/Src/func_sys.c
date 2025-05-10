#include "stm32g0xx_hal.h"
#include "main.h"

void led_on()
{
	HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
}

void led_off()
{
	HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
}

void pwr_5v_on()
{
	HAL_GPIO_WritePin(PWR_5V_EN_GPIO_Port, PWR_5V_EN_Pin, GPIO_PIN_SET);
}

void pwr_5v_off()
{
	HAL_GPIO_WritePin(PWR_5V_EN_GPIO_Port, PWR_5V_EN_Pin, GPIO_PIN_RESET);
}

bool pwr_5v_pg()
{
	//power-good monitor output that asserts low if the FB voltage is not within the specified window thresholds.
	if(HAL_GPIO_ReadPin(PWR_5V_PG_GPIO_Port, PWR_5V_PG_Pin) == 1){
		return true;
	} else {
		return false;
	}
}

void pwr_12v_on()
{
	HAL_GPIO_WritePin(PWR_12V_EN_GPIO_Port, PWR_12V_EN_Pin, GPIO_PIN_SET);
}

void pwr_12v_off()
{
	HAL_GPIO_WritePin(PWR_12V_EN_GPIO_Port, PWR_12V_EN_Pin, GPIO_PIN_RESET);
}

bool pwr_12v_pg()
{
	//power-good monitor output that asserts low if the FB voltage is not within the specified window thresholds.
	if(HAL_GPIO_ReadPin(PWR_12V_PG_GPIO_Port, PWR_12V_PG_Pin) == 1){
		return true;
	} else {
		return false;
	}
}

void relay_port1_on()
{
	HAL_GPIO_WritePin(PWR_PORT1_EN_GPIO_Port, PWR_PORT1_EN_Pin, GPIO_PIN_SET);
}

void relay_port1_off()
{
	HAL_GPIO_WritePin(PWR_PORT1_EN_GPIO_Port, PWR_PORT1_EN_Pin, GPIO_PIN_RESET);
}

void relay_port2_on()
{
	HAL_GPIO_WritePin(PWR_PORT2_EN_GPIO_Port, PWR_PORT2_EN_Pin, GPIO_PIN_SET);
}

void relay_port2_off()
{
	HAL_GPIO_WritePin(PWR_PORT2_EN_GPIO_Port, PWR_PORT2_EN_Pin, GPIO_PIN_RESET);
}
