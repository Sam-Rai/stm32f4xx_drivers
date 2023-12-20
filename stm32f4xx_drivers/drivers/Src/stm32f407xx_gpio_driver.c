/*
 * stm32f407xx_gpio_driver.c
 *
 *  Created on: Dec 10, 2023
 *      Author: samra
 */


#include "stm32f407xx_gpio_driver.h"


/*
 * Peripheral Clock setup
 */
/***************************************************************************
 * @fn					-	GPIO_PeriClockControl
 *
 * @brief				- This function enables or disables peripheral clock for the given GIPO port
 *
 * @param[in}			- base address of the gpio peripheral
 * @param[in}			- ENABLE or DISABLE macros
 * @param[in}			-
 *
 * @return				- none
 *
 * @Note				- none
 */
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi){

	if(EnorDi == ENABLE){

		if(pGPIOx == GPIOA){
			GPIOA_PCLK_EN();
		}else if (pGPIOx == GPIOB){
			GPIOB_PCLK_EN();
		}else if (pGPIOx == GPIOC){
			GPIOC_PCLK_EN();
		}else if (pGPIOx == GPIOD){
			GPIOD_PCLK_EN();
		}else if (pGPIOx == GPIOE){
			GPIOE_PCLK_EN();
		}else if (pGPIOx == GPIOF){
			GPIOF_PCLK_EN();
		}else if (pGPIOx == GPIOG){
			GPIOG_PCLK_EN();
		}else if (pGPIOx == GPIOH){
			GPIOH_PCLK_EN();
		}else if (pGPIOx == GPIOI){
			GPIOI_PCLK_EN();
		}
	}else {
		if(pGPIOx == GPIOA){
				GPIOA_PCLK_DI();
			}else if (pGPIOx == GPIOB){
				GPIOB_PCLK_DI();
			}else if (pGPIOx == GPIOC){
				GPIOC_PCLK_DI();
			}else if (pGPIOx == GPIOD){
				GPIOD_PCLK_DI();
			}else if (pGPIOx == GPIOE){
				GPIOE_PCLK_DI();
			}else if (pGPIOx == GPIOF){
				GPIOF_PCLK_DI();
			}else if (pGPIOx == GPIOG){
				GPIOG_PCLK_DI();
			}else if (pGPIOx == GPIOH){
				GPIOH_PCLK_DI();
			}else if (pGPIOx == GPIOI){
				GPIOI_PCLK_DI();
			}
	}
}

/***************************************************************************
 * @fn					-
 *
 * @brief				-
 *
 * @param[in}			-
 * @param[in}			-
 * @param[in}			-
 *
 * @return				-
 *
 * @Note				-
 */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle)
{
	uint32_t temp = 0; //temp. register

	//1. configure the mode of gpio pin
	if(pGPIOHandle -> GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG)
	{
		//the non interrupt mode
		temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
		pGPIOHandle->pGPIOx->MODER &= ~( 0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);	//clearing
		pGPIOHandle->pGPIOx->MODER |= temp;	//setting
//		temp =0;
	} else
	{
		//this part will code later. (interrupt mode)
	}
	temp = 0;
	//2. configure the speed
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber ));
	pGPIOHandle->pGPIOx->OSPEEDR &= ~( 0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);	//clearing
	pGPIOHandle->pGPIOx->OSPEEDR |= temp;

	temp = 0;
	//3. configure the pupd seetings

	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl ));
	pGPIOHandle->pGPIOx->PUPDR &= ~( 0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);	//clearing
	pGPIOHandle->pGPIOx->PUPDR |= temp;

	temp = 0;
	//4. configure the optype
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType << pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType );
	pGPIOHandle->pGPIOx->OTYPER &= ~( 0x1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);	//clearing
	pGPIOHandle->pGPIOx->OTYPER |= temp;

	//5. confgure the alt functionality
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALTFN){

		//configure the alt function registers.
		uint8_t temp1, temp2;

		temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 8;
		temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 8;
		pGPIOHandle->pGPIOx->AFR[temp1] &= ~( 0xF << (4 * temp2));	//clearing
		pGPIOHandle->pGPIOx->AFR[temp1] |= (pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode << (4 * temp2));
	}

}



/*
 * Init and De-init
 */
/***************************************************************************
 * @fn					- GPIO_DeInit
 *
 * @brief				-
 *
 * @param[in}			-
 * @param[in}			-
 * @param[in}			-
 *
 * @return				-
 *
 * @Note				-
 */
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx)
{
	if(pGPIOx == GPIOA)
	{
		GPIOA_REG_RESET();
	}else if (pGPIOx == GPIOB)
	{
		GPIOA_REG_RESET();
	}else if (pGPIOx == GPIOC)
	{
		GPIOA_REG_RESET();
	}else if (pGPIOx == GPIOD)
	{
		GPIOA_REG_RESET();
	}else if (pGPIOx == GPIOE)
	{
		GPIOA_REG_RESET();
	}else if (pGPIOx == GPIOF)
	{
		GPIOA_REG_RESET();
	}else if (pGPIOx == GPIOG)
	{
		GPIOA_REG_RESET();
	}else if (pGPIOx == GPIOH)
	{
		GPIOA_REG_RESET();
	}else if (pGPIOx == GPIOI)
	{
		GPIOA_REG_RESET();
	}
}


/*
 * Data read and write
 */
/***************************************************************************
 * @fn					-  GPIO_ReadFromInputPin
 *
 * @brief				-
 *
 * @param[in}			-
 * @param[in}			-
 * @param[in}			-
 *
 * @return				- 0 or 1
 *
 * @Note				-
 */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	uint8_t value;

	value = (uint8_t)((pGPIOx->IDR >> PinNumber) & 0x00000001 );

	return value;
}


/***************************************************************************
 * @fn					- GPIO_ReadFromInputPort
 *
 * @brief				-
 *
 * @param[in}			-
 * @param[in}			-
 * @param[in}			-
 *
 * @return				-
 *
 * @Note				-
 *
 */
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx)
{
	uint16_t value;

	value = (uint16_t)pGPIOx->IDR;

	return value;
}


/***************************************************************************
 * @fn					- GPIO_WriteToOutputPin
 *
 * @brief				-
 *
 * @param[in}			-
 * @param[in}			-
 * @param[in}			-
 *
 * @return				-
 *
 * @Note				-
 *
 */
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value)
{

	if(Value == GPIO_PIN_SET)
	{
		//write 1 to the output data register at the bit field corresponding to the pin number
		pGPIOx->ODR |= (1<< PinNumber);
	} else
	{
		//write 0
		pGPIOx->ODR &= ~(1<< PinNumber);
	}
}

/***************************************************************************
 * @fn					- GPIO_WriteToOutputPort
 *
 * @brief				-
 *
 * @param[in}			-
 * @param[in}			-
 * @param[in}			-
 *
 * @return				-
 *
 * @Note				-
 *
 */
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value)
{

	pGPIOx->ODR |= Value;
}


/***************************************************************************
 * @fn					- GPIO_WriteToOutputPort
 *
 * @brief				-
 *
 * @param[in}			-
 * @param[in}			-
 * @param[in}			-
 *
 * @return				-
 *
 * @Note				-
 *
 */
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	pGPIOx->ODR ^= (1 << PinNumber); // In the shorthand of this pGPIOx->ODR = pGPIOx->ODR^ (1 << PinNumber);
}


/*
 * IRQ Configuration adn ISR handling
 */
/***************************************************************************
 * @fn					- GPIO_GPIO_IRQConfig
 *
 * @brief				-
 *
 * @param[in}			-
 * @param[in}			-
 * @param[in}			-
 *
 * @return				-
 *
 * @Note				-
 */
void GPIO_IRQConfig(uint8_t IRQNumber, uint8_t IRQPriority, uint8_t EnorDi){

}

/***************************************************************************
 * @fn					- GPIO_GPIO_IRQHandling
 *
 * @brief				-
 *
 * @param[in}			-
 * @param[in}			-
 * @param[in}			-
 *
 * @return				-
 *
 * @Note				-
 *
 */
void GPIO_IRQHandling(uint8_t PinNumber){

}

