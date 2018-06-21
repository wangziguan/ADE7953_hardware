/************************************************************************************************

 Author        : Wang Ziguan         

 Date          : June 2018

 File          : ADE7953.c

 Hardware      : ADE7953, stm32f103c8t6

 Description   : Test with ADE7953

*************************************************************************************************/

#include "ADE7953.h"

/******************************************************************************
**  Abstract:   
**		RESET ADE7953	
**  Parameters:	
**		none  
**  Returns:
**		none
*******************************************************************************/
void ADE7953Reset(void)
{
	HAL_GPIO_WritePin(RESET_N_GPIO_Port, RESET_N_Pin,GPIO_PIN_RESET);		//RESET ADE7953
	HAL_Delay(10);		
	HAL_GPIO_WritePin(RESET_N_GPIO_Port, RESET_N_Pin,GPIO_PIN_SET);
	HAL_Delay(10);	          
}
/******************************************************************************
**  Abstract:   
**		choose SPI interface,   
**  Parameters:	
**		none  
**  Returns:
**		none
*******************************************************************************/
void ADE7953CommuCfg(void)
{
	ADE7953_Enable();			  //choose SPI communication
}

/******************************************************************************
**  Abstract:   
**		ADE7953 configuration
**  Parameters:	
**		none  
**  Returns:
**		none
*******************************************************************************/
void ADE7953SPILock(void)
{
	uint8_t cmd[5];
	cmd[0] = (uint8_t)(0x0102>>8);
	cmd[1] = (uint8_t)(0x02);
	cmd[2] = 0x00; 
	cmd[3] = (uint8_t)(0x0004>>8);
	cmd[3] = (uint8_t)(0x04);
	
	ADE7953_Enable();
	/* Send the reset command */
	HAL_SPI_Transmit(&hspi1, cmd, 5, ADE7953_TIMEOUT_VALUE);
	ADE7953_Disable();
}

/******************************************************************************
**  Abstract:   
**		short delay
**  Parameters:	
**		none  
**  Returns:
**		none
*******************************************************************************/
void SPIDelay(void)
{
	int i;
	for(i=0; i<20; i++);
}

/******************************************************************************
**  Abstract:   
**		write 4 bytes to ADE7953
**  Parameters:	
**		unsigned int address: register address 
**		long int sendtemp   : configuration value  
**  Returns:
**		none
*******************************************************************************/
void SPIWrite4Bytes(uint16_t address , uint32_t sendtemp)
{
	uint8_t cmd[7];
	cmd[0] = (uint8_t)(address>>8);
	cmd[1] = (uint8_t)(address);
	cmd[2] = 0x00; 
	cmd[3] = (uint8_t)(sendtemp>>24);
	cmd[4] = (uint8_t)(sendtemp>>16);
	cmd[5] = (uint8_t)(sendtemp>>8);
	cmd[6] = (uint8_t)(sendtemp);
	
	ADE7953_Enable();
	/* Send the reset command */
	HAL_SPI_Transmit(&hspi1, cmd, 7, ADE7953_TIMEOUT_VALUE);
	ADE7953_Disable();

}

/******************************************************************************
**  Abstract:   
**		write 2 bytes to ADE7953
**  Parameters:	
**		unsigned int address: register address 
**		long int sendtemp   : configuration value  
**  Returns:
**		none
*******************************************************************************/
void SPIWrite2Bytes(uint16_t address , uint16_t sendtemp)
{
	uint8_t cmd[5];
	cmd[0] = (uint8_t)(address>>8);
	cmd[1] = (uint8_t)(address);
	cmd[2] = 0x00; 
	cmd[3] = (uint8_t)(sendtemp>>8);
	cmd[4] = (uint8_t)(sendtemp);
	
	ADE7953_Enable();
	/* Send the reset command */
	HAL_SPI_Transmit(&hspi1, cmd, 5, ADE7953_TIMEOUT_VALUE);
	ADE7953_Disable();

}

/******************************************************************************
**  Abstract:   
**		write 1 byte to ADE7953
**  Parameters:	
**		unsigned int address: register address 
**		long int sendtemp   : configuration value  
**  Returns:
**		none
*******************************************************************************/
void SPIWrite1Byte(uint16_t address , uint8_t sendtemp)
{
	uint8_t cmd[4];
	cmd[0] = (uint8_t)(address>>8);
	cmd[1] = (uint8_t)(address);
	cmd[2] = 0x00; 
	cmd[3] = (uint8_t)(sendtemp);
	
	ADE7953_Enable();
	/* Send the reset command */
	HAL_SPI_Transmit(&hspi1, cmd, 4, ADE7953_TIMEOUT_VALUE);
	ADE7953_Disable();
	
}

/**
  * @brief  Reads an amount of data from the QSPI memory.
  * @param  pData: Pointer to data to be read
  * @param  ReadAddr: Read start address
  * @param  Size: Size of data to read    
  * @retval QSPI memory status
  */
uint8_t BSP_ADE7953_Read(uint8_t* pData, uint16_t ReadAddr, uint32_t Size)
{
	uint8_t cmd[3];

	/* Configure the command */
	cmd[0] = (uint8_t)(ReadAddr >> 8);
	cmd[1] = (uint8_t)(ReadAddr);
	cmd[2] = 0x80;
	
	ADE7953_Enable();
	/* Send the read ID command */
	HAL_SPI_Transmit(&hspi1, cmd, 3, ADE7953_TIMEOUT_VALUE);	
	/* Reception of the data */
	if (HAL_SPI_Receive(&hspi1, pData, Size, ADE7953_TIMEOUT_VALUE) != HAL_OK)
  {
    return ADE7953_ERROR;
  }
	ADE7953_Disable();
	return ADE7953_OK;
	
}
/******************************************************************************
**  Abstract:   
**		ADE7953 configuration
**  Parameters:	
**		none  
**  Returns:
**		none
*******************************************************************************/
void ADE7953Cfg(void)
{
	SPIWrite2Bytes(CONFIG,0x000C); 		//SPI LOCK, PFMODE CYCMODE
//	SPIWrite4Bytes(AIGAIN,0x400000);
//	SPIWrite4Bytes(AVGAIN,0x400000);
//	SPIWrite4Bytes(AWGAIN,0x400000);
//	SPIWrite4Bytes(AVARGAIN,0x400000);
//	SPIWrite4Bytes(AVAGAIN,0x400000);
//	SPIWrite4Bytes(AIRMSOS,0x000000);
//	SPIWrite4Bytes(AVRMSOS,0x000000);
//	SPIWrite4Bytes(AWATTOS,0x000000);
//	SPIWrite4Bytes(AVAROS,0x000000);
//	SPIWrite4Bytes(AVAOS,0x000000);
//	SPIWrite4Bytes(BIGAIN,0x400000);
//	SPIWrite4Bytes(BWGAIN,0x400000);
//	SPIWrite4Bytes(BVARGAIN,0x400000);
//	SPIWrite4Bytes(BVAGAIN,0x400000);
//	SPIWrite4Bytes(BIRMSOS,0x000000);
//	SPIWrite4Bytes(BWATTOS,0x000000);
//	SPIWrite4Bytes(BVAROS,0x000000);
//	SPIWrite4Bytes(BVAOS,0x000000);		//GAIN SET
	
//	SPIWrite2Bytes(CFMODE,0x0300);		
//	SPIWrite2Bytes(CF1DEN,0x003F);
//	SPIWrite2Bytes(CF2DEN,0x003F);		//CF DISABLE
   	
//	SPIWrite4Bytes(OVLVL,0xFFFFFF);	  //311mv at the input get pead read 0x3DB88C, times 1.2 get this 
//	SPIWrite4Bytes(OILVL,0xFFFFFF);		//Over Voltage and Over Current
	
	SPIWrite1Byte(SAGCYC,0xFF);		  	//The SAGCYC register holds a maximum value of 255. At 50 Hz, the maximum sag cycle time is 2.55 seconds
	SPIWrite4Bytes(SAGLVL,0x000100);  //The SAGLVL voltage
	
//	SPIWrite2Bytes(ZXTOUT,0xFFFF);    //Zero crossing timeout.the maximum programmable timeout period is 4.58 seconds. 
	SPIWrite1Byte(LCYCMODE,0x55);			//ENABLE the ALWATT, ALVAR, ALVA CYCMODE.
	SPIWrite2Bytes(LINECYC,0x7530);		//LINECYC = 30000. At 50Hz, the cyc time is 300 seconds.
	
	SPIWrite4Bytes(IRQENA,0x140038);	//RESET,CYCEND,AEOFA,VAREOFA,VAEOFA
}
