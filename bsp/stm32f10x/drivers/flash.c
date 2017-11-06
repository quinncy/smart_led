#include "flash.h"

#define  STARTADDR  0x08010000                   	 //STM32F103RC
volatile FLASH_Status FLASHStatus = FLASH_COMPLETE;      //Flash²Ù×÷×´Ì¬±äÁ¿

/****************************************************************
*Name:		ReadFlashNBtye
*Function:	???Flash??N????
*Input:		ReadAddress:????(????)ReadBuf:????	ReadNum:?????
*Output:	??????  
*Author:    ValerianFan
*Date:		2014/04/09
*E-Mail:	fanwenjingnihao@163.com
*Other:		
****************************************************************/
int ReadFlashNBtye(uint32_t ReadAddress, uint8_t *ReadBuf, int32_t ReadNum) 
{
        int DataNum = 0;
		ReadAddress = (uint32_t)STARTADDR + ReadAddress; 
        while(DataNum < ReadNum) 
		{
           *(ReadBuf + DataNum) = *(__IO uint8_t*) ReadAddress++;
           DataNum++;
        }
        return DataNum;
}

/****************************************************************
*Name:		WriteFlashOneWord
*Function:	???Flash??32???
*Input:		WriteAddress:????(????)WriteData:????
*Output:	NULL 
*Author:    ValerianFan
*Date:		2014/04/09
*E-Mail:	fanwenjingnihao@163.com
*Other:		
****************************************************************/

void WriteFlashOneWord(uint32_t WriteAddress,uint32_t WriteData)
{
	FLASH_UnlockBank1();
	FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPRTERR); 
    FLASHStatus = FLASH_ErasePage(STARTADDR);

	if(FLASHStatus == FLASH_COMPLETE)
	{
		FLASHStatus = FLASH_ProgramWord(STARTADDR + WriteAddress, WriteData);    //flash.c ?API??
		//FLASHStatus = FLASH_ProgramWord(StartAddress+4, 0x56780000);                      //???????????
		//FLASHStatus = FLASH_ProgramWord(StartAddress+8, 0x87650000);                      //???????????
	}
	FLASH_LockBank1();
} 

