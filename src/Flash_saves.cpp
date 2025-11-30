#include "Flash_saves.h"

/* Global define */
typedef enum
{
    FAILED = 0,
    PASSED = !FAILED
} TestStatus;
// #define PAGE_WRITE_START_ADDR ((uint32_t)0x08008000) /* Start from 32K */
// #define PAGE_WRITE_END_ADDR ((uint32_t)0x08009000)   /* End at 36K */
#define FLASH_PAGE_SIZE 4096
#define FLASH_PAGES_TO_BE_PROTECTED FLASH_WRProt_Pages60to63

/* Global Variable */
uint32_t EraseCounter = 0x0, Address = 0x0;
uint16_t Data = 0xAAAA;
uint32_t WRPR_Value = 0xFFFFFFFF, ProtectedPages = 0x0;

volatile FLASH_Status FLASHStatus = FLASH_COMPLETE;
volatile TestStatus MemoryProgramStatus = PASSED;
volatile TestStatus MemoryEraseStatus = PASSED;

#define Fadr (0x08020000)
#define Fsize ((((256 * 4)) >> 2))
u32 buf[Fsize];

void Flash_saves_init()
{
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_CRC, ENABLE);
}


bool Flash_saves(void *buf, uint32_t length, uint32_t address)
{
    uint32_t end_address = address + length;
    uint32_t erase_counter = 0;
    uint32_t address_i = 0;
    uint32_t page_num = length / FLASH_PAGE_SIZE;
    uint16_t *data_ptr=(uint16_t *)buf;

    __disable_irq(); // 禁用中断
    FLASH_Unlock();

    FLASH_ClearFlag(FLASH_FLAG_BSY | FLASH_FLAG_EOP | FLASH_FLAG_WRPRTERR);

    for (erase_counter = 0; (erase_counter < page_num) && (FLASHStatus == FLASH_COMPLETE); erase_counter++)
    {
        FLASHStatus = FLASH_ErasePage(address + (FLASH_PAGE_SIZE * erase_counter)); // Erase 4KB

        if (FLASHStatus != FLASH_COMPLETE)
            return false;
    }

    address_i = address;
    while ((address_i < end_address) && (FLASHStatus == FLASH_COMPLETE))
    {
        FLASHStatus = FLASH_ProgramHalfWord(address_i, *data_ptr);
        address_i = address_i + 2;
        data_ptr++;
    }
    uint32_t *data_ptr32=(uint32_t *)address;
    CRC->CTLR = 1;
    for(uint32_t i=0;i<length;i+=4)
    {
        CRC->DATAR = data_ptr32[i];
    }
    uint32_t crc = CRC->DATAR;
    uint32_t addr=address+length+4-((address+length)%4);
    FLASH_ProgramHalfWord(addr, crc);//在对齐4字节的位置写入crc
    FLASH_ProgramHalfWord(addr+2, crc>>16);
    
    FLASH_Lock();
    __enable_irq();
    return true;
}

bool Flash_reads(void *buf, uint32_t length, uint32_t address)
{
    uint32_t *data_ptr32=(uint32_t *)address;
    CRC->CTLR = 1;
    for(uint32_t i=0;i<length;i+=4)
    {
        CRC->DATAR = data_ptr32[i];
    }
    uint32_t crc=CRC->DATAR;
    
    if ((crc == *((uint32_t *)(address+length+4-(address+length)%4))))
    {
        memcpy(buf, (void *)address, length);
        return true;
    }
    return false;
}