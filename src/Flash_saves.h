#pragma once

#include "main.h"

#include "ch32v20x_flash.h"


#define FLASH_PAGE_SIZE 4096

extern void Flash_saves_init();
extern bool Flash_saves(void*buf,uint32_t length,uint32_t address);
extern bool Flash_reads(void *buf, uint32_t length, uint32_t address);