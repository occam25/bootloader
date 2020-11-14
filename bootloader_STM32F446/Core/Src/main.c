
#include <string.h>
#include <stdarg.h>
#include <stdio.h>

#include "main.h"

#define BL_DEBUG_MSG_EN
#define D_UART	&huart3
#define C_UART	&huart2

#define CRC_SUCCESS 	0
#define CRC_FAIL 		1

CRC_HandleTypeDef hcrc;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

uint8_t bl_rx_buffer[512];

uint8_t supported_commands[] = {BL_GET_VER,
								BL_GET_HELP,
								BL_GET_CID,
								BL_GET_RDP_STATUS,
								BL_GO_TO_ADDR,
								BL_FLASH_ERASE,
								BL_MEM_WRITE,
								BL_EN_R_W_PROTECT,
								BL_MEM_READ,
								BL_READ_SECTOR_STATUS,
								BL_OTP_READ,
								BL_DIS_R_W_PROTECT};

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_CRC_Init(void);
static void MX_USART3_UART_Init(void);

void printmsg(char *format, ...);

static uint8_t CRC_check(uint8_t *pData, uint32_t len, uint32_t crc_host);
static uint16_t get_mcu_chip_id(void);
static uint8_t get_flash_rpd(void);
static uint8_t verify_address(uint32_t address);
static uint8_t flash_erase_sectors(uint8_t first_sector, uint8_t num_of_sectors);
static uint8_t flash_mass_erase(void);

int main(void)
{

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_CRC_Init();
  MX_USART3_UART_Init();

  if(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) == GPIO_PIN_RESET){
	  // Button pressed, enter bootloader mode
	  printmsg("BL_DEBUG_MSG: Button is pressed, going to BL mode\r\n");
	  bootloader_uart_read_data();
  }else{
	  printmsg("BL_DEBUG_MSG: Button not pressed, executing user app\r\n");
	  bootloader_jump_to_user_app();
  }

  uint32_t current_tick;
  while (1)
  {

	  //HAL_UART_Transmit(&huart3, TxBuff, strlen((char *)TxBuff), HAL_MAX_DELAY);
	  //HAL_Delay(100);

	  current_tick = HAL_GetTick();
	  printmsg("Current tick: %d\r\n", current_tick);

	  while(HAL_GetTick() <= current_tick + 500);
  }
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage 
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief CRC Initialization Function
  * @param None
  * @retval None
  */
static void MX_CRC_Init(void)
{

  hcrc.Instance = CRC;
  if (HAL_CRC_Init(&hcrc) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

}

void bootloader_uart_read_data(void)
{
	uint8_t rcv_len = 0;
	HAL_StatusTypeDef status;

	printmsg("BL_DEBUG_MSG: content of %#x: %#x\r\n", (FLASH_SECTOR2_BASE_ADDRESS + 4), *((uint32_t *)(FLASH_SECTOR2_BASE_ADDRESS + 4)));

	/* Read commands from serial port */
	while(1){

		memset(bl_rx_buffer, 0, sizeof(bl_rx_buffer));

		// 1. Read first byte containing the length
		HAL_UART_Receive(C_UART, bl_rx_buffer, 1, HAL_MAX_DELAY);
		rcv_len = bl_rx_buffer[0];

		// 2. Read command bytes
		status = HAL_UART_Receive(C_UART, &bl_rx_buffer[1], rcv_len, HAL_MAX_DELAY);

		if(status != HAL_OK)
			continue;

		switch(bl_rx_buffer[1]){
		case BL_GET_VER:
			bl_get_version_cmd(bl_rx_buffer);
			break;
		case BL_GET_HELP:
			bl_get_help_cmd(bl_rx_buffer);
			break;
		case BL_GET_CID:
			bl_get_cid_cmd(bl_rx_buffer);
			break;
		case BL_GET_RDP_STATUS:
			bl_get_rdp_status_cmd(bl_rx_buffer);
			break;
		case BL_GO_TO_ADDR:
			bl_go_to_address_cmd(bl_rx_buffer);
			break;
		case BL_FLASH_ERASE:
			bl_flash_erase_cmd(bl_rx_buffer);
			break;
		case BL_MEM_WRITE:
			bl_mem_write_cmd(bl_rx_buffer);
			break;
		case BL_EN_R_W_PROTECT:
			bl_enable_R_W_protect_cmd(bl_rx_buffer);
			break;
		case BL_MEM_READ:
			bl_mem_read_cmd(bl_rx_buffer);
			break;
		case BL_READ_SECTOR_STATUS:
			bl_read_sector_status_cmd(bl_rx_buffer);
			break;
		case BL_OTP_READ:
			bl_otp_read_cmd(bl_rx_buffer);
			break;
		case BL_DIS_R_W_PROTECT:
			bl_disable_R_W_protect_cmd(bl_rx_buffer);
			break;
		default:
			printmsg("BL_DEBUG_MSG: Invalid command code received from host (0x%02x)\r\n", bl_rx_buffer[1]);
			break;
		}
	}
}
/*
 *
 */
void bl_get_version_cmd(uint8_t *pBuf)
{
	uint8_t bl_version;

	printmsg("BL_DEBUG_MSG: bl_get_version_cmd\r\n");

	// Check CRC
	if(!bl_command_CRC_check(pBuf)){
		// CRC OK
		printmsg("BL_DEBUG_MSG: CRC verification OK.\r\n");
		bl_send_ACK(1);
		// Get version
		bl_version = bl_get_version();
		printmsg("BL_DEBUG_MSG: BL version 0x%02x\r\n", bl_version);

		bl_uart_write_data(&bl_version, 1);
	}else{
		// CRC corrupted
		printmsg("BL_DEBUG_MSG: CRC verification failed.\r\n");
		bl_send_NACK();
	}
}

void bl_get_help_cmd(uint8_t *pBuf)
{
	printmsg("BL_DEBUG_MSG: bl_get_help_cmd\r\n");

	// Check CRC
	if(!bl_command_CRC_check(pBuf)){
		// CRC OK
		printmsg("BL_DEBUG_MSG: CRC verification OK.\r\n");
		bl_send_ACK(strlen((char *)supported_commands));

		bl_uart_write_data(supported_commands, strlen((char *)supported_commands));
	}else{
		// CRC corrupted
		printmsg("BL_DEBUG_MSG: CRC verification failed.\r\n");
		bl_send_NACK();
	}
}
void bl_get_cid_cmd(uint8_t *pBuf)
{
	uint16_t bl_cid;

	printmsg("BL_DEBUG_MSG: bl_get_cid_cmd\r\n");

	// Check CRC
	if(!bl_command_CRC_check(pBuf)){
		// CRC OK
		printmsg("BL_DEBUG_MSG: CRC verification OK.\r\n");
		bl_send_ACK(2);
		bl_cid = get_mcu_chip_id();
		bl_uart_write_data((uint8_t *)&bl_cid, 2);
	}else{
		// CRC corrupted
		printmsg("BL_DEBUG_MSG: CRC verification failed.\r\n");
		bl_send_NACK();
	}
}

static uint16_t get_mcu_chip_id(void)
{
	/*
	The STM32F446xx MCUs integrate an MCU ID code. This ID identifies the ST MCU partnumber
	and the die revision. It is part of the DBG_MCU component and is mapped on the
	external PPB bus (see Section 33.16 on page 1304). This code is accessible using the
	JTAG debug pCat.2ort (4 to 5 pins) or the SW debug port (two pins) or by the user software.
	It is even accessible while the MCU is under system reset. */
	return ((uint16_t)(DBGMCU->IDCODE) & 0x0fff);
}

void bl_get_rdp_status_cmd(uint8_t *pBuf)
{
	uint8_t flash_rpd;

	printmsg("BL_DEBUG_MSG: bl_get_rdp_status_cmd\r\n");

	// Check CRC
	if(!bl_command_CRC_check(pBuf)){
		// CRC OK
		printmsg("BL_DEBUG_MSG: CRC verification OK.\r\n");
		bl_send_ACK(1);
		flash_rpd = get_flash_rpd();
		printmsg("BL_DEBUG_MSG: RDP level: %#x\r\n", flash_rpd);
		bl_uart_write_data(&flash_rpd, 1);
	}else{
		// CRC corrupted
		printmsg("BL_DEBUG_MSG: CRC verification failed.\r\n");
		bl_send_NACK();
	}
}
static uint8_t get_flash_rpd(void)
{
	uint8_t opt_flash_rpd_status;

#ifdef RPD_USE_FLASH_DRIVER_METHOD
	FLASH_OBProgramInitTypeDef ob_handle; // option bytes handle
	HAL_FLASHEx_OBGetConfig(&ob_handle);
	opt_flash_rpd_status = (uint8_t)ob_handle.RDPLevel;
#else
	opt_flash_rpd_status = (uint8_t)(*((volatile uint32_t *)0x1fffc000) >> 8);
#endif
	return opt_flash_rpd_status;
}

void bl_go_to_address_cmd(uint8_t *pBuf)
{

	uint32_t goto_address;
	uint8_t status;

	printmsg("BL_DEBUG_MSG: bl_go_to_address_cmd\r\n");

	// Check CRC
	if(!bl_command_CRC_check(pBuf)){
		// CRC OK
		printmsg("BL_DEBUG_MSG: CRC verification OK.\r\n");
		bl_send_ACK(1);
		goto_address = *((uint32_t *)(&pBuf[2]));
		printmsg("BL_DEBUG_MSG: GOTO address: %#x\r\n", goto_address);
		if(verify_address(goto_address) == ADDR_VALID){
			status = 1;
			bl_uart_write_data(&status, 1);

            /*jump to "go" address.
            we dont care what is being done there.
            host must ensure that valid code is present over there
            Its not the duty of bootloader. so just trust and jump */

            /* Not doing the below line will result in hardfault exception for ARM cortex M */
            //watch : https://www.youtube.com/watch?v=VX_12SjnNhY

			goto_address |= 1; //make T bit =1
			void (*jump)(void) = (void *)goto_address;
			printmsg("BL_DEBUG_MSG: Jumping to address: %#x\r\n", goto_address);
			jump(); // In app code the reset handler is at 0x08008A50
		}else{
			status = 0;
			bl_uart_write_data(&status, 1);
		}

//		func();
	}else{
		// CRC corrupted
		printmsg("BL_DEBUG_MSG: CRC verification failed.\r\n");
		bl_send_NACK();
	}
}
static uint8_t verify_address(uint32_t address)
{
	//so, what are the valid addresses to which we can jump ?
	//can we jump to system memory ? yes
	//can we jump to sram1 memory ?  yes
	//can we jump to sram2 memory ? yes
	//can we jump to backup sram memory ? yes
	//can we jump to peripheral memory ? its possible , but dont allow. so no
	//can we jump to external memory ? yes.
	if((address >= SRAM1_BASE)&&(address <= SRAM1_END)){
		return ADDR_VALID;
	}else if((address >= SRAM2_BASE)&&(address <= SRAM2_END)){
		return ADDR_VALID;
	}else if((address >= FLASH_BASE)&&(address <= FLASH_END)){
		return ADDR_VALID;
	}else if((address >= BKPSRAM_BASE)&&(address <= BKPSRAM_END)){
		return ADDR_VALID;
	}else
		return ADDR_INVALID;
}

#define USE_FLASH_DRIVER
void bl_flash_erase_cmd(uint8_t *pBuf)
{
	uint8_t first_sector;
	uint8_t num_of_sectors;
	uint8_t status = 0;

	printmsg("BL_DEBUG_MSG: bl_flash_erase_cmd\r\n");

	// Check CRC
	if(!bl_command_CRC_check(pBuf)){
		// CRC OK
		printmsg("BL_DEBUG_MSG: CRC verification OK.\r\n");
		bl_send_ACK(1);

		first_sector = pBuf[2];
		num_of_sectors = pBuf[3];

		HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
		if(first_sector == 0xff)
			status = flash_mass_erase();
		else
			status = flash_erase_sectors(first_sector, num_of_sectors);
		HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

		bl_uart_write_data(&status, 1);
	}else{
		// CRC corrupted
		printmsg("BL_DEBUG_MSG: CRC verification failed.\r\n");
		bl_send_NACK();
	}
}

static uint8_t flash_erase_sectors(uint8_t first_sector, uint8_t num_of_sectors)
{

	uint8_t status = 0;

	/*
	 * STM32F446RE MCU has 8 sectors (sector 0 to 7)
	 * first_sector has to be in the range of 0 to 7
	 */

	if(first_sector > 7){
		printmsg("BL_DEBUG_MSG: Invalid sector (%d)\r\n", first_sector);
		return 1;
	}

	if(num_of_sectors == 0){
		printmsg("BL_DEBUG_MSG: Invalid number of sectors (%d)\r\n", num_of_sectors);
		return 1;
	}

	if(num_of_sectors > 8 - first_sector)
		num_of_sectors = 8 - first_sector;

	printmsg("BL_DEBUG_MSG: Erasing %d sector(s) starting from sector %d\r\n", num_of_sectors, first_sector);

	/************* Sector(s) erase **************/
#ifdef USE_FLASH_DRIVER
	uint32_t SectorError;
	FLASH_EraseInitTypeDef flash_erase_conf;

	flash_erase_conf.Sector = first_sector;
	flash_erase_conf.NbSectors = num_of_sectors;
	flash_erase_conf.VoltageRange = FLASH_VOLTAGE_RANGE_3;
	flash_erase_conf.TypeErase = FLASH_TYPEERASE_SECTORS;
	flash_erase_conf.Banks = FLASH_BANK_1;

	HAL_FLASH_Unlock();
	status = (uint8_t)HAL_FLASHEx_Erase(&flash_erase_conf, &SectorError);
	HAL_FLASH_Lock();
	printmsg("BL_DEBUG_MSG: Flash driver sector(s) erase status: %#x\r\n", SectorError);
#else
	/********* Unlock Flash sequence ********/
	// 1. Write KEY1 = 0x45670123 in the Flash key register (FLASH_KEYR)
	FLASH->KEYR = 0x45670123;
	// 2. Write KEY2 = 0xCDEF89AB in the Flash key register (FLASH_KEYR)
	FLASH->KEYR = 0xCDEF89AB;
	for(uint8_t i = 0; i < num_of_sectors; i++){
		// 1. Check that no flash memory operation is ongoing by checking the BSY bit in the FLASH_SR register
		while(FLASH->SR & FLASH_SR_BSY);
		// 2. Set the SER bit and select the sector out of the 7 sectors in the main memory block you
		//	  wish to erase (SNB) in the FLASH_CR register
		uint32_t tempreg = 0;
		tempreg |= FLASH_CR_SER | ((first_sector + i) << FLASH_CR_SNB_Pos);
		FLASH->CR |= tempreg;
		// 3. Set the STRT bit in the FLASH_CR register
		FLASH->CR |= FLASH_CR_STRT;
		// 4. Wait for the BSY bit to be cleared
		while(FLASH->SR & FLASH_SR_BSY);
	}
	/************* Lock flash **************/
	// Lock the flash again: The FLASH_CR register can be locked again by software by setting the LOCK bit in the FLASH_CR register.
	FLASH->CR |= FLASH_CR_LOCK;
#endif

	return status;
}
static uint8_t flash_mass_erase(void)
{
	uint8_t status = 0;

	/************* Mass erase: erase complete flash **************/
#ifdef USE_FLASH_DRIVER
	uint32_t SectorError;
	FLASH_EraseInitTypeDef flash_erase_conf;
	flash_erase_conf.VoltageRange = FLASH_VOLTAGE_RANGE_3;
	flash_erase_conf.TypeErase = FLASH_TYPEERASE_MASSERASE;
	flash_erase_conf.Banks = FLASH_BANK_1;

	HAL_FLASH_Unlock();
	status = (uint8_t)HAL_FLASHEx_Erase(&flash_erase_conf, &SectorError);
	HAL_FLASH_Lock();

	printmsg("BL_DEBUG_MSG: Flash driver mass erase status: %#x\r\n", SectorError);
#else
	/********* Unlock Flash sequence ********/
	// 1. Write KEY1 = 0x45670123 in the Flash key register (FLASH_KEYR)
	FLASH->KEYR = 0x45670123;
	// 2. Write KEY2 = 0xCDEF89AB in the Flash key register (FLASH_KEYR)
	FLASH->KEYR = 0xCDEF89AB;
	/********* Mass erase ********/
	// 1. Check that no flash memory operation is ongoing by checking the BSY bit in the FLASH_SR register
	while(FLASH->SR & FLASH_SR_BSY);
	// 2. Set the MER bit in the FLASH_CR register
	FLASH->CR |= FLASH_CR_MER;
	// 3. Set the STRT bit in the FLASH_CR register
	FLASH->CR |= FLASH_CR_STRT;
	// 4. Wait for the BSY bit to be cleared
	while(FLASH->SR & FLASH_SR_BSY);
	/************* Lock flash **************/
	// Lock the flash again: The FLASH_CR register can be locked again by software by setting the LOCK bit in the FLASH_CR register.
	FLASH->CR |= FLASH_CR_LOCK;
#endif

		return status;
}

void bl_mem_write_cmd(uint8_t *pBuf)
{

}
void bl_enable_R_W_protect_cmd(uint8_t *pBuf)
{

}
void bl_mem_read_cmd(uint8_t *pBuf)
{

}
void bl_read_sector_status_cmd(uint8_t *pBuf)
{

}
void bl_otp_read_cmd(uint8_t *pBuf)
{

}
void bl_disable_R_W_protect_cmd(uint8_t *pBuf)
{

}

uint8_t bl_get_version(void)
{
	return (uint8_t)BL_VERSION;
}




static uint8_t CRC_check(uint8_t *pData, uint32_t len, uint32_t crc_host)
{
	uint32_t uwCRCValue = 0xff;

	__HAL_CRC_DR_RESET(&hcrc);

	for(uint32_t i = 0; i < len; i++){
		uint32_t i_data = pData[i];
		uwCRCValue = HAL_CRC_Accumulate(&hcrc, &i_data, 1);
	}

	if(uwCRCValue == crc_host){
		return CRC_SUCCESS;
	}

	return CRC_FAIL;
}

/*
 *    ______ 1 __________ n ____________________ 4 ________________
 *   |               |         |        |        |        |        |
 *   | len_to_follow | command |            host_crc               |
 *   |_______________|_________|________|________|________|________|
 *
 *   len_to_follow = n + 4
 *   Total command length: len_to_follow + 1
 *   (CRC computed with len_to_follow and command bytes)
 */
uint8_t bl_command_CRC_check(uint8_t *pCommand)
{
	uint32_t total_len;
	uint32_t host_crc;

	total_len = pCommand[0] + 1;
	host_crc = *((uint32_t *)(pCommand + total_len - 4));

	return CRC_check(pCommand, total_len - 4, host_crc);
}

/*
 *
 */
void bl_uart_write_data(uint8_t *data, uint8_t len)
{
	HAL_UART_Transmit(C_UART, data, len, HAL_MAX_DELAY);
}

void bl_send_ACK(uint8_t follow_len)
{
	uint8_t ack_buf[2];
	ack_buf[0] = BL_ACK;
	ack_buf[1] = follow_len;
	HAL_UART_Transmit(C_UART, ack_buf, 2, HAL_MAX_DELAY);
}

void bl_send_NACK(void)
{
	uint8_t nack;
	nack = BL_NACK;
	HAL_UART_Transmit(C_UART, &nack, 1, HAL_MAX_DELAY);
}

/*
 * Code to jump to user application
 * Here we are assuming FLASH_SECTOR2_BASE_ADDRESS is
 * where the user application is stored
 */
void bootloader_jump_to_user_app(void)
{
	// function pointer to hold the address of the reset handler of the user app
	void (*app_reset_handler)(void);

	printmsg("BL_DEBUG_MSG: bootloader_jump_to_user_app\r\n");

	// 1. Configure the MSP by reading the value from the base address of the sector 2
	uint32_t msp_value = *(volatile uint32_t *)FLASH_SECTOR2_BASE_ADDRESS;
	printmsg("BL_DEBUG_MSG: MSP value: %#x\r\n", msp_value);

	// this function comes from CMSIS
	__set_MSP(msp_value);

	// SCB->VTOR = FLASH_SECTOR2_BASE_ADDRESS
	//SCB->VTOR = 0x08000000 | 0x8000;


	// 2. Now fetch the reset handler address of the user application
	//    from the location FLASH_SECTOR2_BASE_ADDRESS
	uint32_t resethandler_address = *(volatile uint32_t *)(FLASH_SECTOR2_BASE_ADDRESS + 4);

	app_reset_handler = (void *)resethandler_address;

	printmsg("BL_DEBUG_MSG: app reset handler addr: %#x\r\n", app_reset_handler);

	app_reset_handler();

}

/* prints formatted string to console over UART */
void printmsg(char *format, ...)
{
#ifdef BL_DEBUG_MSG_EN
	char str[80];
	va_list args;
	va_start(args, format);
	vsprintf(str, format, args);
	HAL_UART_Transmit(D_UART, (uint8_t *)str, strlen(str), HAL_MAX_DELAY);
#endif
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* User can add his own implementation to report the HAL error return state */

}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

