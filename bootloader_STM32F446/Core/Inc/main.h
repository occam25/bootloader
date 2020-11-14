
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"

#define B1_Pin GPIO_PIN_13
#define B1_GPIO_Port GPIOC
#define USART_TX_Pin GPIO_PIN_2
#define USART_TX_GPIO_Port GPIOA
#define USART_RX_Pin GPIO_PIN_3
#define USART_RX_GPIO_Port GPIOA
#define LD2_Pin GPIO_PIN_5
#define LD2_GPIO_Port GPIOA
//#define TMS_Pin GPIO_PIN_13
//#define TMS_GPIO_Port GPIOA
//#define TCK_Pin GPIO_PIN_14
//#define TCK_GPIO_Port GPIOA
//#define SWO_Pin GPIO_PIN_3
//#define SWO_GPIO_Port GPIOB

// version 1.0
#define BL_VERSION	0x10

#define BL_ACK		0xa5
#define BL_NACK		0x7f

#define ADDR_VALID			0
#define ADDR_INVALID		1

// Commands
#define BL_GET_VER				0x51
#define BL_GET_HELP				0x52
#define BL_GET_CID 				0x53
#define BL_GET_RDP_STATUS		0x54
#define BL_GO_TO_ADDR			0x55
#define BL_FLASH_ERASE			0x56
#define BL_MEM_WRITE			0x57
#define BL_EN_R_W_PROTECT		0x58
#define BL_MEM_READ				0x59
#define BL_READ_SECTOR_STATUS	0x5a
#define BL_OTP_READ				0x5b
#define BL_DIS_R_W_PROTECT		0x5c

/*Some Start and End addresses of different memories of STM32F446xx MCU */
/*Change this according to your MCU */
#define SRAM1_SIZE			112*1024    // STM32F446RE has 112KB of SRAM1
#define SRAM1_END			(SRAM1_BASE + SRAM1_SIZE)
#define SRAM2_SIZE			16*1024     // STM32F446RE has 16KB of SRAM2
#define SRAM2_END			(SRAM2_BASE + SRAM2_SIZE)
#define FLASH_SIZE			512*1024    // STM32F446RE has 512KB of Flash
#define BKPSRAM_SIZE		4*1024		// STM32F446RE has 4KB of backup SRAM
#define BKPSRAM_END			(BKPSRAM_BASE + BKPSRAM_SIZE)

void Error_Handler(void);

void bootloader_uart_read_data(void);
void bootloader_jump_to_user_app(void);

void bl_get_version_cmd(uint8_t *pBuf);
void bl_get_help_cmd(uint8_t *pBuf);
void bl_get_cid_cmd(uint8_t *pBuf);
void bl_get_rdp_status_cmd(uint8_t *pBuf);
void bl_go_to_address_cmd(uint8_t *pBuf);
void bl_flash_erase_cmd(uint8_t *pBuf);
void bl_mem_write_cmd(uint8_t *pBuf);
void bl_enable_R_W_protect_cmd(uint8_t *pBuf);
void bl_mem_read_cmd(uint8_t *pBuf);
void bl_read_sector_status_cmd(uint8_t *pBuf);
void bl_otp_read_cmd(uint8_t *pBuf);
void bl_disable_R_W_protect_cmd(uint8_t *pBuf);

void bl_send_ACK(uint8_t follow_len);
void bl_send_NACK(void);


uint8_t bl_command_CRC_check(uint8_t *pCommand);
uint8_t bl_get_version(void);
void bl_uart_write_data(uint8_t *data, uint8_t len);

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

