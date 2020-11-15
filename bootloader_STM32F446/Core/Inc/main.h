
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

#define FLASH_SECTOR2_BASE_ADDRESS 		0x08008000U

// version 1.0
#define BL_VERSION	0x10

#define BL_ACK		0xa5
#define BL_NACK		0x7f

#define BL_ERROR		1
#define BL_SUCCESS		0

#define BL_TRUE			1
#define BL_FALSE		0

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

#define BL_FLASH_SEC0_BASE		0x08000000UL
#define BL_FLASH_SEC0_END		0x08003FFFUL	// Sector 0 size: 16KB
#define BL_FLASH_SEC1_BASE		0x08004000UL
#define BL_FLASH_SEC1_END		0x08007FFFUL	// Sector 1 size: 16KB
#define BL_FLASH_SEC2_BASE		0x08008000UL
#define BL_FLASH_SEC2_END		0x0800BFFFUL	// Sector 2 size: 16KB
#define BL_FLASH_SEC3_BASE		0x0800C000UL
#define BL_FLASH_SEC3_END		0x0800FFFFUL	// Sector 3 size: 16KB
#define BL_FLASH_SEC4_BASE		0x08010000UL
#define BL_FLASH_SEC4_END		0x0801FFFFUL	// Sector 4 size: 64KB
#define BL_FLASH_SEC5_BASE		0x08020000UL
#define BL_FLASH_SEC5_END		0x0803FFFFUL	// Sector 5 size: 128KB
#define BL_FLASH_SEC6_BASE		0x08040000UL
#define BL_FLASH_SEC6_END		0x0805FFFFUL	// Sector 6 size: 128KB
#define BL_FLASH_SEC7_BASE		0x08060000UL
#define BL_FLASH_SEC7_END		0x0807FFFFUL	// Sector 7 size: 128KB

#define BL_FLASH_SYSMEM_BASE	0x1FFF0000UL
#define BL_FLASH_SYSMEM_END		0x1FFF77FFUL	// System memory size: 30KB

#define BL_FLASH_OTP_BASE		0x1FFF7800UL
#define BL_FLASH_OTP_END		0x1FFF7A0FUL	// One time programming area size: 528B

#define BL_FLASH_OPT_BASE		0x1FFFC000UL
#define BL_FLASH_OPT_END		0x1FFFC00FUL	// Option bytes size: 16B

typedef enum bl_mem_type_enum{
	BL_SRAM1,
	BL_SRAM2,
	BL_FLASH_SEC0,
	BL_FLASH_SEC1,
	BL_FLASH_SEC2,
	BL_FLASH_SEC3,
	BL_FLASH_SEC4,
	BL_FLASH_SEC5,
	BL_FLASH_SEC6,
	BL_FLASH_SEC7,
	BL_FLASH_SYSMEM,
	BL_FLASH_OTP,
	BL_FLASH_OPT,
	BL_BKPRAM,
	BL_INVALID
}bl_mem_type_t;


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

