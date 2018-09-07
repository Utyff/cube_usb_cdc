#include "main.h"

#define LED1_Pin GPIO_IDR_ID4
#define LED1_GPIO_Port GPIOA
#define  LED1_TOGGLE() LED1_GPIO_Port->ODR ^= LED1_Pin

extern USBD_HandleTypeDef USBD_Device;
uint8_t CDC_BUF[128];

void SetClocks();


int main(void) {
	// Initialize the UART
//	UARTx_Init(USART2,USART_TX,1382400);
//	printf("--- STM32L151RDT6 ---\r\n");

    SetClocks();

    RCC->IOPENR |= RCC_IOPENR_GPIOAEN;
    GPIOA->MODER = (GPIOA->MODER & ~(GPIO_MODER_MODE4)) | (GPIO_MODER_MODE4_0);

    // Initialize the CDC Application
	USBD_Init(&USBD_Device,&USBD_CDC_Descriptor,0);
	// Add Supported Class
	USBD_RegisterClass(&USBD_Device,&USBD_CDC);
	// Add CDC Interface Class
	USBD_CDC_RegisterInterface(&USBD_Device,&USBD_CDC_fops);
	// Start Device Process
	USBD_Start(&USBD_Device);

	// Stuff the buffer
	CDC_BUF[0]  = 'H';
	CDC_BUF[1]  = 'E';
	CDC_BUF[2]  = 'L';
	CDC_BUF[3]  = 'L';
	CDC_BUF[4]  = 'O';
	CDC_BUF[5]  = ' ';
	CDC_BUF[6]  = 'C';
	CDC_BUF[7]  = 'D';
	CDC_BUF[8]  = 'C';
	CDC_BUF[9]  = '\r';
	CDC_BUF[10] = '\n';

	uint32_t i;
	while(1) {
		CDC_Itf_Transmit(CDC_BUF,11);
		for (i = 0x008FFFFF; i--; );
        LED1_TOGGLE();
	}
}


void SetClocks() {
// ==============  HSI
    RCC->CR |= RCC_CR_HSION;
    while (RCC->CR & RCC_CR_HSIRDY == RESET);

// ============== HSI48
    SET_BIT(RCC->CRRCR, RCC_CRRCR_HSI48ON);
    SET_BIT(RCC->APB2ENR, RCC_APB2ENR_SYSCFGEN);
    while (RCC->CRRCR & RCC_CRRCR_HSI48RDY == RESET);

// ============== PLL
    RCC->CFGR |= RCC_CFGR_PLLSRC_HSI | RCC_CFGR_PLLMUL4 | RCC_CFGR_PLLDIV2;
    // Enable the main PLL.
    SET_BIT(RCC->CR, RCC_CR_PLLON);
    // Wait till PLL is ready
    while (RCC->CR & RCC_CR_PLLRDY == RESET);

// ============== FLASH
    FLASH->ACR |= FLASH_ACR_LATENCY;
    // Check that the new number of wait states is taken into account to access the Flash
    // memory by reading the FLASH_ACR register
//    if ((FLASH->ACR & FLASH_ACR_LATENCY) == RESET) { Error_Handler(); }

// ============== HCLK
    RCC->CFGR |= RCC_CFGR_HPRE_DIV1;

// ============== SYSCLK
    RCC->CFGR |= RCC_CFGR_SW_PLL;
    while (RCC->CFGR & RCC_CFGR_SWS != RCC_CFGR_SWS_PLL);

    // set HSI48 as USB Clock source
    RCC->CCIPR |= RCC_CCIPR_HSI48SEL;

// ==========  RCCEx_CRSConfig

    // Before configuration, reset CRS registers to their default values
    // RCC_CRS_FORCE_RESET()
    SET_BIT(RCC->APB1RSTR, RCC_APB1RSTR_CRSRST);
    // RCC_CRS_RELEASE_RESET()
    CLEAR_BIT(RCC->APB1RSTR, RCC_APB1RSTR_CRSRST);

// Set the SYNCDIV[2:0] bits to Prescaler value
#define RCC_CRS_SYNC_DIV1              ((uint32_t)0x00000000U) // Synchro Signal not divided (default)
// Set the SYNCSRC[1:0] bits to Source value
#define RCC_CRS_SYNC_SOURCE_USB        CRS_CFGR_SYNCSRC_1      // Synchro Signal source USB SOF (default)
// Set the SYNCSPOL bit to Polarity value
#define RCC_CRS_SYNC_POLARITY_RISING   ((uint32_t)0x00000000U) // Synchro Active on rising edge (default)
// Set the RELOAD[15:0] bits to ReloadValue value
#define RCC_CRS_RELOADVALUE_CALCULATE(__FTARGET__, __FSYNC__)  (((__FTARGET__) / (__FSYNC__)) - 1)
// Set the FELIM[7:0] bits according to ErrorLimitValue value
#define RCC_CRS_ERROR_LIMIT 34

    CRS->CFGR = RCC_CRS_SYNC_DIV1 | RCC_CRS_SYNC_SOURCE_USB | RCC_CRS_SYNC_POLARITY_RISING |
                RCC_CRS_RELOADVALUE_CALCULATE(48000000, 1000) | (RCC_CRS_ERROR_LIMIT << CRS_CFGR_FELIM_Pos);

    // Adjust HSI48 oscillator smooth trimming
    // Set the TRIM[5:0] bits according to RCC_CRS_HSI48CalibrationValue value
    //MODIFY_REG(CRS->CR, CRS_CR_TRIM, (32 << CRS_CR_TRIM_Pos));

    // Enable Automatic trimming & Frequency error counter
    CRS->CR |= CRS_CR_AUTOTRIMEN | CRS_CR_CEN;
}
