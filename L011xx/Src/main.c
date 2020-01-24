#include<string.h>
#include"stm32l0xx.h"
#include"system_stm32l0xx.h"

#define TIMEOUT_1 (100U)

/* Private debugging flag */
#define DEBUG__ (1U)
#define SIMPLE__ (1U)

/* FPU guard */
#if !defined(__SOFT_FP__) && defined(__ARM_FP)
  #warning "FPU is not initialized, but the project is compiling for an FPU. Please initialize the FPU before use."
#endif


/* Debugging function to blink User LED and execute some code */
#if defined(DEBUG__)
#define blinked(x) \
	{ \
		LED_ON();\
		delay(30000);\
		x;\
		LED_OFF();\
		delay(30000);\
	}
#else
#define blinked(x) (x)
#endif


/* User LED: LD3 @ PB3 pin (UM 6.5) */
#define LED_PIN (3U)
#define LED_PORT (GPIOB)

/* Generic LED-controlling macros */
#define LED_ON() (LED_PORT->BSRR |= (1 << LED_PIN))
#define LED_OFF() (LED_PORT->BRR |= (1 << LED_PIN))
#define LED_TOG() (LED_PORT->ODR ^= (1 << LED_PIN))

#define RX_CAP (17U)


enum status
{
	OK,
	EXC_TIMEOUT
};

volatile int dma_stop=0;
volatile int done=0;

volatile uint8_t q_buffer[RX_CAP];
volatile int q_front=0,
		q_back=0,
		new_q_back=0,
		last_transfer=0,
		q_size=0,
		q_overflow=0,
		q_cap=RX_CAP;


void q_empty(void)
{
	DMA1_Channel3->CCR &= DMA_CCR_EN;
	DMA1_Channel3->CMAR = (uint32_t)q_buffer;
	q_front=q_back=new_q_back=last_transfer=q_size=q_overflow=0;
	DMA1_Channel3->CNDTR = (uint16_t)q_cap;
	new_q_back = q_cap-1;
	last_transfer = q_cap;
	DMA1_Channel3->CCR |= DMA_CCR_EN;
}

// #pragma not optimize
// inline
/* change to us using systemcoreclcok */
void delay(volatile unsigned n)
{
	while(--n);
}

void delay_ns(uint32_t ns)
{
	/* total_time_ns = scale*n*op*1e9 + off
	 * op = 1/SystemCoreClock
	 * total_time = scale*n*(1e9)/SystemCoreClock + off
	 * n = (total_time - off) / scale / 1e9 * SystemCoreClock
	 */
	__disable_irq();
	//uint32_t scale=1, offset=0;
	//uint32_t n = (ns - offset) / 1000000000 * SystemCoreClock / scale;

	while (ns-- >= 0);
	__enable_irq();
}

void LPUART1_Init(void)
{
	/* TX: PA1 (AF6)
	 * RX: PA0 (AF6)
	 */
	RCC->IOPENR |= RCC_IOPENR_IOPAEN;
	GPIOA->MODER &= ~(GPIO_MODER_MODE0 | GPIO_MODER_MODE1);
	GPIOA->MODER |= GPIO_MODER_MODE0_1 | GPIO_MODER_MODE1_1;
    GPIOA->AFR[0] &= ~(GPIO_AFRL_AFSEL0 | GPIO_AFRL_AFSEL1);
    GPIOA->AFR[0] |= (0x6) << GPIO_AFRL_AFSEL0_Pos;
    GPIOA->AFR[0] |= (0x6) << GPIO_AFRL_AFSEL1_Pos;

	/* Configuration:
	 * Clock: HSI16 (1)
	 * Word: 1-8-1
	 * Baudrate: 1M
	 * No parity bits
	 * Mode: TX, RX
	 * Swap TX and RX
	 * No hardware control
	 */
    RCC->CR |= RCC_CR_HSION;
    while ((RCC->CR & RCC_CR_HSIRDY) == 0);
	RCC->CCIPR &= ~RCC_CCIPR_LPUART1SEL;  /* (1) */
	RCC->CCIPR |= RCC_CCIPR_LPUART1SEL_1;
	RCC->APB1ENR |= RCC_APB1ENR_LPUART1EN;

	// LPUART1->CR1 &= ~LPUART_CR1_M;
	LPUART1->CR2 |= USART_CR2_SWAP;
	LPUART1->CR3 |= USART_CR3_OVRDIS;
	uint32_t f_ck, baud;
	f_ck = 16000000; // SystemCoreClock;
	baud = 1000000;
	LPUART1->BRR |= 256 * f_ck / baud;

	LPUART1->CR3 |= USART_CR3_DMAR | USART_CR3_DMAT;

	LPUART1->CR1 |= USART_CR1_UE;
	LPUART1->CR1 |= USART_CR1_RE;
	LPUART1->CR1 |= USART_CR1_TE;

	/* Clear Framing Error and Character Match flags */
	LPUART1->ICR |= USART_ICR_FECF;
	/* Poll idle frame transmission */
	while ((LPUART1->ISR & USART_ISR_TC) == 0);
	/* Clear TC flag after idle frame transmission */
    LPUART1->ICR |= USART_ICR_TCCF;

#if !defined(SIMPLE__)
    LPUART1->CR1 |= USART_CR1_RXNEIE;
    NVIC_EnableIRQ(LPUART1_IRQn);
    NVIC_SetPriority(LPUART1_IRQn, 0);
#endif
}


void DMA_Init(volatile uint8_t receiver_buf[])
{
	/* Errata: DMA channel 5 cannot be used for LPUART1 data reception
	 * Workaround: Use channel 3
	 */
	RCC->AHBENR |= RCC_AHBENR_DMA1EN;
	DMA1_CSELR->CSELR &= ~(DMA_CSELR_C2S | DMA_CSELR_C3S);
	DMA1_CSELR->CSELR |= ((0b0101 << DMA_CSELR_C2S_Pos) | (0b0101 << DMA_CSELR_C3S_Pos));

	/* Peripheral addresses: RDR, TDR registers of LPUART1 */
	DMA1_Channel2->CPAR = (uint32_t)(&LPUART1->TDR);
	DMA1_Channel3->CPAR = (uint32_t)(&LPUART1->RDR);
	DMA1_Channel3->CMAR = (uint32_t)(receiver_buf);
	/* Increment memory address */
	DMA1_Channel2->CCR |= DMA_CCR_MINC;
	DMA1_Channel3->CCR |= DMA_CCR_MINC;
	/* Transfer direction: memory to peripheral for TX, reversed for RX */
	DMA1_Channel2->CCR |= DMA_CCR_DIR;
	DMA1_Channel3->CCR &= ~DMA_CCR_DIR;

	/* Enable interrupts */
	DMA1_Channel2->CCR |= DMA_CCR_TCIE;
	DMA1_Channel3->CCR |= DMA_CCR_TCIE;
	NVIC_EnableIRQ(DMA1_Channel2_3_IRQn);
	NVIC_SetPriority(DMA1_Channel2_3_IRQn, 0);

	/* Enable the receiver */
	DMA1_Channel3->CNDTR = (uint16_t)q_cap;
	new_q_back = q_cap-1;
	last_transfer = q_cap;
	DMA1_Channel3->CCR |= DMA_CCR_EN;
}

int q_is_full(void)
{
	return q_size == q_cap;
}

int q_is_empty(void)
{
	return q_size == 0;
}

int dequeue(uint8_t *dest, uint32_t timeout)
{
	while(q_is_empty() && timeout--);
	if (!timeout) return 0;

	*dest = q_buffer[q_front];
	q_front = (q_front + 1) % q_cap;
	q_size--;

	return timeout;
}


#if defined(SIMPLE__)

int LPUART1_Transmit_Receive
(
		const uint8_t *src,
		uint8_t *dest,
		uint8_t ct,
		uint8_t cr,
		uint32_t timeout
)
{
	DMA1_Channel2->CCR &= ~DMA_CCR_EN;
	DMA1_Channel3->CCR &= ~DMA_CCR_EN;

	DMA1_Channel2->CMAR = (uint32_t)src;
	DMA1_Channel3->CMAR = (uint32_t)dest;

	DMA1_Channel2->CNDTR = ct;
	if (cr) DMA1_Channel3->CNDTR = cr;

	done = 0;
	DMA1_Channel2->CCR |= DMA_CCR_EN;
	if (cr) DMA1_Channel3->CCR |= DMA_CCR_EN;
	while(!done && timeout--);

	done=0;
	if (cr) while(!done && timeout--);
	return timeout;
}

#else

int LPUART1_Transmit_Receive
(
		const uint8_t *src,
		uint8_t *dest,
		uint8_t count_transmit,
		uint8_t count_receive,
		uint32_t timeout
)
{
	DMA1_Channel2->CCR &= ~DMA_CCR_EN;
	DMA1_Channel2->CMAR = (uint32_t)src;
	DMA1_Channel2->CNDTR = count_transmit;
	done = 0;
	DMA1_Channel2->CCR |= DMA_CCR_EN;
	while(!done && timeout--);

	for (int i=0; i < count_receive && timeout; ++i)
	{
		timeout = dequeue(dest++, timeout);
	}

	return timeout;
}

#endif

void I2C1_Init(void)
{
	/*
	 * Pins:
	 * 		in debug mode:	SCL: PB6 (AF1)
	 * 						SDA: PB7 (AF1)
	 * 		otherwise: 		SCL: PA9 (AF1)
	 * 						SDA: PA10 (AF1)
	 */

	/* Sequence: RM p. 216 */
#if defined(DEBUG__)
	RCC->IOPENR |= RCC_IOPENR_IOPBEN;
	GPIOB->MODER &= ~(GPIO_MODER_MODE6 | GPIO_MODER_MODE7);
	GPIOB->MODER |= GPIO_MODER_MODE6_1 | GPIO_MODER_MODE7_1;
    GPIOB->AFR[0] &= ~(GPIO_AFRL_AFSEL6 | GPIO_AFRL_AFSEL7);
    GPIOB->AFR[0] |= (0x1) << GPIO_AFRL_AFSEL6_Pos;
    GPIOB->AFR[0] |= (0x1) << GPIO_AFRL_AFSEL7_Pos;
    GPIOB->OTYPER |= GPIO_OTYPER_OT_6 | GPIO_OTYPER_OT_7;
#else
	RCC->IOPENR |= RCC_IOPENR_IOPAEN;
	GPIOA->MODER &= ~(GPIO_MODER_MODE9 | GPIO_MODER_MODE10);
	GPIOA->MODER |= GPIO_MODER_MODE9_1 | GPIO_MODER_MODE10_1;
	GPIOA->AFR[1] &= ~(GPIO_AFRH_AFSEL9 | GPIO_AFRH_AFSEL10);
	GPIOA->AFR[1] |= (0x1) << GPIO_AFRH_AFSEL9_Pos;
	GPIOA->AFR[1] |= (0x1) << GPIO_AFRH_AFSEL10_Pos;
	GPIOA->OTYPER |= GPIO_OTYPER_OT_9 | GPIO_OTYPER_OT_10;
#endif // defined(DEBUG__)

	/* Configuration:
	 * Mode: master
	 * Freq.: 100 kHz
	 * Source clock freq.: 16 MHz
	 * Filters: analog ON, digital OFF
	 * Rise time: 100ns
	 * Fall time: 10ns
	 * Addressing: 7 bit
	 *
	 * Source clock: HSI16
	 */
    RCC->CR |= RCC_CR_HSION;
    while ((RCC->CR & RCC_CR_HSIRDY) == 0)
    {
    	/* Add here timeout management */
    }

	RCC->CCIPR &= ~RCC_CCIPR_I2C1SEL;  /* (1) */
	RCC->CCIPR |= RCC_CCIPR_I2C1SEL_1;
	RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;

	/* Sequence: RM p. 590 */
	I2C1->TIMINGR |= 0x00503D5A;

    I2C1->OAR1 |= (uint32_t)(0x69 << 1);
    I2C1->OAR1 |= I2C_OAR1_OA1EN;

	I2C1->CR1 |= I2C_CR1_PE;
}


int I2C_Master_Receive(
		I2C_TypeDef *I2C,
		uint8_t slave_addr,
		uint8_t *buf,
		uint8_t count,
		uint32_t timeout
)
{
	/* Sequence: RM p. 611, 612 */
	I2C->CR2 = 0;
	while (I2C->ISR & I2C_ISR_BUSY && timeout--);
	if (!timeout) return 0;
	I2C->CR2 |= count << I2C_CR2_NBYTES_Pos;
	I2C->CR2 |= I2C_CR2_RD_WRN;
	I2C->CR2 |=  slave_addr;
	I2C->CR2 |= I2C_CR2_START;
	for (int i=0; i < count; ++i)
	{
		while ((I2C->ISR & I2C_ISR_RXNE) == 0 && timeout--);
		if (!timeout) return 0;
		*buf++ = I2C1->RXDR;
	}
	while ((I2C->ISR & I2C_ISR_TC) == 0 && timeout--);
	I2C->CR2 |= I2C_CR2_STOP;
	while (I2C->ISR & I2C_ISR_BUSY && timeout--);
	I2C->ICR |= I2C_ICR_STOPCF;
	return timeout;
}


int I2C_Master_Transmit(
		I2C_TypeDef *I2C,
		uint8_t slave_addr,
		volatile const uint8_t *buf,
		uint8_t count,
		uint32_t timeout
)
{
	// Sequence: RM p. 607, 608
	I2C1->CR2 = 0;
	while (I2C1->ISR & I2C_ISR_BUSY && timeout--);
	if (!timeout) return 0;
	I2C1->CR2 |= (count << I2C_CR2_NBYTES_Pos) | slave_addr;
	I2C1->CR2 |= I2C_CR2_START;

	if (I2C1->ISR & I2C_ISR_NACKF)
	{
		LED_ON();
		return 0;
	}

	for (int i=0; i < count; ++i)
	{
		while ((I2C1->ISR & I2C_ISR_TXE) == 0 && timeout--);
		if (!timeout) return 0;
		I2C1->TXDR = *buf++;
	}

	while ((I2C1->ISR & I2C_ISR_TC) == 0 && timeout--);
	I2C1->CR2 |= I2C_CR2_STOP;
	while (I2C1->ISR & I2C_ISR_BUSY && timeout--);
	I2C1->ICR |= I2C_ICR_STOPCF;
	return timeout;
}


void ADC_config(void)
{
	RCC->IOPENR |= RCC_IOPENR_IOPAEN;
	GPIOA->MODER |= GPIO_MODER_MODE0;

	RCC->APB2ENR |= RCC_APB2ENR_ADCEN;
	ADC1->CHSELR |= ADC_CHSELR_CHSEL0;

	ADC1->CFGR1 |= ADC_CFGR1_CONT;
	ADC1->CFGR1 |= ADC_CFGR1_OVRMOD;
	ADC1->CR |= ADC_CR_ADCAL;
	while (ADC1->CR & ADC_CR_ADCAL);

	ADC1->CR |= ADC_CR_ADEN;
	if (ADC1->ISR & ADC_ISR_ADRDY) {
	    ADC1->ISR |= ADC_ISR_ADRDY;
	}
    ADC1->CR |= ADC_CR_ADSTART;
    ADC->CCR |= ADC_CCR_VREFEN;
    while ((ADC1->ISR & ADC_ISR_ADRDY) == 0) {}
}


void USART_Receive (
		USART_TypeDef *USART,
		uint8_t *is_empty_flag,
		uint8_t *dest,
		uint8_t *src,
		uint8_t count)
{


}


void LTC_Init(void)
{
	uint8_t ltc_addr = 0x64 << 1;
	uint8_t tx_buf[] = {0};
	uint8_t rx_buf[64] = {0};
	I2C_Master_Transmit(I2C1, ltc_addr, tx_buf, sizeof(tx_buf), TIMEOUT_1);
	I2C_Master_Receive(I2C1, ltc_addr, rx_buf, sizeof(rx_buf), TIMEOUT_1);
}

void BQ_Init(void)
{
	RCC->IOPENR |= RCC_IOPENR_IOPCEN;
	GPIOC->MODER &= ~(GPIO_MODER_MODE15);
	GPIOC->MODER |= GPIO_MODER_MODE15_0;
	uint8_t buf[100];

	int f=12;
	int small_del=48, med_delay=1000*f, big_del=10000*f;

	uint8_t cmd0[] = {0xD0, 0x01, 0x1D, 0x00, 0x60, 0x74}; // broadcast write 0x00 to ecc_test 0x11d
	uint8_t cmd1[] = {0xD0, 0x00, 0x01, 0x00, 0x39, 0x74}; // broadcast write 0x00 to config 0x001
	uint8_t cmd2[] = {0xD0, 0x01, 0x05, 0x01, 0xAB, 0xB4}; // broadcast write 0x01 to control1 0x105
	uint8_t cmd3[] = {0xD0, 0x01, 0x04, 0x00, 0x6B, 0xE4}; // broadcast write addr=00 to devadd_usr[add] 0x104
	uint8_t cmd4[] = {0x90, 0x00, 0x00, 0x01, 0x00, 0xE5, 0x8D}; // single-dev-write 0x00  to config[stack_dev] 0x001
	uint8_t cmd5[] = {0x80, 0x00, 0x02, 0xBB, 0x01, 0x37, 0x2E}; // single-dev read 0x2bb from dev 0x00
	uint8_t cmd6[] = {0xD0, 0x00, 0x23, 0x56, 0xA1, 0xEA}; // set communication timeout to

	uint8_t init_cells[] = {0xD0, 0x01, 0x09, 0x7F, 0x2E, 0x94};
	uint8_t contin_conv[] = {0xD0, 0x00, 0x25, 0x08, 0x23, 0xB2};
	uint8_t adc_go[] = {0xD0, 0x01, 0x06, 0x81, 0xAA, 0xE4};

	// Send wake tone
	GPIOC->BRR |= (1 << 15);
	delay(small_del);
	GPIOC->BSRR |= (1 << 15);

	// Send init seq
	delay(big_del);
	LPUART1_Transmit_Receive(cmd0, buf, sizeof(cmd0), 0, TIMEOUT_1);
	LPUART1_Transmit_Receive(cmd1, buf, sizeof(cmd1), 0, TIMEOUT_1);
	LPUART1_Transmit_Receive(cmd2, buf, sizeof(cmd2), 0, TIMEOUT_1);
	LPUART1_Transmit_Receive(cmd3, buf, sizeof(cmd3), 0, TIMEOUT_1);
	LPUART1_Transmit_Receive(cmd4, buf, sizeof(cmd4), 0, TIMEOUT_1);
	LPUART1_Transmit_Receive(cmd5, buf, sizeof(cmd5), 0, TIMEOUT_1);

	// Set communication timeout to 30 min
	delay(med_delay);
	LPUART1_Transmit_Receive(cmd6, buf, sizeof(cmd6), 0, TIMEOUT_1);

	// Init all 6 cells
	delay(med_delay);
	LPUART1_Transmit_Receive(init_cells, buf, sizeof(init_cells), 0, TIMEOUT_1);

	// Enable continuous conversion
	delay(med_delay);
	LPUART1_Transmit_Receive(contin_conv, buf, sizeof(contin_conv), 0, TIMEOUT_1);

	// Start conversion
	delay(med_delay);
	LPUART1_Transmit_Receive(adc_go, buf, sizeof(adc_go), 0, TIMEOUT_1);
}

int main(void)
{
	RCC->IOPENR |= RCC_IOPENR_IOPBEN;
	GPIOB->MODER &= ~(GPIO_MODER_MODE3);
	GPIOB->MODER |= GPIO_MODER_MODE3_0;

	I2C1_Init();
	DMA_Init(q_buffer);
	LPUART1_Init();

#if !defined(DEBUG__)
	BQ_Init();
	LTC_Init();
	while(1);
#endif

	uint8_t buf_tx[8]="chujnia",
			buf_rx[8]={0};

	uint8_t slave=100;

	while (1)
	{
		q_empty();
		if (!LPUART1_Transmit_Receive(buf_tx, buf_rx, 8, 8, TIMEOUT_1)) continue;
		memset(buf_tx, 0, 8);
		if (!I2C_Master_Transmit(I2C1, slave, buf_rx, 8, TIMEOUT_1)) continue;
		if (!I2C_Master_Receive(I2C1, slave, buf_tx, 8, TIMEOUT_1)) continue;
	}
}

void AES_RNG_LPUART1_IRQHandler(void)
{
	if (LPUART1->ISR & USART_ISR_RXNE && LPUART1->ISR & USART_ISR_IDLE)
	{
		LPUART1->CR1 &= ~USART_CR1_RXNEIE;
		LPUART1->RDR;
	}
	if (LPUART1->ISR & USART_ISR_RXNE) {
		LPUART1->CR1 |= USART_CR1_IDLEIE;
		LPUART1->CR1 &= ~USART_CR1_RXNEIE;
	}
	else if (LPUART1->ISR & USART_ISR_IDLE)
	{
		LPUART1->CR1 &= ~USART_CR1_IDLEIE;
		dma_stop = 1;
		NVIC_SetPendingIRQ(DMA1_Channel2_3_IRQn);
	}
}

void DMA1_Channel2_3_IRQHandler(void)
{

#if !defined(SIMPLE__)
	if (DMA1->ISR & DMA_ISR_TCIF3 || dma_stop)
#else
	if (DMA1->ISR & DMA_ISR_TCIF3)
#endif
	{
		done = 1;
#if defined(SIMPLE__)
		{
			DMA1->IFCR |= DMA_IFCR_CTCIF3;
			return;
		}
#endif

		dma_stop = 0;

		DMA1_Channel3->CCR &= ~DMA_CCR_EN;
		int cap;
		int left = DMA1_Channel3->CNDTR;
		q_back = new_q_back - left;
		if (q_back < 0) q_back = q_cap - q_back;
		q_size += last_transfer - left;

		if (q_is_full())
		{
			q_overflow = 1;
			DMA1->IFCR |= DMA_IFCR_CTCIF3;
			return;
		}

		if (q_back >= q_front)
		{
			cap = q_cap - q_back - 1;
			if (cap == 0) // gotta rewind
			{
				DMA1_Channel3->CMAR = (uint32_t)q_buffer;
				DMA1_Channel3->CNDTR = q_front;
				last_transfer = q_front;
			}
			else
			{
				DMA1_Channel3->CMAR = (uint32_t)(&q_buffer[q_back + 1]);
				DMA1_Channel3->CNDTR = cap;
				last_transfer = cap;
			}
		}
		else
		{
			cap = q_front - q_back - 1;
			DMA1_Channel3->CMAR = (uint32_t)(&q_buffer[q_back + 1]);
			DMA1_Channel3->CNDTR = cap;
			last_transfer = cap;
		}

		LPUART1->CR1 |= USART_CR1_RXNEIE;
		DMA1->IFCR |= DMA_IFCR_CTCIF3;
		DMA1_Channel3->CCR |= DMA_CCR_EN;

	}
	else if (DMA1->ISR & DMA_ISR_TCIF2)
	{
		done = 1;
		DMA1->IFCR |= DMA_IFCR_CTCIF2;
	}
	else
	{
		LED_ON();
		while(1);
	}
}

