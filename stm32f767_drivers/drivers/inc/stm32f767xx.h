#ifndef INC_STM32F767XX_H_
#define INC_STM32F767XX_H_

#include <stdint.h>

#define __vo volatile

/*ARM Cortex Mx Processor NVIC ISERx register addresses*/

#define NVIC_ISER0 ((__vo uint32_t *)0xE000E100)
#define NVIC_ISER1 ((__vo uint32_t *)0xE000E104)
#define NVIC_ISER2 ((__vo uint32_t *)0xE000E108)
#define NVIC_ISER3 ((__vo uint32_t *)0xE000E10C)
#define NVIC_ISER4 ((__vo uint32_t *)0xE000E110)

/*ICER*/
#define NVIC_ICER0 ((__vo uint32_t *)0XE000E180)
#define NVIC_ICER1 ((__vo uint32_t *)0XE000E184)
#define NVIC_ICER2 ((__vo uint32_t *)0XE000E188)
#define NVIC_ICER3 ((__vo uint32_t *)0XE000E18C)
#define NVIC_ICER4 ((__vo uint32_t *)0XE000E190)

/*Priority Register*/
#define NVIC_PR_BASE_ADDR ((__vo uint32_t *)0xE000E400)

#define NUM_PR_BITS_IMPLEMENTED 4

/*end of NVIC ISERx*/

#define ENABLE         1
#define DISABLE        0
#define SET            ENABLE
#define RESET          DISABLE
#define GPIO_PIN_SET   SET
#define GPIO_PIN_RESET RESET

#define FLASH_BASEADDR 0x08000000U
#define SRAM_BASEADDR  0x20000000U

#define PERIPH_BASE       0x40000000U
#define APB1PERIPH_BASE   PERIPH_BASE
#define APB2PERIPH_BASE   0x40010000U
#define AHB1PERIPH_BASE   0x40020000U
#define AHB2PERIPH_BASE   0x50000000U

/*GPIO ON AHB1 BUS*/
#define GPIOA_BASEADDR (AHB1PERIPH_BASE + 0x0000U)
#define GPIOB_BASEADDR (AHB1PERIPH_BASE + 0x0400U)
#define GPIOC_BASEADDR (AHB1PERIPH_BASE + 0x0800U)
#define GPIOD_BASEADDR (AHB1PERIPH_BASE + 0x0C00U)
#define GPIOE_BASEADDR (AHB1PERIPH_BASE + 0x1000U)
#define GPIOF_BASEADDR (AHB1PERIPH_BASE + 0x1400U)
#define GPIOG_BASEADDR (AHB1PERIPH_BASE + 0x1800U)
#define GPIOH_BASEADDR (AHB1PERIPH_BASE + 0x1C00U)
#define GPIOI_BASEADDR (AHB1PERIPH_BASE + 0x2000U)
#define GPIOJ_BASEADDR (AHB1PERIPH_BASE + 0x2400U)
#define GPIOK_BASEADDR (AHB1PERIPH_BASE + 0x2800U)

#define GPIO_BASEADDR_TO_CODE(x) ((x == GPIOA)?0:\
                                 (x == GPIOB)?1:\
                                 (x == GPIOC)?2:\
                                 (x == GPIOD)?3:\
                                 (x == GPIOE)?4:\
                                 (x == GPIOF)?5:0)
/*RCC on AHB1 bus*/
#define RCC_BASEADDR   (AHB1PERIPH_BASE + 0x3800U)

/*APB1 BUS*/
#define I2C1_BASEADDR (APB1PERIPH_BASE +0x5400)
#define I2C2_BASEADDR (APB1PERIPH_BASE +0x5800)
#define I2C3_BASEADDR (APB1PERIPH_BASE +0x5C00)
#define I2C4_BASEADDR (APB1PERIPH_BASE +0x6000)

#define SPI2_BASEADDR (APB1PERIPH_BASE +0x3800)
#define SPI3_BASEADDR (APB1PERIPH_BASE +0x3C00)

#define USART2_BASEADDR (APB1PERIPH_BASE +0x4400)
#define USART3_BASEADDR (APB1PERIPH_BASE +0x4800)

#define UART4_BASEADDR (APB1PERIPH_BASE +0x4C00)
#define UART5_BASEADDR (APB1PERIPH_BASE +0x5000)

/*APB2 BUS*/
#define SPI1_BASEADDR (APB2PERIPH_BASE +0x3000)

#define USART1_BASEADDR (APB2PERIPH_BASE +0x1000)
#define USART6_BASEADDR (APB2PERIPH_BASE +0x1400)

#define EXTI_BASEADDR (APB2PERIPH_BASE +0x3C00)

#define SYSCFG_BASEADDR (APB2PERIPH_BASE +0x3800)


/*GPIO REG STRUCTURE*/
typedef struct _gpio_regdef_t
{
    __vo uint32_t MODER;
    __vo uint32_t OTYPER;
    __vo uint32_t OSPEEDR;
    __vo uint32_t PUPDR;
    __vo uint32_t IDR;
    __vo uint32_t ODR;
    __vo uint32_t BSRR;
    __vo uint32_t LCKR;
    __vo uint32_t AFR[2];
}GPIO_RegDef_t;

typedef struct _rcc_regdef_t
{
    __vo uint32_t CR;
    __vo uint32_t PLLCFGR;
    __vo uint32_t CFGR;
    __vo uint32_t CIR;
    __vo uint32_t AHB1RSTR;
    __vo uint32_t AHB2RSTR;
    __vo uint32_t AHB3RSTR;

    __vo uint32_t RESERVED_0;

    __vo uint32_t APB1RSTR;
    __vo uint32_t APB2RSTR;

    __vo uint32_t RESERVED_1;
    __vo uint32_t RESERVED_2;

    __vo uint32_t AHB1ENR;
    __vo uint32_t AHB2ENR;
    __vo uint32_t AHB3ENR;

    __vo uint32_t RESERVED_3;

    __vo uint32_t APB1ENR;
    __vo uint32_t APB2ENR;

    __vo uint32_t RESERVED_4;
    __vo uint32_t RESERVED_5;

    __vo uint32_t AHB1LPENR;
    __vo uint32_t AHB2LPENR;
    __vo uint32_t AHB3LPENR;

    __vo uint32_t RESERVED_6;

    __vo uint32_t APB1LPENR;
    __vo uint32_t APB2LPENR;

    __vo uint32_t RESERVED_7;
    __vo uint32_t RESERVED_8;

    __vo uint32_t BDCR;
    __vo uint32_t CSR;

    __vo uint32_t RESERVED_9;
    __vo uint32_t RESERVED_10;

    __vo uint32_t SSCGR;
    __vo uint32_t PLLI2SCFGR;
    __vo uint32_t PLLSAICFGR;
    __vo uint32_t DCKCFGR1;
    __vo uint32_t DCKCFGR2;
}RCC_RegDef_t;

typedef struct _exti_regdef_t
{
    __vo uint32_t IMR;
    __vo uint32_t EMR;
    __vo uint32_t RTSR;
    __vo uint32_t FTSR;
    __vo uint32_t SWIER;
    __vo uint32_t PR;
}EXTI_RegDef_t;

typedef struct _syscfg_regdef_t
{
    __vo uint32_t MEMRMP;
    __vo uint32_t PMC;
    __vo uint32_t EXTICR[4];
    __vo uint32_t RESERVED_0;
    __vo uint32_t CBR;
    __vo uint32_t CMPCR;
}SYSCFG_RegDef_t;

typedef struct _spi_regdef_t
{
    __vo uint32_t CR1;
    __vo uint32_t CR2;
    __vo uint32_t SR;
    __vo uint32_t DR;
    __vo uint32_t CRCPR;
    __vo uint32_t RXCRCR;
    __vo uint32_t TXCRCR;
    __vo uint32_t I2SCFGR;
    __vo uint32_t I2SPR;
}SPI_RegDef_t;

typedef struct _i2c_regdef_t
{
    __vo uint32_t CR1;
    __vo uint32_t CR2;
    __vo uint32_t OAR1;
    __vo uint32_t OAR2;
    __vo uint32_t TIMINGR;
    __vo uint32_t TIMOUTR;
    __vo uint32_t ISR;
    __vo uint32_t ICR;
    __vo uint32_t PECR;
    __vo uint32_t RXDR;
    __vo uint32_t TXDR;
}I2C_RegDef_t;

typedef struct _usart_regdef_t
{
    __vo uint32_t CR1;
    __vo uint32_t CR2;
    __vo uint32_t CR3;
    __vo uint32_t BRR;
    __vo uint32_t GTPR;
    __vo uint32_t RTOR;
    __vo uint32_t RQR;
    __vo uint32_t ISR;
    __vo uint32_t ICR;
    __vo uint32_t RDR;
    __vo uint32_t TDR;
}USART_RegDef_t;


#define SPI_READY      0
#define SPI_BUSY_IN_RX 1
#define SPI_BUSY_IN_TX 2

/*Possible SPI application events*/
#define SPI_EVENT_TX_CMPLT 1
#define SPI_EVENT_RX_CMPLT 2
#define SPI_EVENT_OVR_ERR  3
#define SPI_EVENT_CRC_ERR  4

#define GPIOA ((GPIO_RegDef_t *)GPIOA_BASEADDR)
#define GPIOB ((GPIO_RegDef_t *)GPIOB_BASEADDR)
#define GPIOC ((GPIO_RegDef_t *)GPIOC_BASEADDR)
#define GPIOD ((GPIO_RegDef_t *)GPIOD_BASEADDR)
#define GPIOE ((GPIO_RegDef_t *)GPIOE_BASEADDR)
#define GPIOF ((GPIO_RegDef_t *)GPIOF_BASEADDR)
#define GPIOG ((GPIO_RegDef_t *)GPIOG_BASEADDR)
#define GPIOH ((GPIO_RegDef_t *)GPIOH_BASEADDR)
#define GPIOI ((GPIO_RegDef_t *)GPIOI_BASEADDR)
#define GPIOJ ((GPIO_RegDef_t *)GPIOJ_BASEADDR)
#define GPIOK ((GPIO_RegDef_t *)GPIOK_BASEADDR)

#define RCC ((RCC_RegDef_t *)RCC_BASEADDR)

#define EXTI ((EXTI_RegDef_t *)EXTI_BASEADDR)

#define SYSCFG ((SYSCFG_RegDef_t *)SYSCFG_BASEADDR)

#define GPIOA_PCLK_EN() (RCC->AHB1ENR |= (1 << 0))
#define GPIOB_PCLK_EN() (RCC->AHB1ENR |= (1 << 1))
#define GPIOC_PCLK_EN() (RCC->AHB1ENR |= (1 << 2))
#define GPIOD_PCLK_EN() (RCC->AHB1ENR |= (1 << 3))

/*GPIO RESET*/
#define GPIOA_REG_RESET() do { (RCC->AHB1RSTR |= (1 << 0)); (RCC->AHB1RSTR &= ~(1 << 0)); } while(0)
#define GPIOB_REG_RESET() do { (RCC->AHB1RSTR |= (1 << 1)); (RCC->AHB1RSTR &= ~(1 << 1)); } while(0)
#define GPIOC_REG_RESET() do { (RCC->AHB1RSTR |= (1 << 2)); (RCC->AHB1RSTR &= ~(1 << 2)); } while(0)
#define GPIOD_REG_RESET() do { (RCC->AHB1RSTR |= (1 << 3)); (RCC->AHB1RSTR &= ~(1 << 3)); } while(0)
#define GPIOE_REG_RESET() do { (RCC->AHB1RSTR |= (1 << 4)); (RCC->AHB1RSTR &= ~(1 << 4)); } while(0)
#define GPIOF_REG_RESET() do { (RCC->AHB1RSTR |= (1 << 5)); (RCC->AHB1RSTR &= ~(1 << 5)); } while(0)
#define GPIOG_REG_RESET() do { (RCC->AHB1RSTR |= (1 << 6)); (RCC->AHB1RSTR &= ~(1 << 6)); } while(0)
#define GPIOH_REG_RESET() do { (RCC->AHB1RSTR |= (1 << 7)); (RCC->AHB1RSTR &= ~(1 << 7)); } while(0)
#define GPIOI_REG_RESET() do { (RCC->AHB1RSTR |= (1 << 8)); (RCC->AHB1RSTR &= ~(1 << 8)); } while(0)
#define GPIOJ_REG_RESET() do { (RCC->AHB1RSTR |= (1 << 9)); (RCC->AHB1RSTR &= ~(1 << 9)); } while(0)
#define GPIOK_REG_RESET() do { (RCC->AHB1RSTR |= (1 << 10)); (RCC->AHB1RSTR &= ~(1 << 10)); } while(0)

#define I2C1_PCLK_EN() (RCC->APB1ENR |= (1 << 21))
#define I2C2_PCLK_EN() (RCC->APB1ENR |= (1 << 22))
#define I2C3_PCLK_EN() (RCC->APB1ENR |= (1 << 23))



#define SPI1_PCLK_EN() (RCC->APB2ENR |= (1 << 12))

#define SPI2_PCLK_EN() (RCC->APB1ENR |= (1 << 14))
#define SPI3_PCLK_EN() (RCC->APB1ENR |= (1 << 15))

/*USART enable/disable*/
#define USART1_PCLK_EN() (RCC->APB2ENR |= (1 << 4))
#define USART2_PCLK_EN() (RCC->APB1ENR |= (1 << 17))
#define USART3_PCLK_EN() (RCC->APB1ENR |= (1 << 18))
#define USART6_PCLK_EN() (RCC->APB2ENR |= (1 << 5))

/*Add RESET macros later*/

#define SYSCFG_PCLK_EN() (RCC->APB2ENR |= (1 << 14))

/*CLOCK DISABLE*/
#define GPIOA_PCLK_DI() (RCC->AHB1ENR &= ~(1 << 0))



/*IRQ Numbers*/
#define IRQ_NUM_EXTI0 6
#define IRQ_NUM_EXTI1 7
#define IRQ_NUM_EXTI2 8
#define IRQ_NUM_EXTI3 9
#define IRQ_NUM_EXTI4 10
#define IRQ_NUM_EXTI9_5 23
#define IRQ_NUM_EXTI15_10 40

#define IRQ_NUM_SPI1 35
#define IRQ_NUM_SPI2 36
#define IRQ_NUM_SPI3 51

#define IRQ_NUM_SPI4 84
#define IRQ_NUM_SPI5 85
#define IRQ_NUM_SPI6 86

/*IRQ Priorities*/
#define NVIC_IRQ_PRI0  0
#define NVIC_IRQ_PRI15 15

/*SPI definitions*/
#define SPI1 ((SPI_RegDef_t*)SPI1_BASEADDR)
#define SPI2 ((SPI_RegDef_t*)SPI2_BASEADDR)
#define SPI3 ((SPI_RegDef_t*)SPI3_BASEADDR)

/*SPI Device Modes*/
#define SPI_DEVICE_MODE_MASTER 1
#define SPI_DEVICE_MODE_SLAVE  0

#define SPI_BUS_CONFIG_FD               1 //FULL DUPLEX
#define SPI_BUS_CONFIG_HD               2 //HALF DUPLEX
#define SPI_BUS_CONFIG_SIMPLEX_RX_ONLY  3

/*SPI Speed*/
#define SPI_SCLK_SPEED_DIV_2   0
#define SPI_SCLK_SPEED_DIV_4   1
#define SPI_SCLK_SPEED_DIV_8   2
#define SPI_SCLK_SPEED_DIV_16  3
#define SPI_SCLK_SPEED_DIV_32  4
#define SPI_SCLK_SPEED_DIV_64  5
#define SPI_SCLK_SPEED_DIV_128 6
#define SPI_SCLK_SPEED_DIV_256 7

/*DFF*/
#define SPI_DFF_8_BITS  0
#define SPI_DFF_16_BITs 1

/*CPOL*/
#define SPI_CPOL_HIGH 1
#define SPI_CPOL_LOW  0

/*CPHA*/
#define SPI_CPHA_HIGH 1
#define SPI_CPHA_LOW  0

/*SSM Software Slave Management*/
#define SPI_SSM_EN 1
#define SPI_SSM_DI 0

//bit positions of SPI CR1 register
#define SPI_CR1_CPHA_BIT 0
#define SPI_CR1_CPOL_BIT 1
#define SPI_CR1_MSTR_BIT 2
#define SPI_CR1_BAUD_RATE_CTRL_BIT 3
#define SPI_CR1_SPE_BIT 6
#define SPI_CR1_SSI_BIT 8
#define SPI_CR1_SSM_BIT 9
#define SPI_CR1_RXONLY_BIT   10
#define SPI_CR1_BIDIMODE_BIT 15

#define SPI_CR2_ERRIE_BIT  5
#define SPI_CR2_RXNEIE_BIT 6
#define SPI_CR2_TXEIE_BIT  7

#define SPI_SR_RXNE_BIT 0
#define SPI_SR_TXE_BIT  1
#define SPI_SR_OVR_BIT  6

/*I2C definitions*/
#define I2C1 ((I2C_RegDef_t *)I2C1_BASEADDR)
#define I2C2 ((I2C_RegDef_t *)I2C2_BASEADDR)
#define I2C3 ((I2C_RegDef_t *)I2C3_BASEADDR)

/*I2C CR1 bit positions*/
#define I2C_CR1_PE_BIT   0
#define I2C_CR1_TXIE_BIT 1
#define I2C_CR1_RXIE_BIT 2
#define I2C_CR1_ADDRIE_BIT 3
#define I2C_CR1_NACKIE_BIT 4
#define I2C_CR1_STOPIE_BIT 5
#define I2C_CR1_TCIE_BIT   6
#define I2C_CR1_ERRIE_BIT  7
#define I2C_CR1_DNF_POS_BIT 8
#define I2C_CR1_ANFOFF_BIT  12
#define I2C_CR1_SBC_BIT     16

/*I2C CR2 bit positions*/
#define I2C_CR2_RDWRN_BIT 10
#define I2C_CR2_ADD10_BIT 11
#define I2C_CR2_HEAD10_BIT 12
#define I2C_CR2_START_BIT 13
#define I2C_CR2_STOP_BIT  14
#define I2C_CR2_NACK_BIT  15
#define I2C_CR2_RELOAD_BIT 24
#define I2C_CR2_AUTOEND_BIT 25
#define I2C_CR2_PECBYTE_BIT 26

/*USART definitions*/
#define USART1 ((USART_RegDef_t *)USART1_BASEADDR)
#define USART2 ((USART_RegDef_t *)USART2_BASEADDR)
#define USART3 ((USART_RegDef_t *)USART3_BASEADDR)
#define USART6 ((USART_RegDef_t *)USART6_BASEADDR)

/*USART register bits*/
#define USART_CR1_UE_BIT     0
#define USART_CR1_UESM_BIT   1
#define USART_CR1_RE_BIT     2
#define USART_CR1_TE_BIT     3
#define USART_CR1_IDLEIE_BIT 4
#define USART_CR1_RXNEIE_BIT 5
#define USART_CR1_TCIE_BIT   6
#define USART_CR1_TXEIE_BIT  7
#define USART_CR1_PEIE_BIT   8
#define USART_CR1_PS_BIT     9
#define USART_CR1_PCE_BIT    10
#define USART_CR1_WAKE_BIT   11
#define USART_CR1_M0_BIT     12
#define USART_CR1_MME_BIT    13
#define USART_CR1_CMIE_BIT   14
#define USART_CR1_OVER8_BIT  15
#define USART_CR1_DEDT_BIT   16
#define USART_CR1_DEAT_BIT   21
#define USART_CR1_RTOIE_BIT  26
#define USART_CR1_EOBIE_BIT  27
#define USART_CR1_M1_BIT     28

#define USART_CR1_UE_BITMASK     (1 << USART_CR1_UE_BIT)
#define USART_CR1_UESM_BITMASK   (1 << USART_CR1_UESM_BIT)
#define USART_CR1_RE_BITMASK     (1 << USART_CR1_RE_BIT)
#define USART_CR1_TE_BITMASK     (1 << USART_CR1_TE_BIT)
#define USART_CR1_IDLEIE_BITMASK (1 << 4)
#define USART_CR1_RXNEIE_BITMASK (1 << 5)
#define USART_CR1_TCIE_BITMASK   (1 << 6)
#define USART_CR1_TXEIE_BITMASK  (1 << 7)
#define USART_CR1_PEIE_BITMASK   (1 << 8)
#define USART_CR1_PS_BITMASK     (1 << 9)
#define USART_CR1_PCE_BITMASK    (1 << 10)
#define USART_CR1_WAKE_BITMASK   (1 << 11)
#define USART_CR1_M0_BITMASK     (1 << 12)
#define USART_CR1_MME_BITMASK    (1 << 13)
#define USART_CR1_CMIE_BITMASK   (1 << 14)
#define USART_CR1_OVER8_BITMASK  (1 << 15)
#define USART_CR1_DEDT_BITMASK   (1 << 16)
#define USART_CR1_DEAT_BITMASK   (1 << 21)
#define USART_CR1_RTOIE_BITMASK  (1 << 26)
#define USART_CR1_EOBIE_BITMASK  (1 << 27)
#define USART_CR1_M1_BITMASK     (1 << 28)

/*Interrupt and status register bits*/
#define USART_ISR_PE_BIT 0
#define USART_ISR_FE_BIT 1
#define USART_ISR_NF_BIT 2
#define USART_ISR_ORE_BIT 3
#define USART_ISR_IDLE_BIT 4
#define USART_ISR_RXNE_BIT 5
#define USART_ISR_TC_BIT 6
#define USART_ISR_TXE_BIT 7
#define USART_ISR_LBDF_BIT 8
#define USART_ISR_CTSIF_BIT 9
#define USART_ISR_CTS_BIT 10
#define USART_ISR_RTOF_BIT 11
#define USART_ISR_EOBF_BIT 12
#define USART_ISR_ABRE_BIT 14
#define USART_ISR_ABRF_BIT 15
#define USART_ISR_BUSY_BIT 16
#define USART_ISR_CMF_BIT 17
#define USART_ISR_SBKF_BIT 18
#define USART_ISR_RWU_BIT 19
#define USART_ISR_WUF_BIT 20
#define USART_ISR_TEACK_BIT 21
#define USART_ISR_REACK_BIT 22
#define USART_ISR_TCBGT_BIT 25

#define USART_ISR_PE_BITMASK   (1 << 0)
#define USART_ISR_FE_BITMASK   (1 << 1)
#define USART_ISR_NF_BITMASK   (1 << 2)
#define USART_ISR_ORE_BITMASK  (1 << 3)
#define USART_ISR_IDLE_BITMASK (1 << 4)
#define USART_ISR_RXNE_BITMASK (1 << 5)
#define USART_ISR_TC_BITMASK   (1 << 6)
#define USART_ISR_TXE_BITMASK  (1 << 7)
#define USART_ISR_LBDF_BITMASK (1 << 8)
#define USART_ISR_CTSIF_BITMASK (1 << 9)
#define USART_ISR_CTS_BITMASK  (1 << 10)
#define USART_ISR_RTOF_BITMASK (1 << 11)
#define USART_ISR_EOBF_BITMASK (1 << 12)
#define USART_ISR_ABRE_BITMASK (1 << 14)
#define USART_ISR_ABRF_BITMASK (1 << 15)
#define USART_ISR_BUSY_BITMASK (1 << 16)
#define USART_ISR_CMF_BITMASK  (1 << 17)
#define USART_ISR_SBKF_BITMASK (1 << 18)
#define USART_ISR_RWU_BITMASK  (1 << 19)
#define USART_ISR_WUF_BITMASK  (1 << 20)
#define USART_ISR_TEACK_BITMASK (1 << 21)
#define USART_ISR_REACK_BITMASK (1 << 22)
#define USART_ISR_TCBGT_BITMASK (1 << 25)



#endif /* INC_STM32F767XX_H_ */
