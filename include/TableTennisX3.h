// Define
//----------------------------------------------------------------



#define _GET_GPIO_PORT( pin ) ( GPIO_T* )( ( ( uint32_t )&pin ) & 0xFFFFFDC0 )
#define _GET_GPIO_PIN( pin )  ( ( ( ( uint32_t )&pin ) & 0x0000003F ) >> 2 )
#define _MASK( n )            ( 0x1u << n )

#define _ADC_CH0_PIN        PA0
#define _ADC_CH1_PIN        PA1
#define _ADC_CH2_PIN        PA2
#define _ADC_CH3_PIN        PA3
#define _ADC_CH4_PIN        PA4
#define _ADC_CH5_PIN        PA5
#define _ADC_CH6_PIN        PA6
#define _NRF_CE_PIN         PA8
#define _LED_R_PWM_PIN      PA12
#define _LED_G_PWM_PIN      PA13
#define _UART_RX_PIN        PA14
#define _UART_TX_PIN        PA15
#define _W25Q_SPI1_MOSI_PIN PB0
#define _W25Q_SPI1_MISO_PIN PB1
#define _W25Q_SPI1_CLK_PIN  PB2
#define _W25Q_SPI1_SS_PIN   PB3
#define _NRF_SPI2_SS_PIN    PB4
#define _NRF_SPI2_CLK_PIN   PB5
#define _NRF_SPI2_MISO_PIN  PB6
#define _NRF_SPI2_MOSI_PIN  PB7
#define _LED_B_PWM_PIN      PB11
#define _NRF_INT0_PIN       PB14
#define _ICM_INT1_PIN       PB15
#define _ICM_SPI0_SS_PIN    PC0
#define _ICM_SPI0_CLK_PIN   PC1
#define _ICM_SPI0_MISO_PIN  PC2
#define _ICM_SPI0_MOSI_PIN  PC3
#define _ADC_CH7_PIN        PC7
#define _POWER_PIN          PD6
#define _KEY0_PIN           PD7
#define _BAT_STDBY_PIN      PD14
#define _BAT_CHRG_PIN       PD15
#define _ICE_DAT_PIN        PF0
#define _ICE_CLK_PIN        PF1
#define _XT1_OUT_PIN        PF2
#define _XT1_IN_PIN         PF3
#define _X32O_PIN           PF6
#define _X32I_PIN           PF7
#define _ICM_FYNC_PIN       PE5

// Include
//----------------------------------------------------------------
// Configuration

#define _OPEN_DEBUG_PORT true
#define _DEBUG_PORT      UART0
#define _UART0_BAUD      57600
#define _SYS_TICK_Freq   10
#define _RTC_CALIBRATION 327655980

//----------------------------------------------------------------

#include "Nano103.h"
#include "Driver_MCU_SPI.h"
#include "Driver_MPU9250.h"
#include "Driver_ADC.h"
#include <stdbool.h>
#include <stdio.h>

// Variable
//----------------------------------------------------------------
// Application.c
void MCU_SysTickInit( float sysTickFrequency );
void MCU_PinInit( void );
void MCU_ClkInit( void );
void MCU_GpioInit( void );
void MCU_UartInit( void );
void MCU_SpiInit( void );
void MCU_RtcInit( S_RTC_TIME_DATA_T *sPt );
void MCU_NvicInit( void );
void MCU_AdcInit(void);
void MCU_SysTickDelayUs( uint32_t delayTime );
void DelayUs( uint32_t delayTime );
void PrintClockStatus( void );

//MPU9250
void MPU9250_Init(void);

//----------------------------------------------------------------
// boot.c
void MCU_Init(void);

//-----------------------------------------------------------------
extern DRV_SPI * spi0;
extern DRV_MPU9250 * mpu9250;
extern DRV_ADC * adc;
extern uint32_t adcTemp;
