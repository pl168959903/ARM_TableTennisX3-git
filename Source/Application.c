

#include "TableTennisX3.h"

//**************************************************************

DRV_SPI *     spi0;
DRV_SPI *     spi2;
DRV_MPU9250 * mpu9250;
DRV_NRF *     nrf;
DRV_ADC *     adc;
DRV_PRESSURE *pre;
uint32_t      adcTemp = 0;

void NRF_SetCE( uint8_t en ) {
    if ( en ) { _NRF_CE_PIN = 1; }
    else {
        _NRF_CE_PIN = 0;
    }
}
void NRF_Init( void ) {
    nrf = DRV_nrfNew();
    nrf->Initialize( nrf, ( DRV_NRF_PARAMS ){ spi2, NRF_SetCE, 60, 3, 1, 3, 15, 15 } );
    GPIO_EnableInt( _GET_GPIO_PORT( _NRF_INT0_PIN ), _GET_GPIO_PIN( _NRF_INT0_PIN ), GPIO_INT_FALLING );
}
void MPU9250_Init( void ) {
    mpu9250 = DRV_MPU9250New( spi0 );
    DRV_MPU9250_RESET( mpu9250 );
    DelayUs( 100000 );
    GPIO_EnableInt( _GET_GPIO_PORT( _ICM_INT1_PIN ), _GET_GPIO_PIN( _ICM_INT1_PIN ), GPIO_INT_FALLING );
    mpu9250->Initialize( mpu9250, ( DRV_MPU9250_PARAMS ){ 24, 4, 4, 3, 3 } );
}
void MCU_SysTickDelayUs( uint32_t delayTime ) {
    uint32_t tempLoad = SysTick->LOAD;
    uint32_t tempCtrl = SysTick->CTRL;
    CLK_SysTickDelay( delayTime );
    SysTick->LOAD = tempLoad;
    SysTick->CTRL = tempCtrl;
}
void MCU_SysTickInit( float sysTickFrequency ) {
    uint32_t clksrc = CLK_GetHCLKFreq();
    if ( ( ( float )clksrc / sysTickFrequency ) > 0xFFFFFF ) {
        SysTick->CTRL &= ~SysTick_CTRL_CLKSOURCE_Msk;
        SysTick->LOAD = ( uint32_t )( ( float )( clksrc >> 3 ) / sysTickFrequency );
    }
    else {
        SysTick->CTRL |= SysTick_CTRL_CLKSOURCE_Msk;
        SysTick->LOAD = ( uint32_t )( ( float )clksrc / sysTickFrequency );
    }
    SysTick->VAL = 0;
    SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk | SysTick_CTRL_TICKINT_Msk;
}
void MCU_PinInit( void ) {
    // Pin fuction
    //----------------------------------------------------------------
    SYS->GPA_MFPL |=
        ( SYS_GPA_MFPL_PA0MFP_ADC_CH0 |  //
          SYS_GPA_MFPL_PA1MFP_ADC_CH1 |  //
          SYS_GPA_MFPL_PA2MFP_ADC_CH2 |  //
          SYS_GPA_MFPL_PA3MFP_ADC_CH3 |  //
          SYS_GPA_MFPL_PA4MFP_ADC_CH4 |  //
          SYS_GPA_MFPL_PA5MFP_ADC_CH5 |  //
          SYS_GPA_MFPL_PA6MFP_ADC_CH6    //
        );
    SYS->GPA_MFPH |=
        ( SYS_GPA_MFPH_PA8MFP_GPIO |        //
          SYS_GPA_MFPH_PA9MFP_GPIO |        // useless
          SYS_GPA_MFPH_PA10MFP_GPIO |       // useless
          SYS_GPA_MFPH_PA11MFP_GPIO |       // useless
          SYS_GPA_MFPH_PA12MFP_PWM0_CH0 |   //
          SYS_GPA_MFPH_PA13MFP_PWM0_CH1 |   //
          SYS_GPA_MFPH_PA14MFP_UART0_RXD |  //
          SYS_GPA_MFPH_PA15MFP_UART0_TXD    //
        );
    SYS->GPB_MFPL |=
        ( SYS_GPB_MFPL_PB0MFP_SPI1_MOSI0 |  //
          SYS_GPB_MFPL_PB1MFP_SPI1_MISO0 |  //
          SYS_GPB_MFPL_PB2MFP_SPI1_CLK |    //
          SYS_GPB_MFPL_PB3MFP_SPI1_SS0 |    //
          SYS_GPB_MFPL_PB4MFP_SPI2_SS0 |    //
          SYS_GPB_MFPL_PB5MFP_SPI2_CLK |    //
          SYS_GPB_MFPL_PB6MFP_SPI2_MISO0 |  //
          SYS_GPB_MFPL_PB7MFP_SPI2_MOSI0    //
        );
    SYS->GPB_MFPH |=
        ( SYS_GPB_MFPH_PB8MFP_GPIO |       // useless
          SYS_GPB_MFPH_PB9MFP_GPIO |       // useless
          SYS_GPB_MFPH_PB10MFP_GPIO |      // useless
          //SYS_GPB_MFPH_PB11MFP_PWM0_CH4 |  //
          SYS_GPB_MFPH_PB11MFP_GPIO |  //
          SYS_GPB_MFPH_PB13MFP_GPIO |      // useless
          SYS_GPB_MFPH_PB14MFP_GPIO |      //
          SYS_GPB_MFPH_PB15MFP_GPIO        //
        );
    SYS->GPC_MFPL |=
        ( SYS_GPC_MFPL_PC0MFP_SPI0_SS0 |    //
          SYS_GPC_MFPL_PC1MFP_SPI0_CLK |    //
          SYS_GPC_MFPL_PC2MFP_SPI0_MISO0 |  //
          SYS_GPC_MFPL_PC3MFP_SPI0_MOSI0 |  //
          SYS_GPC_MFPL_PC6MFP_GPIO |        // useless
          SYS_GPC_MFPL_PC7MFP_ADC_CH7       //
        );
    SYS->GPC_MFPH |=
        ( SYS_GPC_MFPH_PC8MFP_GPIO |   // useless
          SYS_GPC_MFPH_PC9MFP_GPIO |   // useless
          SYS_GPC_MFPH_PC10MFP_GPIO |  // useless
          SYS_GPC_MFPH_PC11MFP_GPIO |  // useless
          SYS_GPC_MFPH_PC14MFP_GPIO |  // useless
          SYS_GPC_MFPH_PC15MFP_GPIO    // useless
        );
    SYS->GPD_MFPL |=
        ( SYS_GPD_MFPL_PD6MFP_GPIO |  //
          SYS_GPD_MFPL_PD7MFP_GPIO    //
        );
    SYS->GPD_MFPH |=
        ( SYS_GPD_MFPH_PD14MFP_GPIO |  //
          SYS_GPD_MFPH_PD15MFP_GPIO    //
        );
    SYS->GPE_MFPL |= ( SYS_GPE_MFPL_PE5MFP_GPIO  //
    );
    SYS->GPF_MFPL |=
        ( SYS_GPF_MFPL_PF0MFP_ICE_DAT |  //
          SYS_GPF_MFPL_PF1MFP_ICE_CLK |  //
          SYS_GPF_MFPL_PF2MFP_XT1_OUT |  //
          SYS_GPF_MFPL_PF3MFP_XT1_IN |   //
          SYS_GPF_MFPL_PF6MFP_X32_OUT |  //
          SYS_GPF_MFPL_PF7MFP_X32_IN     //
        );
}
void MCU_GpioInit( void ) {
    // Disable  digital signal input;
    GPIO_DISABLE_DIGITAL_PATH( _GET_GPIO_PORT( _ADC_CH0_PIN ), _MASK( _GET_GPIO_PIN( _ADC_CH0_PIN ) ) );
    GPIO_DISABLE_DIGITAL_PATH( _GET_GPIO_PORT( _ADC_CH1_PIN ), _MASK( _GET_GPIO_PIN( _ADC_CH1_PIN ) ) );
    GPIO_DISABLE_DIGITAL_PATH( _GET_GPIO_PORT( _ADC_CH2_PIN ), _MASK( _GET_GPIO_PIN( _ADC_CH2_PIN ) ) );
    GPIO_DISABLE_DIGITAL_PATH( _GET_GPIO_PORT( _ADC_CH3_PIN ), _MASK( _GET_GPIO_PIN( _ADC_CH3_PIN ) ) );
    GPIO_DISABLE_DIGITAL_PATH( _GET_GPIO_PORT( _ADC_CH4_PIN ), _MASK( _GET_GPIO_PIN( _ADC_CH4_PIN ) ) );
    GPIO_DISABLE_DIGITAL_PATH( _GET_GPIO_PORT( _ADC_CH5_PIN ), _MASK( _GET_GPIO_PIN( _ADC_CH5_PIN ) ) );
    GPIO_DISABLE_DIGITAL_PATH( _GET_GPIO_PORT( _ADC_CH6_PIN ), _MASK( _GET_GPIO_PIN( _ADC_CH6_PIN ) ) );
    GPIO_DISABLE_DIGITAL_PATH( _GET_GPIO_PORT( _ADC_CH7_PIN ), _MASK( _GET_GPIO_PIN( _ADC_CH7_PIN ) ) );
    GPIO_DISABLE_DIGITAL_PATH( _GET_GPIO_PORT( _XT1_OUT_PIN ), _MASK( _GET_GPIO_PIN( _XT1_OUT_PIN ) ) );
    GPIO_DISABLE_DIGITAL_PATH( _GET_GPIO_PORT( _XT1_IN_PIN ), _MASK( _GET_GPIO_PIN( _XT1_IN_PIN ) ) );
    GPIO_DISABLE_DIGITAL_PATH( _GET_GPIO_PORT( _X32O_PIN ), _MASK( _GET_GPIO_PIN( _X32O_PIN ) ) );
    GPIO_DISABLE_DIGITAL_PATH( _GET_GPIO_PORT( _X32I_PIN ), _MASK( _GET_GPIO_PIN( _X32I_PIN ) ) );

    // Debounce
    GPIO_SET_DEBOUNCE_TIME( GPIO_DBCLKSRC_IRC10K, GPIO_DBCLKSEL_128 );  // Debounce clock 10K*128 = 78.125Hz ; 12.8ms
    GPIO_ENABLE_DEBOUNCE( _GET_GPIO_PORT( _KEY0_PIN ), _MASK( _GET_GPIO_PIN( _KEY0_PIN ) ) );
    GPIO_ENABLE_DEBOUNCE( _GET_GPIO_PORT( _BAT_CHRG_PIN ), _MASK( _GET_GPIO_PIN( _BAT_CHRG_PIN ) ) );
    GPIO_ENABLE_DEBOUNCE( _GET_GPIO_PORT( _BAT_STDBY_PIN ), _MASK( _GET_GPIO_PIN( _BAT_STDBY_PIN ) ) );

    // GPIO mode;
    GPIO_SetMode( _GET_GPIO_PORT( _NRF_INT0_PIN ), _MASK( _GET_GPIO_PIN( _NRF_INT0_PIN ) ), GPIO_PMD_INPUT );
    GPIO_SetMode( _GET_GPIO_PORT( _ICM_INT1_PIN ), _MASK( _GET_GPIO_PIN( _ICM_INT1_PIN ) ), GPIO_PMD_INPUT );
    GPIO_SetMode( _GET_GPIO_PORT( _BAT_STDBY_PIN ), _MASK( _GET_GPIO_PIN( _BAT_STDBY_PIN ) ), GPIO_PMD_INPUT );
    GPIO_SetMode( _GET_GPIO_PORT( _BAT_CHRG_PIN ), _MASK( _GET_GPIO_PIN( _BAT_CHRG_PIN ) ), GPIO_PMD_INPUT );
    GPIO_SetMode( _GET_GPIO_PORT( _KEY0_PIN ), _MASK( _GET_GPIO_PIN( _KEY0_PIN ) ), GPIO_PMD_INPUT );
    GPIO_SetMode( _GET_GPIO_PORT( _NRF_CE_PIN ), _MASK( _GET_GPIO_PIN( _NRF_CE_PIN ) ), GPIO_PMD_OUTPUT );
    GPIO_SetMode( _GET_GPIO_PORT( _POWER_PIN ), _MASK( _GET_GPIO_PIN( _POWER_PIN ) ), GPIO_PMD_OUTPUT );
    GPIO_SetMode( _GET_GPIO_PORT( _ICM_FYNC_PIN ), _MASK( _GET_GPIO_PIN( _ICM_FYNC_PIN ) ), GPIO_PMD_OUTPUT );

    GPIO_SetMode( _GET_GPIO_PORT( _LED_B_PWM_PIN ), _MASK( _GET_GPIO_PIN( _LED_B_PWM_PIN ) ), GPIO_PMD_OUTPUT );

    // pull up
    GPIO_ENABLE_PULL_UP( _GET_GPIO_PORT( _BAT_CHRG_PIN ), _MASK( _GET_GPIO_PIN( _BAT_CHRG_PIN ) ) );
    GPIO_ENABLE_PULL_UP( _GET_GPIO_PORT( _BAT_STDBY_PIN ), _MASK( _GET_GPIO_PIN( _BAT_STDBY_PIN ) ) );
    GPIO_ENABLE_PULL_UP( _GET_GPIO_PORT( _ICM_INT1_PIN ), _MASK( _GET_GPIO_PIN( _ICM_INT1_PIN ) ) );

    // GPIO default value
    _NRF_CE_PIN   = 0;
    _POWER_PIN    = 0;
    _ICM_FYNC_PIN = 0;
    _LED_B_PWM_PIN = 1;
}
void MCU_ClkInit( void ) {
    //----------------------------------------------------------------
    // Enable Clock
    // LXT
    CLK_EnableXtalRC( CLK_PWRCTL_LXTEN_Msk );
    while ( !CLK_WaitClockReady( CLK_STATUS_LXTSTB_Msk ) ) {};

    // HXT
    CLK_EnableXtalRC( CLK_PWRCTL_HXTEN_Msk );
    while ( !CLK_WaitClockReady( CLK_STATUS_HXTSTB_Msk ) ) {};

    // MIRC
    CLK_EnableXtalRC( CLK_PWRCTL_MIRCEN_Msk );
    while ( !CLK_WaitClockReady( CLK_STATUS_MIRCSTB_Msk ) ) {};

    // HIRC0
    CLK_EnableXtalRC( CLK_PWRCTL_HIRC0EN_Msk );
    while ( !CLK_WaitClockReady( CLK_STATUS_HIRC0STB_Msk ) ) {};

    // HIRC1
    CLK_EnableXtalRC( CLK_PWRCTL_HIRC1EN_Msk );
    while ( !CLK_WaitClockReady( CLK_STATUS_HIRC1STB_Msk ) ) {};

    // LIRC
    CLK_EnableXtalRC( CLK_PWRCTL_LIRCEN_Msk );
    while ( !CLK_WaitClockReady( CLK_STATUS_LIRCSTB_Msk ) ) {};

    //----------------------------------------------------------------
    // Trim the clock.
    SYS->IRC0TCTL |= ( SYS_IRCTCTL_LOOP_32CLK | SYS_IRCTCTL_RETRY_512 );
    SYS->IRC1TCTL |= ( SYS_IRCTCTL_LOOP_32CLK | SYS_IRCTCTL_RETRY_512 );
    SYS->MIRCTCTL |= ( SYS_IRCTCTL_LOOP_32CLK | SYS_IRCTCTL_RETRY_512 );

    SYS_EnableHIRC0Trim( SYS_IRC0TCTL_TRIM_11_0592M, ( SYS_IRCTIEN_FAIL_EN | SYS_IRCTIEN_32KERR_EN ) );
    while ( !( SYS_GET_IRC0TRIM_INT_FLAG() & SYS_IRCTISTS_FREQLOCK ) ) {};

    SYS_EnableHIRC1Trim( SYS_IRC1TCTL_TRIM_36M, ( SYS_IRCTIEN_FAIL_EN | SYS_IRCTIEN_32KERR_EN ) );
    while ( !( SYS_GET_IRC1TRIM_INT_FLAG() & SYS_IRCTISTS_FREQLOCK ) ) {};

    SYS_EnableMIRCTrim( SYS_MIRCTCTL_TRIM_4M, ( SYS_IRCTIEN_FAIL_EN | SYS_IRCTIEN_32KERR_EN ) );
    while ( !( SYS_GET_MIRCTRIM_INT_FLAG() & SYS_IRCTISTS_FREQLOCK ) ) {};

    //----------------------------------------------------------------
    // Setting BUS clock
    CLK->CLKSEL0 |= CLK_CLKSEL0_HIRCSEL_Msk;  // Select HIRC clock source from HIRC0.

    CLK_SetHCLK( CLK_CLKSEL0_HCLKSEL_HIRC1, CLK_HCLK_CLK_DIVIDER( 1 ) );
    CLK_SetPCLK0( CLK_APB0DIV_HCLK );
    CLK_SetPCLK1( CLK_APB1DIV_HCLK );

    //----------------------------------------------------------------
    // Enable Clock Error interrupt
    CLK->CDUPB  = 0x156;  // HXT upper limit clock.
    CLK->CDLOWB = 0x152;  // HXT lower limit clock.
    CLK->CLKDCTL |= 0x7;  // Clock monitoring enabled.
    CLK->CLKDIE |= 0x7;   // Clock error interrupt enabled.
}
void MCU_UartInit( void ) {
#if _OPEN_DEBUG_PORT
    CLK_SetModuleClock( UART0_MODULE, CLK_CLKSEL1_UART0SEL_HIRC, CLK_UART0_CLK_DIVIDER( 1 ) );
    CLK_EnableModuleClock( UART0_MODULE );
    SYS_ResetModule( UART0_RST );
    while ( SYS->IPRST2 & SYS_IPRST2_UART0RST_Msk ) {};
    UART_Open( UART0, _UART0_BAUD );
    printf( "Debug serial port has been opened!!\n" );
#endif
}
void MCU_RtcInit( S_RTC_TIME_DATA_T *sPt ) {
    CLK_EnableModuleClock( RTC_MODULE );
    RTC_Open( sPt );
    RTC_32KCalibration( _RTC_CALIBRATION );  // 328660000
}
void MCU_SpiInit( void ) {
    CLK_SetModuleClock( SPI0_MODULE, CLK_CLKSEL1_SPI0SEL_HCLK, NULL );
    CLK_EnableModuleClock( SPI0_MODULE );
    spi0 = DRV_SpiNew( ( DRV_SPI_PARAMS ){ SPI0, 0, 3, 8, 1000000 } );
    spi0->Initialize( spi0 );

    CLK_SetModuleClock( SPI2_MODULE, CLK_CLKSEL1_SPI2SEL_HCLK, NULL );
    CLK_EnableModuleClock( SPI2_MODULE );
    spi2 = DRV_SpiNew( ( DRV_SPI_PARAMS ){ SPI2, 0, 0, 8, 1000000 } );
    spi2->Initialize( spi2 );
}
void MCU_AdcInit( void ) {

    CLK_SetModuleClock( ADC_MODULE, CLK_CLKSEL1_ADCSEL_HCLK, CLK_ADC_CLK_DIVIDER( 1 ) );
    CLK_EnableModuleClock( ADC_MODULE );

    SYS->IVREFCTL |= SYS_IVREFCTL_EXTMODE_Msk | SYS_IVREFCTL_SEL25_Msk | SYS_IVREFCTL_REGEN_Msk | SYS_IVREFCTL_BGPEN_Msk;

    adc = DRV_AdcNew();
    adc->Initialize( adc, ( DRV_ADC_PARAMS ){ ADC, ADC_RESSEL_12_BIT, ADC_REFSEL_INT_VREF, ADC_INPUT_MODE_SINGLE_END, ADC_OPERATION_MODE_CONTINUOUS } );
    adc->ChannelEnable( adc, 0, 9 );
    adc->ChannelEnable( adc, 1, 9 );
    adc->ChannelEnable( adc, 2, 9 );
    adc->ChannelEnable( adc, 3, 9 );
    adc->ChannelEnable( adc, 4, 9 );
    adc->ChannelEnable( adc, 5, 9 );
    adc->ChannelEnable( adc, 6, 9 );
    adc->ChannelEnable( adc, 7, 9 );
    adc->Conversion( adc );

    pre = DRV_PressureNew( ( DRV_PRESSURE_PARAMS ){ adc } );
    pre->SetChannel( pre, 0, 0, 0, 2600, 2500 );
    pre->SetChannel( pre, 1, 1, 0, 2600, 2500 );
    pre->SetChannel( pre, 2, 2, 0, 2600, 2500 );
    pre->SetChannel( pre, 3, 3, 0, 2600, 2500 );
    pre->SetChannel( pre, 4, 4, 0, 2600, 2500 );
    pre->SetChannel( pre, 5, 5, 0, 2600, 2500 );
    pre->SetChannel( pre, 6, 6, 0, 2600, 2500 );
    pre->SetChannel( pre, 7, 7, 0, 2600, 2500 );
}
void MCU_NvicInit( void ) {
    NVIC_EnableIRQ( GPABC_IRQn );
    NVIC_EnableIRQ( ADC_IRQn );
}
// ------------------------------------------------
// Function
void DelayUs( uint32_t delayTime ) {
    do {
        uint32_t t = ( delayTime > ( 0x1000000 / ( CLK_GetHCLKFreq() / 1000000 ) ) ) ? ( 0x1000000 / ( CLK_GetHCLKFreq() / 1000000 ) ) : delayTime;
        MCU_SysTickDelayUs( t );
        delayTime -= t;
    } while ( delayTime );
}
void PrintClockStatus( void ) {
    printf( "--------------------------------\n" );
    printf( "Clock config : \n" );
    printf( "HXT : %d Hz\n", CLK_GetHXTFreq() );
    printf( "LXT : %d Hz\n", CLK_GetLXTFreq() );
    printf( "HIRC0 : %d Hz\n", CLK_GetHIRC0Freq() );
    printf( "HIRC1 : %d Hz\n", CLK_GetHIRC1Freq() );
    printf( "HIRC : %d Hz\n", CLK_GetHIRCFreq() );
    printf( "MIRC : %d Hz\n", CLK_GetMIRCFreq() );
    printf( "LIRC : %d Hz\n", CLK_GetLIRCFreq() );
    printf( "CPU : %d Hz\n", CLK_GetCPUFreq() );
    printf( "HCLK : %d Hz\n", CLK_GetHCLKFreq() );
    printf( "PCLK0 : %d Hz\n", CLK_GetPCLK0Freq() );
    printf( "PCLK1 : %d Hz\n", CLK_GetPCLK1Freq() );
    printf( "PLL : %d Hz\n", CLK_GetPLLClockFreq() );
}
// ------------------------------------------------
// IRQHandler
void GPABC_IRQHandler( void ) {
    if ( GPIO_GET_INT_FLAG( _GET_GPIO_PORT( _ICM_INT1_PIN ), _MASK( _GET_GPIO_PIN( _ICM_INT1_PIN ) ) ) ) {
        uint8_t intStatus = DRV_MPU9250_GET_INT_STATUS( mpu9250 );
        if ( intStatus & _DRV_MPU9250_INT_STATUS_WON_INT_Msk ) mpu9250->Event_WakeOnMotion( mpu9250 );
        if ( intStatus & _DRV_MPU9250_INT_STATUS_FIFO_OVERFLOW_INT_Msk ) mpu9250->Event_FifoOverflow( mpu9250 );
        if ( intStatus & _DRV_MPU9250_INT_STATUS_FSYNC_INT_INT_Msk ) mpu9250->Event_Fsync( mpu9250 );
        if ( intStatus & _DRV_MPU9250_INT_STATUS_RAW_RDY_INT_Msk ) mpu9250->Event_RawDataReady( mpu9250 );

        GPIO_CLR_INT_FLAG( _GET_GPIO_PORT( _ICM_INT1_PIN ), _MASK( _GET_GPIO_PIN( _ICM_INT1_PIN ) ) );
    }
    if ( GPIO_GET_INT_FLAG( _GET_GPIO_PORT( _NRF_INT0_PIN ), _MASK( _GET_GPIO_PIN( _NRF_INT0_PIN ) ) ) ) {
        uint8_t intStatus = DRV_NRF_NOP( nrf );
        if ( intStatus & _DRV_NRF_REG_STATUS_TX_DS_MSK ) nrf->Event_TX_DS( nrf );
        if ( intStatus & _DRV_NRF_REG_STATUS_RX_DR_MSK ) nrf->Event_RX_DR( nrf );
        if ( intStatus & _DRV_NRF_REG_STATUS_MAX_RT_MSK ) nrf->Event_MAX_RT( nrf );
        GPIO_CLR_INT_FLAG( _GET_GPIO_PORT( _NRF_INT0_PIN ), _MASK( _GET_GPIO_PIN( _NRF_INT0_PIN ) ) );
    }
}
void GPDEF_IRQHandler( void ) {
    if ( GPIO_GET_INT_FLAG( _GET_GPIO_PORT( _KEY0_PIN ), _MASK( _GET_GPIO_PIN( _KEY0_PIN ) ) ) ) {
        if ( _KEY0_PIN )
            printf( "Button up\n" );
        else {
            printf( "Button down\n" );
        }
        GPIO_CLR_INT_FLAG( _GET_GPIO_PORT( _KEY0_PIN ), _MASK( _GET_GPIO_PIN( _KEY0_PIN ) ) );
    }

    if ( GPIO_GET_INT_FLAG( _GET_GPIO_PORT( _BAT_CHRG_PIN ), _MASK( _GET_GPIO_PIN( _BAT_CHRG_PIN ) ) )
         | GPIO_GET_INT_FLAG( _GET_GPIO_PORT( _BAT_STDBY_PIN ), _MASK( _GET_GPIO_PIN( _BAT_STDBY_PIN ) ) ) ) {
        if ( _BAT_STDBY_PIN == 1 && _BAT_CHRG_PIN == 0 ) { printf( "The battery is charging.\n" ); }
        else if ( _BAT_STDBY_PIN == 0 && _BAT_CHRG_PIN == 1 ) {
            printf( "Battery fully charged.\n" );
        }
        else if ( _BAT_STDBY_PIN == 1 && _BAT_CHRG_PIN == 1 ) {
            uint32_t i;
            DelayUs( 500000 );
            if ( _BAT_STDBY_PIN == 1 && _BAT_CHRG_PIN == 1 )
                ;
            printf( "Power is unplugged.\n" );
            printf( "Power off after %d seconds", 1 );
            for ( i = 0; i < 10; i++ ) {
                DelayUs( 100000 );
                printf( "." );
            }
            printf( "\nPower down\n" );
            CLK_SysTickDelay( 10000 );
            _POWER_PIN = 0;
        }
        GPIO_CLR_INT_FLAG( _GET_GPIO_PORT( _BAT_CHRG_PIN ), _MASK( _GET_GPIO_PIN( _BAT_CHRG_PIN ) ) );
        GPIO_CLR_INT_FLAG( _GET_GPIO_PORT( _BAT_STDBY_PIN ), _MASK( _GET_GPIO_PIN( _BAT_STDBY_PIN ) ) );
    }
}
void UART0_IRQHandler( void ) {
    uint32_t intSt = UART0->INTSTS;
    if ( intSt & UART_INTSTS_ABRIF_Msk ) {
        uint32_t trsrSt = UART0->TRSR;
        if ( trsrSt & UART_TRSR_ABRDTOIF_Msk ) { printf( "Catch buad rate error!!\n" ); }
        else if ( trsrSt & UART_TRSR_ABRDIF_Msk ) {
            printf( "--------------------------------\n" );
            printf( "Catch UART0 baud rate done.\n" );
            printf( "Buad rate : %d\n", _UART_GetUartClk( UART0 ) / ( UART0->BAUD + 1 ) );
        }
        UART_ClearIntFlag( UART0, UART_INTSTS_ABRIF_Msk );
    }
}
void CKSD_IRQHandler( void ) {
    uint32_t clkDieFlag = CLK->CLKDSTS;

    if ( clkDieFlag & ( 0x1U << 2 ) ) {
        printf( "HXT Error!!\n" );
        CLK->CLKDSTS = ( 0x1 << 2 );
    }
    if ( clkDieFlag & ( 0x1U << 1 ) ) {
        printf( "LXT Stop!!\n" );
        CLK->CLKDSTS = ( 0x1 << 1 );
    }
    if ( clkDieFlag & ( 0x1U << 0 ) ) {
        printf( "HXT Stop!!\n" );
        CLK->CLKDSTS = ( 0x1 << 0 );
    }
}
void HIRC_IRQHandler( void ) {
    uint32_t hirc0Flag = SYS_GET_IRC0TRIM_INT_FLAG();
    uint32_t hirc1Flag = SYS_GET_IRC1TRIM_INT_FLAG();
    uint32_t mircFlag  = SYS_GET_IRC1TRIM_INT_FLAG();

    if ( ( hirc0Flag & SYS_IRCTISTS_32KERR_INT ) | ( hirc1Flag & SYS_IRCTISTS_32KERR_INT ) | ( mircFlag & SYS_IRCTISTS_32KERR_INT ) ) {
        printf( "LXT Clock error.\n" );
        SYS_CLEAR_IRC0TRIM_INT_FLAG( hirc0Flag );
        SYS_CLEAR_IRC1TRIM_INT_FLAG( hirc1Flag );
        SYS_CLEAR_MIRCTRIM_INT_FLAG( mircFlag );
    }

    if ( hirc0Flag & SYS_IRCTISTS_FAIL_INT ) {
        printf( "HIRC0 : Trim Failure.\n" );
        SYS_CLEAR_IRC0TRIM_INT_FLAG( hirc0Flag );
    }

    if ( hirc1Flag & SYS_IRCTISTS_FAIL_INT ) {
        printf( "HIRC1 : Trim Failure.\n" );
        SYS_CLEAR_IRC1TRIM_INT_FLAG( hirc1Flag );
    }

    if ( mircFlag & SYS_IRCTISTS_FAIL_INT ) {
        printf( "MIRC : Trim Failure.\n" );
        SYS_CLEAR_MIRCTRIM_INT_FLAG( mircFlag );
    }
}
void SysTick_Handler( void ) {
    static uint32_t powerDoneCounter = 0;
    if(_POWER_PIN == 1){
        if(_KEY0_PIN == 0){
            if(powerDoneCounter++ > 15){
                _POWER_PIN = 0;
                _LED_B_PWM_PIN = 1;
            }
        }
        else{
            powerDoneCounter = 0;
        }
    }
}
void ADC_IRQHandler( void ) {
    uint32_t status = ADC->STATUS;
    if ( status & ADC_STATUS_ADIF_Msk ) {
        adcTemp += 1;
        adc->Event_AdcConverted( adc );
        pre->Update( pre );
        ADC_CLR_INT_FLAG( ADC, ADC_ADF_INT );
    }
    if ( status & ADC_STATUS_ADCMPF0_Msk ) { ADC_CLR_INT_FLAG( ADC, ADC_CMP0_INT ); }
    if ( status & ADC_STATUS_ADCMPF1_Msk ) { ADC_CLR_INT_FLAG( ADC, ADC_CMP1_INT ); }
}
