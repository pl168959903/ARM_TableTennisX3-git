#include "TableTennisX3.h"

int main( void ) {
    MCU_Init();
    MPU9250_Init();
    printf( "START!!\n" );


    while ( 1 ) {
        adcTemp = 0;
        ADC_START_CONV(ADC);
        DelayUs( 1000000 );
        ADC_STOP_CONV(ADC);
        uint8_t i;
        for( i = 0; i <8;i++)printf( "ch%d:%d ",i, adc->channel[i].value );
        printf( " adcTemp：%d\n", adcTemp);
    };
}
