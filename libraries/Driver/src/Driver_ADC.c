
#define DRIVER_ADC_DRV_VERSION DRIVER_VERSION_MAIJOR_MINOR( 1, 00 )

#include "Driver_ADC.h"
#include "TableTennisX3.h"

#define _ADC_BASE                      ( ( ADC_T * )( self->Params.adc ) )
#define _FUNCTION_POINTER_LINK( func ) self->func = func

static void ChannelEnable( DRV_ADC *self, uint16_t channel, uint8_t extCatch ) {
    ADC_SetExtraSampleTime( _ADC_BASE, channel, extCatch );
    self->channel[ channel ].enabled = true;
    _ADC_BASE->CHEN |= ( 0x1 << channel );
}
static void ChannelDisable( DRV_ADC *self, uint16_t channel ) {
    _ADC_BASE->CHEN &= ~( 0x1 << channel );
    self->channel[ channel ].enabled = false;
}

static void Initialize( DRV_ADC *self, DRV_ADC_PARAMS initParams ) {
    self->Params.adc = initParams.adc;
    ADC_SET_RESOLUTION( _ADC_BASE, initParams.resolution );
    ADC_SET_REF_VOLTAGE( _ADC_BASE, initParams.refSource );
    ADC_EnableInt( _ADC_BASE, ADC_ADF_INT );
    ADC_POWER_ON( _ADC_BASE );
    ADC_Open( _ADC_BASE, initParams.dataMode, initParams.catchMode, 0 );
}

void Conversion( DRV_ADC *self ) {
    ADC_START_CONV( _ADC_BASE );
}

static void Event_AdcConverted( DRV_ADC *self ) {
    uint8_t    i ;
    for ( i = 0; i < 18; i++ ) {
        if ( self->channel[ i ].enabled == true ) { 
            //self->channel[ i ].value = ADC_GET_CONVERSION_DATA( _ADC_BASE, i ); 
        }
        
        // printf("%D", i); 
    }

    // self->channel[ 0 ].value = ADC_GET_CONVERSION_DATA( _ADC_BASE, 0 );
    // self->channel[1].value = ADC_GET_CONVERSION_DATA( _ADC_BASE, 1 );
    // self->channel[2].value = ADC_GET_CONVERSION_DATA( _ADC_BASE, 2 );
    // self->channel[3].value = ADC_GET_CONVERSION_DATA( _ADC_BASE, 3 );
    // self->channel[4].value = ADC_GET_CONVERSION_DATA( _ADC_BASE, 4 );
    // self->channel[5].value = ADC_GET_CONVERSION_DATA( _ADC_BASE, 5 );
    // self->channel[6].value = ADC_GET_CONVERSION_DATA( _ADC_BASE, 6 );
    // self->channel[7].value = ADC_GET_CONVERSION_DATA( _ADC_BASE, 7 );
    //MCU_SysTickDelayUs(7);
}

DRV_ADC *DRV_AdcNew( void ) {
    uint16_t i;
    DRV_ADC *self = _DRV_MACCLOC( sizeof( DRV_ADC ) );
    for ( i = 0; i < DRV_ADC_CHANNEL_NUM; i++ ) self->channel[ i ].enabled = false;

    _FUNCTION_POINTER_LINK( Initialize );
    _FUNCTION_POINTER_LINK( ChannelEnable );
    _FUNCTION_POINTER_LINK( ChannelDisable );
    _FUNCTION_POINTER_LINK( Conversion );

    _FUNCTION_POINTER_LINK( Event_AdcConverted );

    return self;
}

static void DRV_AdcFree( DRV_ADC *self ) {
    _DRV_FREE( self );
}
