#ifndef _DRIVER_PRESSURE_
#define _DRIVER_PRESSURE_

#define DRIVER_ADC_API_VERSION DRIVER_VERSION_MAIJOR_MINOR( 1, 00 )

#include "Driver_MCU.h"

#define DRV_ADC_CHANNEL_NUM 18

typedef struct _DRV_ADC_PARAMS {
    void *   adc;
    uint32_t resolution;
    uint32_t refSource;
    uint32_t dataMode;
    uint32_t catchMode;
} DRV_ADC_PARAMS;

typedef struct _DRV_ADC_CHANNEL {
    uint8_t  enabled : 1;
    uint8_t  extCatch : 4;
    uint16_t value;
} DRV_ADC_CHANNEL;

typedef struct _DRV_ADC {
    struct {
        uint32_t *adc;
    } Params;

    DRV_ADC_CHANNEL channel[ DRV_ADC_CHANNEL_NUM ];

    void ( *Initialize )( struct _DRV_ADC *self, DRV_ADC_PARAMS initParams );
    void ( *ChannelEnable )( struct _DRV_ADC *self, uint16_t channel, uint8_t extCatch );
    void ( *ChannelDisable )( struct _DRV_ADC *self, uint16_t channel );
    void ( *Conversion )( struct _DRV_ADC *self );

    void ( *Event_AdcConverted )( struct _DRV_ADC *self );

    void ( *DRV_AdcFree )( struct _DRV_ADC *self );

} DRV_ADC;

DRV_ADC *DRV_AdcNew( void );

#endif
