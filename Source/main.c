#include "TableTennisX3.h"
uint8_t g_rfAddress[ 5 ] = { 0x00, 0x00, 0x01, 0x00, 0x00 };

uint8_t testData[ 5 ] = { 0x0A, 0x0B, 0x0C, 0x0D, 0x0E };

uint8_t send_data0[ 26 ] = { 0 };
uint8_t send_data1[ 26 ] = { 0 };
uint8_t send_data2[ 26 ] = { 0 };

float mpuData[ 6 ] = { 0 };
float fsrData[ 8 ] = { 0 };

int main( void ) {
    uint8_t i;
    MCU_Init();
    MPU9250_Init();
    i = 0;
    while(_KEY0_PIN == 0){
        DelayUs(100000);
        if(i++ > 30) break;
    }

    _POWER_PIN = 1;
    _LED_B_PWM_PIN = 0;

    printf( "START!!\n" );

    NRF_Init();

    send_data0[ 0 ]  = 0;
    send_data1[ 0 ]  = 1;
    send_data2[ 0 ]  = 2;
    send_data2[ 9 ]  = 0x0a;
    send_data2[ 10 ] = 0x0d;

    ADC_START_CONV( ADC );

    while ( 1 ) {
        if ( mpu9250->Params.RawDataReady_Flag == 1 ) {
            mpu9250->Params.RawDataReady_Flag = 0;

            mpuData[ 0 ] = ( float )( mpu9250->Params.RawData.Accel.X );
            mpuData[ 1 ] = ( float )( mpu9250->Params.RawData.Accel.Y );
            mpuData[ 2 ] = ( float )( mpu9250->Params.RawData.Accel.Z );
            mpuData[ 3 ] = ( float )( mpu9250->Params.RawData.Gyro.X );
            mpuData[ 4 ] = ( float )( mpu9250->Params.RawData.Gyro.Y );
            mpuData[ 5 ] = ( float )( mpu9250->Params.RawData.Gyro.Z );

            for ( i = 0; i < 8; i++ ) {
                if ( pre->channels[ i ].isCapturing == 0 && pre->channels[ i ].Value.extremum != 0 ) {
                    fsrData[ i ]                      = ( float )( 4096 - ( pre->channels[ i ].Value.extremum ) );
                    printf( "c:%d v:%d\n",i, fsrData[ i ]);
                    pre->channels[ i ].Value.extremum = 0;
                }
                else {
                    fsrData[ i ] = 0;
                }
            }

            for ( i = 0; i < 24; i++ ) send_data0[ i + 1 ] = *( ( ( uint8_t* )mpuData ) + i );
            for ( i = 0; i < 24; i++ ) send_data1[ i + 1 ] = *( ( ( uint8_t* )fsrData ) + i );
            for ( i = 0; i < 8; i++ ) send_data2[ i + 1 ] = *( ( ( uint8_t* )fsrData ) + i + 24 );

            if ( !( nrf->SendPacket( nrf, ( DRV_NRF_TX_PARAMS ){ g_rfAddress, 1, 1, send_data0, 26 } ) ) ) {
                printf( "1 : SEND FAIL\n" );
                DRV_NRF_FLUSH_TX( nrf );
            }

            if ( !( nrf->SendPacket( nrf, ( DRV_NRF_TX_PARAMS ){ g_rfAddress, 1, 1, send_data1, 26 } ) ) ) {
                printf( "2 : SEND FAIL\n" );
                DRV_NRF_FLUSH_TX( nrf );
            }

            if ( !( nrf->SendPacket( nrf, ( DRV_NRF_TX_PARAMS ){ g_rfAddress, 1, 1, send_data2, 26 } ) ) ) {
                printf( "3 : SEND FAIL\n" );
                DRV_NRF_FLUSH_TX( nrf );
            }
        }
    };
}
