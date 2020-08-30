#include "TableTennisX3.h"

volatile uint8_t mpuFlag = 0;

void MPU_init( void );

void test9aix( void );

int main( void ) {
    MCU_Init();
    MPU9250_Init();
    // vMemInfoPrint();
    printf( "START!!\n" );
    GPIO_EnableInt( _GET_GPIO_PORT( _ICM_INT1_PIN ), _GET_GPIO_PIN( _ICM_INT1_PIN ), GPIO_INT_FALLING );
    MPU_init();

    while ( 1 ) {
        // while(1){};
        uint8_t temp = 0xFF;
        // mpu9250->ReadRegister( mpu9250, 0x3F, &temp, 1 );
        // printf("x : 0x%02x\n", temp);
        DelayUs( 100000 );
    };
}

void test9aix( void ) {
    uint8_t w[ 2 ] = { 0x3F | 0x80, 0xFF };
    uint8_t a[ 2 ] = { 0 };

    spi0->ReadAndWrite( spi0, w, w, 2 );
}

void readTest( uint8_t addr, uint8_t* data, uint8_t len ) {
    uint8_t rv[ 24 ];
    DRV_MPU9250_WRITE_BYTE( mpu9250, _DRV_MPU9250_I2C_SLV0_ADDR, 0x0C | 0x80 );
    DRV_MPU9250_WRITE_BYTE( mpu9250, _DRV_MPU9250_I2C_SLV0_REG, addr );
    DRV_MPU9250_WRITE_BYTE( mpu9250, _DRV_MPU9250_I2C_SLV0_CTRL, _DRV_MPU9250_I2C_SLV0_CTRL_I2C_SLV0_EN_Msk | len );

    mpuFlag = 0;
    while ( mpuFlag == 0 ) {};

    mpu9250->ReadRegister( mpu9250, _DRV_MPU9250_EXT_SENS_DATA_00, data, len );
}

void writeTest( uint8_t addr, uint8_t data ) {
    DRV_MPU9250_WRITE_BYTE( mpu9250, _DRV_MPU9250_I2C_SLV0_ADDR, 0x0C );
    DRV_MPU9250_WRITE_BYTE( mpu9250, _DRV_MPU9250_I2C_SLV0_REG, addr );
    DRV_MPU9250_WRITE_BYTE( mpu9250, _DRV_MPU9250_I2C_SLV0_DO, data );
    DRV_MPU9250_WRITE_BYTE( mpu9250, _DRV_MPU9250_I2C_SLV0_CTRL, _DRV_MPU9250_I2C_SLV0_CTRL_I2C_SLV0_EN_Msk | 0x01 );

    mpuFlag = 0;
    while ( mpuFlag == 0 ) {};
}

void switchTest( uint8_t t ) {
    switch ( t ) {
        do {
        case 0:
            printf( "t : %d\n", t );
        case 1:
            printf( "t : %d\n", t );
        case 2:
            printf( "t : %d\n", t );
        case 3:
            printf( "t : %d\n", t );
        case 4:
            printf( "t : %d\n", t );
        case 5:
            printf( "t : %d\n", t );
        } while ( t < 0 );
    }
}

void MPU_init( void ) {
    uint8_t data;
    uint8_t i;

    DRV_MPU9250_RESET( mpu9250 );
    DelayUs( 100000 );
    data = DRV_MPU9250_READ_BYTE( mpu9250, _DRV_MPU9250_WHO_AM_I );
    printf( "Check MPU9250 connect : %s\n", ( ( data == 0x71 ) ? "true" : "false" ) );

    mpu9250->Initialize( mpu9250, (DRV_MPU9250_PARAMS){24, 4, 4, 3, 3});

    /*
    while ( 1 ) {
        printf( "Accel X:%4d Y:%4d Z:%4d  ", mpu9250->Params.RawData.Accel.X, mpu9250->Params.RawData.Accel.Y, mpu9250->Params.RawData.Accel.Z );
        printf( "Gyro X:%4d Y:%4d Z:%4d  ", mpu9250->Params.RawData.Gyro.X, mpu9250->Params.RawData.Gyro.Y, mpu9250->Params.RawData.Gyro.Z );
        printf( "Mag X:%4d Y:%4d Z:%4d  ", mpu9250->Params.RawData.Mag.X, mpu9250->Params.RawData.Mag.Y, mpu9250->Params.RawData.Mag.Z );
        printf( "Temp %4d\n", mpu9250->Params.RawData.Temp );
        DelayUs( 100000 );
    }
    */

    // PRINT_REG();
}

//
