#include "TableTennisX3.h"

volatile uint8_t mpuFlag = 0;

void MPU_init( void );

void test9aix( void );

int main( void ) {
    MCU_Init();
    MPU9250_Init();
    // vMemInfoPrint();
    printf( "START!!\n" );
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
#define PRINT_REG()                                           \
    for ( i = 0; i < 127; i++ ) {                             \
        data = DRV_MPU9250_READ_BYTE( mpu9250, i );           \
        printf( "addr : 0x%02X(%d) : 0x%02X\n", i, i, data ); \
    }

    DRV_MPU9250_RESET( mpu9250 );
    DelayUs( 100000 );
    data = DRV_MPU9250_READ_BYTE( mpu9250, _DRV_MPU9250_WHO_AM_I );
    printf( "Check MPU9250 connect : %s\n", ( ( data == 0x71 ) ? "true" : "false" ) );

    DRV_MPU9250_USE_PLL( mpu9250 );
    DRV_MPU9250_SAMPLE_DIV( mpu9250, 10 );

    DRV_MPU9250_SET_GYRO_DLPF_CFG( mpu9250, 2 );
    DRV_MPU9250_GYRO_DLPF_ENABLE( mpu9250 );
    DRV_MPU9250_SET_GYRO_FS( mpu9250, DRV_MPU9250_GYRO_FS( 2 ) );

    DRV_MPU9250_ACCEL_SET_CFG( mpu9250, 2 );
    DRV_MPU9250_ACCEL_DLPF_ENABLE( mpu9250 );
    DRV_MPU9250_SET_ACCEL_FS( mpu9250, DRV_MPU9250_ACCEL_FS( 1 ) );

    mpu9250->SetRegisters( mpu9250, _DRV_MPU9250_I2C_SLV4_CTRL, _DRV_MPU9250_I2C_SLV4_CTRL_SLV4_DONE_INT_EN_Msk );

    DRV_MPU9250_WRITE_BYTE( mpu9250, _DRV_MPU9250_INT_PIN_CFG, _DRV_MPU9250_INT_PIN_CFG_ACTL_Msk | _DRV_MPU9250_INT_PIN_CFG_OPEN_Msk );
    DRV_MPU9250_INT_ENABLE( mpu9250, _DRV_MPU9250_INT_ENABLE_RAW_RDY_EN_Msk );
    DRV_MPU9250_INT_ENABLE( mpu9250, 0x8 );
    GPIO_EnableInt( _GET_GPIO_PORT( _ICM_INT1_PIN ), _GET_GPIO_PIN( _ICM_INT1_PIN ), GPIO_INT_FALLING );

    DRV_MPU9250_WRITE_BYTE( mpu9250, _DRV_MPU9250_I2C_MST_CTRL, _DRV_MPU9250_I2C_MST_CTRL_WAIT_FOR_ES_Msk | _DRV_MPU9250_I2C_MST_CLK_400K );
    DRV_MPU9250_WRITE_BYTE( mpu9250, _DRV_MPU9250_USER_CTRL, _DRV_MPU9250_USER_CTRL_I2C_MST_EN_Msk );

    mpu9250->I2cMstWriteByte( mpu9250, _DRV_MPU9250_I2C_SLV0, 0x0C, 0x0A, 0x16 );
    mpu9250->I2cMstRead( mpu9250, _DRV_MPU9250_I2C_SLV0, 0x0C, 0x03, NULL, 7 );
    
    printf("size : %d\n", sizeof(DRV_MPU9250));

    while ( 1 ) {
        mpu9250->Updata( mpu9250 );
        printf( "Accel X:%d Y:%d Z:%d  ", mpu9250->Params.RawData.Accel.X, mpu9250->Params.RawData.Accel.Y, mpu9250->Params.RawData.Accel.Z );
        printf( "Gyro X:%d Y:%d Z:%d  ", mpu9250->Params.RawData.Gyro.X, mpu9250->Params.RawData.Gyro.Y, mpu9250->Params.RawData.Gyro.Z );
        printf( "Mag X:%d Y:%d Z:%d  ", mpu9250->Params.RawData.Mag.X, mpu9250->Params.RawData.Mag.Y, mpu9250->Params.RawData.Mag.Z );
        printf( "Temp %d\n", mpu9250->Params.RawData.Temp );
        DelayUs( 100000 );
    }

    // PRINT_REG();
}

//
