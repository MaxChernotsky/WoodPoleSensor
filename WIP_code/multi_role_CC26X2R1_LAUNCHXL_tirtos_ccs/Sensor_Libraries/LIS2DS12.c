//contains all functions needed to use the MMC5603NJ NJgnetometer



//includes
#include "LIS2DS12.h"
#include <ti/drivers/I2C.h>
#include <stdio.h>

//to use Log_info debug tools
#include <ti/common/cc26xx/uartlog/UartLog.h>  // Comment out if using xdc Log

//ability to use sleep functions
#include <ti/sysbios/knl/Task.h>

//I2C initialisations
#define SENSORS 0



uint8_t LIS2DS12_readByte(uint8_t regToRead){

    //define local variables to be used to handle the tx and rx arrays
    uint8_t txBuffer[1];
    uint8_t rxBuffer[1];

    //set the tx array to registers to read/write
    txBuffer[0] = regToRead;

    //set the parameters to be used to communicate with i2c
    I2C_Params params;
    I2C_Params_init(&params);
    params.bitRate = I2C_400kHz;

    // Open I2C bus for usage
    I2C_Handle i2cHandle = I2C_open(SENSORS, &params);

    // Initialize slave address of transaction
    I2C_Transaction transaction = {0};
    transaction.slaveAddress = LIS2DS12_ADDRESS;

    transaction.writeBuf = txBuffer;
    transaction.writeCount = 1;//0 indicates data will be read from the register
    transaction.readBuf = rxBuffer;
    transaction.readCount = 1;

    bool i2cTransferStatus = I2C_transfer(i2cHandle, &transaction);

    if (i2cTransferStatus){
        Log_info0("i2c transfer success");
    }

    //close i2c instance
    I2C_close(i2cHandle);

    //return result

    printf("readByte Output %d\n", rxBuffer[0]);
    Log_info1("Read: %d", rxBuffer[0]);
    return rxBuffer[0];

}//end readByte

//function to be used to writeByte to the selected device
void LIS2DS12_writeByte(uint8_t regToWrite, uint8_t byteToWrite){

    //define local variables to be used to handle the tx and rx arrays
    uint8_t txBuffer[2];

    //set the tx array to registers to read/write
    txBuffer[0] = regToWrite;
    txBuffer[1] = byteToWrite;

    //set the parameters to be used to communicate with i2c
    I2C_Params params;
    I2C_Params_init(&params);
    params.bitRate = I2C_400kHz;

    // Open I2C bus for usage
    I2C_Handle i2cHandle = I2C_open(SENSORS, &params);

    // Initialize slave address of transaction
    I2C_Transaction transaction = {0};
    transaction.slaveAddress = LIS2DS12_ADDRESS;

    transaction.writeBuf = txBuffer;
    transaction.writeCount = 2;//0 indicates data will be read from the register
    transaction.readBuf = NULL;
    transaction.readCount = 0;

    bool i2cTransferStatus = I2C_transfer(i2cHandle, &transaction);

    if (i2cTransferStatus){
        printf("i2c transfer success \n");
    }

    //close i2c instance
    I2C_close(i2cHandle);
}//end readByte

void LIS2DS12_getAccData (uint16_t * destination) {

    //define variables
    uint8_t txBuffer[1];
    uint8_t rxBuffer[6];

    //set the tx array to registers to read/write
    txBuffer[0] = LIS2DS12_OUT_X_L;

    //set the parameters to be used to communicate with i2c
    I2C_Params params;
    I2C_Params_init(&params);
    params.bitRate = I2C_400kHz;

    // Open I2C bus for usage
    I2C_Handle i2cHandle = I2C_open(SENSORS, &params);

    // Initialize slave address of transaction
    I2C_Transaction transaction = {0};
    transaction.slaveAddress = LIS2DS12_ADDRESS;

    transaction.writeBuf = txBuffer;
    transaction.writeCount = 1;//0 indicates data will be read from the register
    transaction.readBuf = rxBuffer;
    transaction.readCount = 6;

    bool i2cTransferStatus = I2C_transfer(i2cHandle, &transaction);

    if (i2cTransferStatus){
        Log_info0("i2c transfer success");
    }//end if

    //close i2c instance
    I2C_close(i2cHandle);

    Log_info2("XL: %d, XH: %d", rxBuffer[0], rxBuffer[1]);
    Log_info2("YL: %d, YH: %d", rxBuffer[2], rxBuffer[3]);
    Log_info2("ZL: %d, ZH: %d", rxBuffer[4], rxBuffer[5]);

    destination[0] = (uint16_t)(rxBuffer[1] << 8 | rxBuffer[0]);
    destination[1] = (uint16_t)(rxBuffer[3] << 8 | rxBuffer[2]);
    destination[2] = (uint16_t)(rxBuffer[5] << 8 | rxBuffer[4]);

    Log_info3("X: %d, Y: %d, Z: %d", destination[0], destination[1], destination[2]);
}//end getMagData


//function to get the product ID of the device
uint8_t LIS2DS12_getWhoAmI(){
    return LIS2DS12_readByte(LIS2DS12_WHO_AM_I);
}//end getProductID

//function to get the status of the device
uint8_t LIS2DS12_status(){
    return LIS2DS12_readByte(LIS2DS12_STATUS);
}//end getProductID

//function to get the status of the device
uint8_t LIS2DS12_getTemperature(){
    return LIS2DS12_readByte(LIS2DS12_OUT_T);
}//end getProductID

