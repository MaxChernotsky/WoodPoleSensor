//contains all functions needed to use the MMC5983MA magnetometer



//includes
#include "MMC5983MA.h"
#include <ti/drivers/I2C.h>
#include <stdio.h>

//I2C initialisations
#define SENSORS 0



uint8_t MMC5983_readByte(uint8_t regToRead){

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
    transaction.slaveAddress = MMA5983MA_ADDRESS;

    transaction.writeBuf = txBuffer;
    transaction.writeCount = 1;//0 indicates data will be read from the register
    transaction.readBuf = rxBuffer;
    transaction.readCount = 1;

    bool i2cTransferStatus = I2C_transfer(i2cHandle, &transaction);

    //close i2c instance
    I2C_close(i2cHandle);

    //return result
    return rxBuffer[0];

}//end readByte




//function to get the product ID of the device

uint8_t MMC5983_getProductID(){

    //define local variables to be used to handle the tx and rx arrays
    uint8_t txBuffer[1];
    uint8_t rxBuffer[1];

    //set the tx array to registers to read/write
    txBuffer[0] = MMA5983MA_PRODUCT_ID;

    //set the parameters to be used to communicate with i2c
    I2C_Params params;
    I2C_Params_init(&params);
    params.bitRate = I2C_400kHz;

    // Open I2C bus for usage
    I2C_Handle i2cHandle = I2C_open(SENSORS, &params);

    // Initialize slave address of transaction
    I2C_Transaction transaction = {0};
    transaction.slaveAddress = MMA5983MA_ADDRESS;

    transaction.writeBuf = txBuffer;
    transaction.writeCount = 1;//0 indicates data will be read from the register
    transaction.readBuf = rxBuffer;
    transaction.readCount = 1;

    bool i2cTransferStatus = I2C_transfer(i2cHandle, &transaction);

    //close i2c instance
    I2C_close(i2cHandle);

    //return result
    return rxBuffer[0];

}//end getProductID

//internal function to be used to initiate a temperature measurement on the device
static void MMC5983_updateTemperature(){

    //define local variables to be used to handle the tx and rx arrays
    uint8_t txBuffer[2];

    //set the tx array to registers to read/write
    txBuffer[0] = MMA5983MA_CONTROL_0;
    txBuffer[1] = MMA5983MA_CONTROL_0_TM_T;

    //set the parameters to be used to communicate with i2c
    I2C_Params params;
    I2C_Params_init(&params);
    params.bitRate = I2C_400kHz;

    // Open I2C bus for usage
    I2C_Handle i2cHandle = I2C_open(SENSORS, &params);

    // Initialize slave address of transaction
    I2C_Transaction transaction = {0};
    transaction.slaveAddress = MMA5983MA_ADDRESS;

    transaction.writeBuf = txBuffer;
    transaction.writeCount = 2;//0 indicates data will be read from the register
    transaction.readBuf = NULL;
    transaction.readCount = 0;

    bool i2cTransferStatus = I2C_transfer(i2cHandle, &transaction);

    //close i2c instance
    I2C_close(i2cHandle);

}//end updateTemperature


//function to get the raw temperature value in the register of the sensor
uint8_t MMC5983_getTemperature (int loops){

    //call function to initiate a new temp measurement
    MMC5983_updateTemperature();

    //define local variables to be used to handle the tx and rx arrays
    uint8_t txBuffer[1];
    uint8_t rxBuffer[1];

    //set the tx array to registers to read/write
    txBuffer[0] = MMA5983MA_TOUT;

    //set the parameters to be used to communicate with i2c
    I2C_Params params;
    I2C_Params_init(&params);
    params.bitRate = I2C_400kHz;

    // Open I2C bus for usage
    I2C_Handle i2cHandle = I2C_open(SENSORS, &params);

    // Initialize slave address of transaction
    I2C_Transaction transaction = {0};
    transaction.slaveAddress = MMA5983MA_ADDRESS;

    transaction.writeBuf = txBuffer;
    transaction.writeCount = 1;//0 indicates data will be read from the register
    transaction.readBuf = rxBuffer;
    transaction.readCount = 1;

    bool i2cTransferStatus = I2C_transfer(i2cHandle, &transaction);

    //close i2c instance
    I2C_close(i2cHandle);

    //return result
    return rxBuffer[0];
}//end getTemperature

//function to get the current device status and return the value shown
uint8_t MMC5983_getStatus(){
    //define local variables to be used to handle the tx and rx arrays
    uint8_t txBuffer[1];
    uint8_t rxBuffer[1];

    //set the tx array to registers to read/write
    txBuffer[0] = MMA5983MA_STATUS;

    //set the parameters to be used to communicate with i2c
    I2C_Params params;
    I2C_Params_init(&params);
    params.bitRate = I2C_400kHz;

    // Open I2C bus for usage
    I2C_Handle i2cHandle = I2C_open(SENSORS, &params);

    // Initialize slave address of transaction
    I2C_Transaction transaction = {0};
    transaction.slaveAddress = MMA5983MA_ADDRESS;

    transaction.writeBuf = txBuffer;
    transaction.writeCount = 1;//0 indicates data will be read from the register
    transaction.readBuf = rxBuffer;
    transaction.readCount = 1;

    bool i2cTransferStatus = I2C_transfer(i2cHandle, &transaction);

    //close i2c instance
    I2C_close(i2cHandle);

    //return result
    return rxBuffer[0];
}//end getStatus

void MMC5983_initMag(){

    //define local variables to be used to handle the tx and rx arrays
    uint8_t txBuffer[6];

    //set the tx array to registers to read/write
    txBuffer[0] = MMA5983MA_CONTROL_0;
    txBuffer[1] = MMA5983MA_CONTROL_0_AUTO_SET_RESET; //enable auto set/reset & enable magnetometer sensor readings
    txBuffer[2] = MMA5983MA_CONTROL_1;
    txBuffer[3] = BW_400HZ; //set the bandwidth

    //set the device to run in continuous mode
    txBuffer[4] = MMA5983MA_CONTROL_2;
    txBuffer[5] = MMA5983MA_CONTROL_2_CONT_MODE | MODR_100Hz;



    //set the parameters to be used to communicate with i2c
    I2C_Params params;
    I2C_Params_init(&params);
    params.bitRate = I2C_400kHz;

    // Open I2C bus for usage
    I2C_Handle i2cHandle = I2C_open(SENSORS, &params);

    // Initialize slave address of transaction
    I2C_Transaction transaction = {0};
    transaction.slaveAddress = MMA5983MA_ADDRESS;

    transaction.writeBuf = txBuffer;
    transaction.writeCount = 6;//0 indicates data will be read from the register
    transaction.readBuf = NULL;
    transaction.readCount = 0;

    bool i2cTransferStatus = I2C_transfer(i2cHandle, &transaction);

    //close i2c instance
    I2C_close(i2cHandle);
    printf("Init: DONE\n");



}//end init


uint8_t MMC5983_getXValue() {
    //define local variables to be used to handle the tx and rx arrays
    uint8_t txBuffer[3];
    uint8_t rxBuffer[3];

    //set the tx array to registers to read/write
    txBuffer[0] = MMA5983MA_XOUT_0;
    txBuffer[1] = MMA5983MA_XOUT_1;
    txBuffer[2] = MMA5983MA_XYZOUT_2;

    //set the parameters to be used to communicate with i2c
    I2C_Params params;
    I2C_Params_init(&params);
    params.bitRate = I2C_400kHz;

    // Open I2C bus for usage
    I2C_Handle i2cHandle = I2C_open(SENSORS, &params);

    // Initialize slave address of transaction
    I2C_Transaction transaction = {0};
    transaction.slaveAddress = MMA5983MA_ADDRESS;

    transaction.writeBuf = txBuffer;
    transaction.writeCount = 2;//0 indicates data will be read from the register
    transaction.readBuf = rxBuffer;
    transaction.readCount = 2;

    bool i2cTransferStatus = I2C_transfer(i2cHandle, &transaction);

    //close i2c instance
    I2C_close(i2cHandle);

    printf("xout0: %d, xout1: %d, xout2: %d\n", rxBuffer[0], rxBuffer[1], rxBuffer[2]);

    return rxBuffer[0];

}//end getXValue

uint16_t * MMC5983_getMagData () {

    //define variables
    uint8_t xVals[2];
    uint8_t yVals[2];
    uint8_t zVals[2];
    uint8_t additionalVals[1];

    uint16_t combinedRead[3];


    //call uint8_t MMC5983_readByte() function with a number of times to access all the data required

    //read x data
    xVals[0] = MMC5983_readByte(MMA5983MA_XOUT_0);
    xVals[1] = MMC5983_readByte(MMA5983MA_XOUT_1);

    //read y data
    yVals[0] = MMC5983_readByte(MMA5983MA_YOUT_0);
    yVals[1] = MMC5983_readByte(MMA5983MA_YOUT_1);

    //read z data
    zVals[0] = MMC5983_readByte(MMA5983MA_ZOUT_0);
    zVals[1] = MMC5983_readByte(MMA5983MA_ZOUT_1);

    //read comb data
    additionalVals[0] = MMC5983_readByte(MMA5983MA_XYZOUT_2);


    printf("x0: %d, x1: %d, y0: %d, y1: %d, z0: %d, z1: %d,  add0: %d\n", xVals[0], xVals[1], yVals[0], yVals[1], zVals[0], zVals[1], additionalVals[0]);


    combinedRead[0] = (uint16_t)(xVals[0] << 8 | xVals[1]);
    combinedRead[1] = (uint16_t)(yVals[0] << 8 | yVals[1]);
    combinedRead[2] = (uint16_t)(zVals[0] << 8 | zVals[1]);


    return combinedRead;
    //combine the data into a singular format


}//end getMagData




