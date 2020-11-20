//contains all functions needed to use the MMC5603NJ NJgnetometer



//includes
#include "MMC5603NJ.h"
#include <ti/drivers/I2C.h>
#include <stdio.h>

//to use Log_info debug tools
#include <ti/common/cc26xx/uartlog/UartLog.h>  // Comment out if using xdc Log

//ability to use sleep functions
#include <ti/sysbios/knl/Task.h>

//I2C initialisations
#define SENSORS 0



uint8_t MMC5603_readByte(uint8_t regToRead){

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
    transaction.slaveAddress = MMC5603NJ_ADDRESS;

    transaction.writeBuf = txBuffer;
    transaction.writeCount = 1;//0 indicates data will be read from the register
    transaction.readBuf = rxBuffer;
    transaction.readCount = 1;

    bool i2cTransferStatus = I2C_transfer(i2cHandle, &transaction);

    if (i2cTransferStatus){
        printf("i2c transfer success \n");
    }

    //close i2c instance
    I2C_close(i2cHandle);

    //return result

    printf("readByte Output %d\n", rxBuffer[0]);
    return rxBuffer[0];

}//end readByte

//function to be used to writeByte to the selected device
void MMC5603_writeByte(uint8_t regToWrite, uint8_t byteToWrite){

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
    transaction.slaveAddress = MMC5603NJ_ADDRESS;

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

//function to get the product ID of the device
uint8_t MMC5603_getProductID(){
    return MMC5603_readByte(MMC5603NJ_PRODUCT_ID);
}//end getProductID

//internal function to be used to initiate a temperature measurement on the device
static void MMC5603_updateTemperature(){
    MMC5603_writeByte(MMC5603NJ_CONTROL_0, MMC5603NJ_CONTROL_0_TM_T);
}//end updateTemperature


//function to get the raw temperature value in the register of the sensor
uint8_t MMC5603_getTemperature (int loops){

    //call function to initiate a new temp measurement
    MMC5603_updateTemperature();

    //return result
    return MMC5603_readByte(MMC5603NJ_TOUT);
}//end getTemperature

//function to get the current device status and return the value shown
uint8_t MMC5603_getStatus(){
    return MMC5603_readByte(MMC5603NJ_STATUS);
}//end getStatus

void MMC5603_initMag(){

    //define local variables to be used to handle the tx and rx arrays
    uint8_t txBuffer[6];

    //set the tx array to registers to read/write
    txBuffer[0] = MMC5603NJ_CONTROL_0;
    txBuffer[1] = MMC5603NJ_CONTROL_0_AUTO_SR_EN | MMC5603NJ_CONTROL_0_TM_M; //enable auto set/reset & enable NJgnetometer sensor readings
    txBuffer[2] = MMC5603NJ_CONTROL_1;
    txBuffer[3] = MMC5603NJ_BW_2_0; //set the bandwidth

    //set the device to run in continuous mode
    txBuffer[4] = MMC5603NJ_CONTROL_2;
    txBuffer[5] = MMC5603NJ_CONTROL_2_CMM_FREQ_EN | MMC5603NJ_PRD_SET_100;

    //set the parameters to be used to communicate with i2c
    I2C_Params params;
    I2C_Params_init(&params);
    params.bitRate = I2C_400kHz;

    // Open I2C bus for usage
    I2C_Handle i2cHandle = I2C_open(SENSORS, &params);

    // Initialize slave address of transaction
    I2C_Transaction transaction = {0};
    transaction.slaveAddress = MMC5603NJ_ADDRESS;

    transaction.writeBuf = txBuffer;
    transaction.writeCount = 6;//0 indicates data will be read from the register
    transaction.readBuf = NULL;
    transaction.readCount = 0;

    bool i2cTransferStatus = I2C_transfer(i2cHandle, &transaction);

    //close i2c instance
    I2C_close(i2cHandle);
    printf("Init: DONE\n");
}//end init

void MMC5603_getMagData (uint16_t * destination) {
    //16 bit mode

    //define variables
    uint8_t txBuffer[1];
    uint8_t rxBuffer[9];

    //set the tx array to registers to read/write
    txBuffer[0] = MMC5603NJ_XOUT_0;

    //set the parameters to be used to communicate with i2c
    I2C_Params params;
    I2C_Params_init(&params);
    params.bitRate = I2C_400kHz;

    // Open I2C bus for usage
    I2C_Handle i2cHandle = I2C_open(SENSORS, &params);

    // Initialize slave address of transaction
    I2C_Transaction transaction = {0};
    transaction.slaveAddress = MMC5603NJ_ADDRESS;

    transaction.writeBuf = txBuffer;
    transaction.writeCount = 1;
    transaction.readBuf = rxBuffer;
    transaction.readCount = 9;

    bool i2cTransferStatus = I2C_transfer(i2cHandle, &transaction);

    if (i2cTransferStatus){
        Log_info0("i2c transfer success");
    }//end if

    //close i2c instance
    I2C_close(i2cHandle);

    Log_info2("X0: %d, X1: %d", rxBuffer[0], rxBuffer[1]);
    Log_info2("Y0: %d, Y1: %d", rxBuffer[2], rxBuffer[3]);
    Log_info2("Z0: %d, Z1: %d", rxBuffer[4], rxBuffer[5]);
    Log_info3("X2: %d, Y2: %d, Z2: %d", rxBuffer[6], rxBuffer[7], rxBuffer[8]);

    destination[0] = (uint16_t)(rxBuffer[0] << 8 | rxBuffer[1]); //x value
    destination[1] = (uint16_t)(rxBuffer[2] << 8 | rxBuffer[3]); //y value
    destination[2] = (uint16_t)(rxBuffer[4] << 8 | rxBuffer[5]); //z value

    Log_info3("X: %d, Y: %d, Z: %d", destination[0], destination[1], destination[2]);
}//end getNJgData

void MMC5603_clearInt(){

    uint8_t statusTemp = MMC5603_readByte(MMC5603NJ_STATUS);

    //write 1 to the int bit to clear the interrupt

    //define local variables to be used to handle the tx and rx arrays
    uint8_t txBuffer[1];

    //set the tx array to registers to read/write
    txBuffer[0] = statusTemp & 0x01;

    //set the parameters to be used to communicate with i2c
    I2C_Params params;
    I2C_Params_init(&params);
    params.bitRate = I2C_400kHz;

    // Open I2C bus for usage
    I2C_Handle i2cHandle = I2C_open(SENSORS, &params);

    // Initialize slave address of transaction
    I2C_Transaction transaction = {0};
    transaction.slaveAddress = MMC5603NJ_ADDRESS;

    transaction.writeBuf = txBuffer;
    transaction.writeCount = 1;//0 indicates data will be read from the register
    transaction.readBuf = NULL;
    transaction.readCount = 0;

    bool i2cTransferStatus = I2C_transfer(i2cHandle, &transaction);

    if (i2cTransferStatus){
        printf("i2c transfer success \n");
    }//end if

    //close i2c instance
    I2C_close(i2cHandle);
}//end clearInt

void MMC5603_set(){
    MMC5603_writeByte(MMC5603NJ_CONTROL_0, MMC5603NJ_CONTROL_0_SET);
}//end set

void MMC5603_reset(){
    MMC5603_writeByte(MMC5603NJ_CONTROL_0, MMC5603NJ_CONTROL_0_RESET);
}//end reset

void MMC5603_swReset(){
    MMC5603_writeByte(MMC5603NJ_CONTROL_1, MMC5603NJ_CONTROL_1_SW_RESET);
    Task_sleep(5000); //wait for the task to complete (should be enough time)
}//end reset

void MMC5603_selfTest(){

    //define variables
    uint16_t data_set[3];
    uint16_t data_reset[3];
    uint32_t data_delta[3];

    //clear control registers
    MMC5603_writeByte(MMC5603NJ_CONTROL_0, 0x00);
    MMC5603_writeByte(MMC5603NJ_CONTROL_1, 0x00);
    MMC5603_writeByte(MMC5603NJ_CONTROL_2, 0x00);

    //call set function
    MMC5603_set();

    //one-time measurement
    MMC5603_writeByte(MMC5603NJ_CONTROL_0, MMC5603NJ_CONTROL_0_TM_M);
    Task_sleep(5000);

    //retrieve data
    MMC5603_getNJgData(data_set);

    //call reset function
    MMC5603_reset();

    //one-time measurement
    MMC5603_writeByte(MMC5603NJ_CONTROL_0, MMC5603NJ_CONTROL_0_TM_M);
    Task_sleep(5000);

    //retrieve data
    MMC5603_getNJgData(data_reset);

    //determine delta data
    for (int i = 0; i < 3; i++){
        if (data_set[i] > data_reset[i]) {
            data_delta[i] = data_set[i] - data_reset[i];
            Log_info2("set: %d, reset: %d", data_set[i],  data_reset[i]);
        }//end if

        else {
            data_delta[i] = data_reset[i] - data_set[i];
            Log_info2("set: %d, reset: %d", data_set[i],  data_reset[i]);
        }//end else

    }//end for loop

    //print out results
    Log_info1("X axis: %d (should be >100", data_delta[0]);
    Log_info1("Y axis: %d (should be >100", data_delta[1]);
    Log_info1("Z axis: %d (should be >100", data_delta[2]);
}//end selfTest

void MMC5603_getOffset(float * destination){

    //define variables
    uint16_t data_set[3];
    uint16_t data_reset[3];

    //call set function
    MMC5603_set();

    //one-time measurement
    MMC5603_writeByte(MMC5603NJ_CONTROL_0, MMC5603NJ_CONTROL_0_TM_M);
    Task_sleep(5000);

    //retrieve data
    MMC5603_getNJgData(data_set);

    //call reset function
    MMC5603_reset();

    //one-time measurement
    MMC5603_writeByte(MMC5603NJ_CONTROL_0, MMC5603NJ_CONTROL_0_TM_M);
    Task_sleep(5000);

    //retrieve data
    MMC5603_getNJgData(data_reset);

    for (int i = 0; i < 3; i ++){
        destination[i] = ((float)data_set[i] + (float)data_reset[i])/2.0f;
        Log_info1("destination 1: %d", destination[i]);
    }//end for
}//end getOffset


