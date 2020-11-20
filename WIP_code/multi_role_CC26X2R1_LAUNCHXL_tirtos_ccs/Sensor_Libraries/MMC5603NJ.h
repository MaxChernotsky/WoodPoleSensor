//contains define functions for registers for the MMC5603NJ Magnetometer

//includes
#include <stdint.h>

//same device address as the other device thus need to use a secondary i2c bus
#define MMC5603NJ_ADDRESS          0x30

#define MMC5603NJ_XOUT_0           0x00
#define MMC5603NJ_XOUT_1           0x01
#define MMC5603NJ_YOUT_0           0x02
#define MMC5603NJ_YOUT_1           0x03
#define MMC5603NJ_ZOUT_0           0x04
#define MMC5603NJ_ZOUT_1           0x05
#define MMC5603NJ_XOUT_2           0x06
#define MMC5603NJ_YOUT_2           0x07
#define MMC5603NJ_ZOUT_2           0x08

#define MMC5603NJ_TOUT             0x09

#define MMC5603NJ_ODR              0x1A
#define MMC5603NJ_STATUS           0x18
#define MMC5603NJ_CONTROL_0        0x1B
#define MMC5603NJ_CONTROL_1        0x1C
#define MMC5603NJ_CONTROL_2        0x1D

#define MMC5603NJ_ST_X_TH          0x1E
#define MMC5603NJ_ST_Y_TH          0x1F
#define MMC5603NJ_ST_Z_TH          0x20
#define MMC5603NJ_ST_X             0x27
#define MMC5603NJ_ST_Y             0x28
#define MMC5603NJ_ST_Z             0x29

#define MMC5603NJ_PRODUCT_ID       0x39

//internal control 0 parameters
#define MMC5603NJ_CONTROL_0_TM_M            0x01
#define MMC5603NJ_CONTROL_0_TM_T            0x02
#define MMC5603NJ_CONTROL_0_SET             0x08
#define MMC5603NJ_CONTROL_0_RESET           0x10
#define MMC5603NJ_CONTROL_0_AUTO_SR_EN      0x20
#define MMC5603NJ_CONTROL_0_AUTO_ST         0x40 //need to enable ST_X_TH, Y and Z
#define MMC5603NJ_CONTROL_0_CMM_FREQ_EN     0x80

//internal control 1 parameters
#define MMC5603NJ_CONTROL_1_X_INHIBIT        0x04
#define MMC5603NJ_CONTROL_1_Y_INHIBIT        0x08
#define MMC5603NJ_CONTROL_1_Z_INHIBIT        0x10
#define MMC5603NJ_CONTROL_1_ST_ENP           0x20 //self test coil
#define MMC5603NJ_CONTROL_1_ST_ENM           0x40 //self test coil (opposing direction)
#define MMC5603NJ_CONTROL_1_SW_RESET         0x80 //clears all registers and re-read OTP

//bandwidth selection bits - length of the decimation filter
//BW values
#define MMC5603NJ_BW_6_6           0x00 //6.6ms
#define MMC5603NJ_BW_3_5           0x01 //3.5ms
#define MMC5603NJ_BW_2_0           0x02 //2.0ms
#define MMC5603NJ_BW_1_2           0x03 //1.2ms

//internal control 2 parameters

#define MMC5603NJ_PRD_SET_1                  0x00 //number of samples to take
#define MMC5603NJ_PRD_SET_25                 0x01 //to enable PRD EN ensure EN_PRD_SET and AUTO_SR are enabled
#define MMC5603NJ_PRD_SET_75                 0x02
#define MMC5603NJ_PRD_SET_100                0x03
#define MMC5603NJ_PRD_SET_250                0x04
#define MMC5603NJ_PRD_SET_500                0x05
#define MMC5603NJ_PRD_SET_1000               0x06
#define MMC5603NJ_PRD_SET_2000               0x07

#define MMC5603NJ_CONTROL_2_EN_PRD_SET       0x08
#define MMC5603NJ_CONTROL_2_CMM_FREQ_EN      0x10
#define MMC5603NJ_CONTROL_2_HPOWER           0x80 //enables 1000Hz ODR


//function declarations
uint8_t MMC5603_readByte(uint8_t regToRead);
void MMC5603_writeByte(uint8_t regToWrite, uint8_t byteToWrite);
uint8_t MMC5603_getProductID(void);
static void MMC5603_updateTemperature(void);
uint8_t MMC5603_getTemperature (int loops);
uint8_t MMC5603_getStatus(void);
void MMC5603_initMag(void);
void MMC5603_getMagData (uint16_t * destination);
void MMC5603_clearInt();
void MMC5603_reset();
void MMC5603_set();
void MMC5603_swReset();
void MMC5603_selfTest();
void MMC5603_getOffset(float * destination);

