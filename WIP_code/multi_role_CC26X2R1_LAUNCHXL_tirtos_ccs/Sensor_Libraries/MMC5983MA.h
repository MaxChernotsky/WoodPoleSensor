//contains define functions for registers for the MMC5983MA Magnetometer

#include <stdint.h>

#define MMA5983MA_ADDRESS          0x30

#define MMA5983MA_XOUT_0           0x00
#define MMA5983MA_XOUT_1           0x01
#define MMA5983MA_YOUT_0           0x02
#define MMA5983MA_YOUT_1           0x03
#define MMA5983MA_ZOUT_0           0x04
#define MMA5983MA_ZOUT_1           0x05
#define MMA5983MA_XYZOUT_2         0x06
#define MMA5983MA_TOUT             0x07
#define MMA5983MA_STATUS           0x08
#define MMA5983MA_CONTROL_0        0x09
#define MMA5983MA_CONTROL_1        0x0A
#define MMA5983MA_CONTROL_2        0x0B
#define MMA5983MA_CONTROL_3        0x0C
#define MMA5983MA_PRODUCT_ID       0x2F

//control 0 parameters
#define MMA5983MA_CONTROL_0_TM_M            0x01
#define MMA5983MA_CONTROL_0_TM_T            0x02
#define MMA5983MA_CONTROL_0_INT_MEAS_DONE   0x04
#define MMA5983MA_CONTROL_0_SET             0x08
#define MMA5983MA_CONTROL_0_RESET           0x10
#define MMA5983MA_CONTROL_0_AUTO_SET_RESET  0x20
#define MMA5983MA_CONTROL_0_OTP_READ        0x40

//control 2 parameters
#define MMA5983MA_CONTROL_2_CONT_MODE        0x08
#define MMA5983MA_CONTROL_2_EN_PERIODIC_SET  0x80 //needs continous mode on and auto set reset on



//sample rates
#define MODR_ONESHOT               0x00
#define MODR_1HZ                   0x01
#define MODR_10HZ                  0x02
#define MODR_20HZ                  0x03
#define MODR_50HZ                  0x04
#define MODR_100Hz                 0x05
#define MODR_200Hz                 0x06 //when BW is 0x01 only
#define MODR_1000HZ                0x07 //when BW is 0x11 only

//output resolution/bandwidth
#define BW_100HZ                   0x00 //8ms
#define BW_200HZ                   0x01 //4ms
#define BW_400HZ                   0x02 //2ms
#define BW_800Hz                   0x03 //0.5ms

//pins will determine how often the device will perform a SET operation

#define PRD_SET_1                  0x00
#define PRD_SET_25                 0x01
#define PRD_SET_75                 0x02
#define PRD_SET_100                0x03
#define PRD_SET_250                0x04
#define PRD_SET_500                0x05
#define PRD_SET_1000               0x06
#define PRD_SET_2000               0x07

//function declarations
uint8_t MMC5983_readByte(uint8_t regToRead);
uint8_t MMC5983_getProductID(void);
static void MMC5983_updateTemperature(void);
uint8_t MMC5983_getTemperature (int loops);
uint8_t MMC5983_getStatus(void);
void MMC5983_initMag(void);
uint8_t MMC5983_getXValue();
uint16_t * MMC5983_getMagData ();

