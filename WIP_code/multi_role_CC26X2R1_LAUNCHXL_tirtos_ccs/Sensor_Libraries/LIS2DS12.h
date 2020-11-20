//contains define functions for registers for the LIS2DS12 Accelerometer

//includes
#include <stdint.h>

//function declarations
uint8_t LIS2DS12_readByte(uint8_t regToRead);
void LIS2DS12_writeByte(uint8_t regToWrite, uint8_t byteToWrite);
uint8_t LIS2DS12_getWhoAmI();
uint8_t LIS2DS12_status();
uint8_t LIS2DS12_getTemperature();

//device address
#define LIS2DS12_ADDRESS            0x1D //SDO connected to VCC

//bytes associated to external sensors (UNUSED)
#define LIS2DS12_SENSORHUB_1        0x06
#define LIS2DS12_SENSORHUB_2        0x07
#define LIS2DS12_SENSORHUB_3        0x08
#define LIS2DS12_SENSORHUB_4        0x09
#define LIS2DS12_SENSORHUB_5        0x0A
#define LIS2DS12_SENSORHUB_6        0x0B

#define LIS2DS12_MODULE_OUT         0x0C //read only register (UNUSED)

#define LIS2DS12_WHO_AM_I           0x0F //fixed value output of 43h (hex)

#define LIS2DS12_CONTROL_1          0x20
#define LIS2DS12_CONTROL_2          0x21
#define LIS2DS12_CONTROL_3          0x22
#define LIS2DS12_CONTROL_4          0x23
#define LIS2DS12_CONTROL_5          0x24
#define LIS2DS12_FIFO_CONTROL       0x25

#define LIS2DS12_OUT_T              0x26

#define LIS2DS12_STATUS             0x27

//data output registers
#define LIS2DS12_OUT_X_L            0x28
#define LIS2DS12_OUT_X_H            0x29
#define LIS2DS12_OUT_Y_L            0x2A
#define LIS2DS12_OUT_Y_H            0x2B
#define LIS2DS12_OUT_Z_L            0x2C
#define LIS2DS12_OUT_Z_H            0x2D

#define LIS2DS12_FIFO_THS           0x2E
#define LIS2DS12_FIFO_SRC           0x2F
#define LIS2DS12_FIFO_SAMPLES       0x30

#define LIS2DS12_TAP_THS            0x31
#define LIS2DS12_INT_DURATION       0x32

#define LIS2DS12_WAKE_UP_THS        0x33
#define LIS2DS12_WAKE_UP_DURATION   0x34

#define LIS2DS12_FREE_FALL          0x35
#define LIS2DS12_STATUS_DUP         0x36
#define LIS2DS12_WAKE_UP_SRC        0x37
#define LIS2DS12_TAP_SRC            0x38
#define LIS2DS12_6D_SRC             0x39

#define LIS2DS12_STEP_COUNTER_MINTHS    0x3A
#define LIS2DS12_STEP_COUNTER_L         0x3B
#define LIS2DS12_STEP_COUNTER_H         0x3C

#define LIS2DS12_FUNC_CK_GATE       0x3D
#define LIS2DS12_FUNC_SRC           0x3E
#define LIS2DS12_FUNC_CONTROL       0x3F

//control 1 parameters

//ODR - output data rate and power mode selection
//      PD (power down), LP (low power)
#define LIS2DS12_CONTROL_1_ODR_PD        0x00 //power down mode
#define LIS2DS12_CONTROL_1_ODR_LP_1      0x80 //1Hz
#define LIS2DS12_CONTROL_1_ODR_LP_2      0x90 //12.5Hz
#define LIS2DS12_CONTROL_1_ODR_LP_3      0xA0 //25Hz
#define LIS2DS12_CONTROL_1_ODR_LP_4      0xB0 //50Hz
#define LIS2DS12_CONTROL_1_ODR_LP_5      0xC0 //100Hz
#define LIS2DS12_CONTROL_1_ODR_LP_6      0xD0 //200Hz
#define LIS2DS12_CONTROL_1_ODR_LP_7      0xE0 //400Hz
#define LIS2DS12_CONTROL_1_ODR_LP_8      0xF0 //800Hz

//      HR (high resolution)
#define LIS2DS12_CONTROL_1_ODR_HR_1      0x10 //12.5Hz
#define LIS2DS12_CONTROL_1_ODR_HR_2      0x20 //25Hz
#define LIS2DS12_CONTROL_1_ODR_HR_3      0x30 //50Hz
#define LIS2DS12_CONTROL_1_ODR_HR_4      0x40 //100Hz
#define LIS2DS12_CONTROL_1_ODR_HR_5      0x50 //200Hz
#define LIS2DS12_CONTROL_1_ODR_HR_6      0x60 //400Hz
#define LIS2DS12_CONTROL_1_ODR_HR_7      0x70 //800Hz
//      HF (high frequency)
#define LIS2DS12_CONTROL_1_ODR_HF_1      0x5 //1600Hz
#define LIS2DS12_CONTROL_1_ODR_HF_2      0x06 //3200Hz
#define LIS2DS12_CONTROL_1_ODR_HF_3      0x07 //6400Hz

//full scale selections
#define LIS2DS12_CONTROL_1_FS_2G        0x00 //2g
#define LIS2DS12_CONTROL_1_FS_16G       0x04 //16g
#define LIS2DS12_CONTROL_1_FS_4G        0x08 //4g
#define LIS2DS12_CONTROL_1_FS_8G        0x0C //8g

#define LIS2DS12_CONTROL_1_HF_ODR_EN    0x02 //enable high frequency ODR
#define LIS2DS12_CONTROL_1_BDU          0x01 //block data update (set 1 for output registers not updated until MSB and LSB read), 0 for continuours update

//control 2 parameters
#define LIS2DS12_CONTROL_2_BOOT          0x80 //forces reboot of the flash content
#define LIS2DS12_CONTROL_2_SOFT_RESET    0x40 //reset for all control registers
#define LIS2DS12_CONTROL_2_FUNC_CFG_EN   0x10 //access to pedometer/sensor hub advanced config registers
#define LIS2DS12_CONTROL_2_FDS_SLOPE     0x08 //high pass filter data selection on output register
#define LIS2DS12_CONTROL_2_IF_ADD_INC    0x04 //not sure about this one
#define LIS2DS12_CONTROL_2_I2C_DISABLE   0x02 //disable i2c comms
#define LIS2DS12_CONTROL_2_SIM           0x01 //spi interface mode: 0 - 4 wire, 1 - 3 wire

//control 3 parameters

//self test enables
#define LIS2DS12_CONTROL_3_ST_0          0x00 //normal mode
#define LIS2DS12_CONTROL_3_ST_POS        0x40 //positive sign self test
#define LIS2DS12_CONTROL_3_ST_NEG        0x80 //negative sign self test

//tap enables
#define LIS2DS12_CONTROL_3_TAP_X_EN      0x20 //tap on x enable
#define LIS2DS12_CONTROL_3_TAP_Y_EN      0x10 //tap on y enable
#define LIS2DS12_CONTROL_3_TAP_Z_EN      0x08 //tap on z enable

//interrupts
#define LIS2DS12_CONTROL_3_LIR           0x04 //latches interrupt: 0 - not latched, 1 - latches
#define LIS2DS12_CONTROL_3_H_LACTIVE     0x02 //interrupt active: 0 - high, 1 - low
#define LIS2DS12_CONTROL_3_PP_OD         0x01 //interrupt load: 0 - push-pull, 1 - open drain

//control 4 parameters

//INTERRUPT 1 SETUP
#define LIS2DS12_CONTROL_4_INT1_MASTER_DRDY     0x80 //manage master signal
#define LIS2DS12_CONTROL_4_INT1_S_TAP           0x40 //single tap
#define LIS2DS12_CONTROL_4_INT1_WU              0x20 //wakeup recognition
#define LIS2DS12_CONTROL_4_INT1_FF              0x10 //free-fall recognition
#define LIS2DS12_CONTROL_4_INT1_TAP             0x08 //double-tap recognition
#define LIS2DS12_CONTROL_4_INT1_6D              0x04 //6D recognition
#define LIS2DS12_CONTROL_4_INT1_FTH             0x02 //FIFO threshold
#define LIS2DS12_CONTROL_4_INT1_DRDY            0x01 //data-ready

//control 5 parameter
#define LIS2DS12_CONTROL_5_DRDY_PULSED          0x80 //data ready interrupt mode: 0 - latched, 1 - pulsed
#define LIS2DS12_CONTROL_5_INT2_BOOT            0x40 //boot state
#define LIS2DS12_CONTROL_5_INT2_ON              0x20 //all signal routed on int2 also on int1
#define LIS2DS12_CONTROL_5_INT2_TILT            0x10 //tilt event
#define LIS2DS12_CONTROL_5_INT2_SIG_MOT         0x08 //significant motion detection
#define LIS2DS12_CONTROL_5_INT2_STEP_DET        0x04 //step detection
#define LIS2DS12_CONTROL_5_INT2_FTH             0x02 //fifo threshold
#define LIS2DS12_CONTROL_5_INT2_DRDY            0x01 //data-ready

//FIFO Control parameters

#define LIS2DS12_FIFO_CONTROL_FMODE_OFF         0x00 //bypass mode (FIFO off)
#define LIS2DS12_FIFO_CONTROL_FMODE_2           0x20 //FIFO mode (stops collecting when FIFO full)
#define LIS2DS12_FIFO_CONTROL_FMODE_STREAM      0x60 //continuous to fifo (stream mode)
#define LIS2DS12_FIFO_CONTROL_FMODE_BYPASS      0x80 //bypass to continouus
#define LIS2DS12_FIFO_CONTROL_FMODE_CONT        0xC0 //continuous mode

#define LIS2DS12_FIFO_CONTROL_STEP_COUNTER_INT2_OV      0x10 //step counter overflow interrupt enable on int2
#define LIS2DS12_FIFO_CONTROL_MODULE_TO_FIFO            0x08 //set to 1 to send data to fifo instead of xyz
#define LIS2DS12_FIFO_CONTROL_IF_CS_PU_DIS              0x01 //1 - disconnects pull up in if cs

//tap 6d params
#define LIS2DS12_TAP_THS_4D_EN                  0x80 //enable 4D detection
#define LIS2DS12_TAP_THS_TAP_THS                0x40 //threshold for tap recognition

//6D decoding
#define LIS2DS12_TAP_THS_6D_THS_6               0x00
#define LIS2DS12_TAP_THS_6D_THS_11              0x20
#define LIS2DS12_TAP_THS_6D_THS_16              0x40
#define LIS2DS12_TAP_THS_6D_THS_21              0x60

//int duration parametrs
#define LIS2DS12_INT_DURATION_LAT       0x00 //duration of max time gap for double tap recognition
#define LIS2DS12_INT_DURATION_LAT       0x00 //duration of max time gap for double tap recognition
#define LIS2DS12_INT_DURATION_LAT       0x00 //duration of max time gap for double tap recognition
#define LIS2DS12_INT_DURATION_LAT       0x00 //duration of max time gap for double tap recognition

#define LIS2DS12_INT_DURATION_QUIET     0x00 //expected quiet time after a tap detection
#define LIS2DS12_INT_DURATION_QUIET     0x00

#define LIS2DS12_INT_DURATION_SHOCK     0x00 //max duration of over-threshold event
#define LIS2DS12_INT_DURATION_SHOCK     0x00

//wake up threshold parameters
#define LIS2DS12_WAKE_UP_THS_SINGLE_DOUBLE_TAP      0x80 //0 - single tap, 1 - double tap
#define LIS2DS12_WAKE_UP_THS_SLEEP_ON               0x40 //0 - disabled, 1 - enabled
#define LIS2DS12_WAKE_UP_THS_WH_THS                 0x00 //1LSB = 1/64 of FS

//wake up duration parameters
#define LIS2DS12_WAKE_UP_DURATION_FF            0x80 //free fall duration - in conjunction with FF_DUR
#define LIS2DS12_WAKE_UP_DURATION_WU_DUR        0x00 //wake up duration (2 bytes)
#define LIS2DS12_WAKE_UP_DURATION_INT1_FSS7     0x10 //FIFO flag fss7 is routed to INT1
#define LIS2DS12_WAKE_UP_DURATION_SLEEP_DUR     0x00 //duration to go in sleep mode: 1lsb = 512 TODR

//free fall parameters
#define LIS2DS12_FREE_FALL_FF_THS           0x00 //free fall threshold: 1lsb = 31.25mg
#define LIS2DS12_FREE_FALL_FF_THS_5         0x00 //threshold decoding: 5 LSB
#define LIS2DS12_FREE_FALL_FF_THS_7         0x01 //threshold decoding: 7 LSB
#define LIS2DS12_FREE_FALL_FF_THS_8         0x02 //threshold decoding: 8 LSB
#define LIS2DS12_FREE_FALL_FF_THS_10        0x03 //threshold decoding: 10 LSB
#define LIS2DS12_FREE_FALL_FF_THS_11        0x04 //threshold decoding: 11 LSB
#define LIS2DS12_FREE_FALL_FF_THS_13        0x05 //threshold decoding: 13 LSB
#define LIS2DS12_FREE_FALL_FF_THS_15        0x06 //threshold decoding: 15 LSB
#define LIS2DS12_FREE_FALL_FF_THS_16        0x07 //threshold decoding: 16 LSB










