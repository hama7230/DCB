/* 
 * File:   defines004.h
 * Author: shin
 *
 * Created on 2015/02/23, 1:48
 */

#ifndef DEFINES004_H
#define	DEFINES004_H


//== Pulse Width Modulation ===================================================
#define HIGH       (1)
#define LOW        (0)
#define PWM_INPUT_CHANNEL   4
#define PWM_OUTPUT_CHANNEL  4

#define USE_L3GD20  (1)
#define USE_LSM303  (1)
#define USE_BMP085  (1)

#define CH1_IN      (PORTDbits.RD1)
#define CH2_IN      (PORTDbits.RD2)
#define CH3_IN      (PORTDbits.RD3)
#define CH4_IN      (PORTDbits.RD4)
#define CH5_IN      (PORTDbits.RD5)
#define CH6_IN      (PORTDbits.RD6)
#define CH7_IN      (PORTDbits.RD7)
#define CH8_IN      (PORTDbits.RD8)

#define CH1_OUT     (PORTBbits.RB8)
#define CH2_OUT     (PORTBbits.RB9)
#define CH3_OUT     (PORTBbits.RB10)
#define CH4_OUT     (PORTBbits.RB11)
#define CH5_OUT     (PORTBbits.RB12)
#define CH6_OUT     (PORTBbits.RB13)
#define CH7_OUT     (PORTBbits.RB14)
#define CH8_OUT     (PORTBbits.RB15)

#define BaudNORMAL   (9600)
#define BaudHIGH     (115200)

// UART
#define config1 UART_EN | UART_RX_TX | UART_DIS_WAKE | UART_DIS_LOOPBACK | UART_DIS_ABAUD | UART_NO_PAR_8BIT | UART_1STOPBIT | UART_IRDA_DIS | UART_MODE_FLOWCTRL | UART_DIS_BCLK_CTS_RTS | UART_NORMAL_RX | UART_BRGH_SIXTEEN
#define config2 UART_INT_TX_BUF_EMPTY | UART_TX_PIN_LOW | UART_TX_ENABLE | UART_RX_ENABLE | UART_INT_RX_CHAR | UART_ADR_DETECT_DIS | UART_RX_OVERRUN_CLEAR
#define Baud 115200

static char* CHANGE_BAUDRATE_TO_HIGH = "$PMTK251,115200*1F\r\n";
static char COUNT_BAUD               = 20;
static char* CHANGE_HELZ_TO_HIGH     = "$PMTK220,100*2F\r\n";
static char COUNT_HELZ               = 17;
static char* SELECT_SENTENCE         = "$PMTK314,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*29\r\n";
static char COUNT_SENTENCE           = 51;
static char COUNT_GPGGA_FULL         = 73;
static char COUNT_GPGGA_INCOM        = 40;


//= Delay ===================================================================
#define CCLK        (80000000L)
#define CCLK_MS     (CCLK/2/1000)
#define Fsck        (400000)
#define BRG_VAL     (CCLK/2/2/Fsck)
#define Pi          (3.14159265359)
#define R           (6378137)
#define DISTANCE    (50.0)



#if USE_L3GD20
/*=============================================================================
 *
 *  GYRO SCOPE
 *
 * ===========================================================================*/
/*=============================================================================
 *  I2C ADDRESS/BITS AND SETTINGS
 *===========================================================================*/
#define L3GD20_ADDRESS           (0xD6)        // 1101 0110
#define L3GD20_POLL_TIMEOUT      (100)         // Maximum number of read attempts
#define L3GD20_ID                (0xD4)
#define L3GD20H_ID               (0xD7)
#define GYRO_SENSITIVITY_250DPS  (0.00875F)    // Roughly 22/256 for fixed point match
#define GYRO_SENSITIVITY_500DPS  (0.0175F)     // Roughly 45/256
#define GYRO_SENSITIVITY_2000DPS (0.070F)      // Roughly 18/256


/*===========================================================================
 *  REGISTERS
 * =========================================================================*/
                                             // DEFAULT    TYPE
#define GYRO_REGISTER_WHO_AM_I      (0x0F)   // 11010100   r
#define GYRO_REGISTER_CTRL_REG1     (0x20)   // 00000111   rw
#define GYRO_REGISTER_CTRL_REG2     (0x21)   // 00000000   rw
#define GYRO_REGISTER_CTRL_REG3     (0x22)   // 00000000   rw
#define GYRO_REGISTER_CTRL_REG4     (0x23)   // 00000000   rw
#define GYRO_REGISTER_CTRL_REG5     (0x24)   // 00000000   rw
#define GYRO_REGISTER_REFERENCE     (0x25)   // 00000000   rw
#define GYRO_REGISTER_OUT_TEMP      (0x26)   //            r
#define GYRO_REGISTER_STATUS_REG    (0x27)   //            r
#define GYRO_REGISTER_OUT_X_L       (0x28)   //            r
#define GYRO_REGISTER_OUT_X_H       (0x29)   //            r
#define GYRO_REGISTER_OUT_Y_L       (0x2A)   //            r
#define GYRO_REGISTER_OUT_Y_H       (0x2B)   //            r
#define GYRO_REGISTER_OUT_Z_L       (0x2C)   //            r
#define GYRO_REGISTER_OUT_Z_H       (0x2D)   //            r
#define GYRO_REGISTER_FIFO_CTRL_REG (0x2E)   // 00000000   rw
#define GYRO_REGISTER_FIFO_SRC_REG  (0x2F)   //            r
#define GYRO_REGISTER_INT1_CFG      (0x30)   // 00000000   rw
#define GYRO_REGISTER_INT1_SRC      (0x31)   //            r
#define GYRO_REGISTER_TSH_XH        (0x32)   // 00000000   rw
#define GYRO_REGISTER_TSH_XL        (0x33)   // 00000000   rw
#define GYRO_REGISTER_TSH_YH        (0x34)   // 00000000   rw
#define GYRO_REGISTER_TSH_YL        (0x35)   // 00000000   rw
#define GYRO_REGISTER_TSH_ZH        (0x36)   // 00000000   rw
#define GYRO_REGISTER_TSH_ZL        (0x37)   // 00000000   rw
#define GYRO_REGISTER_INT1_DURATION (0x38)   // 00000000   rw


/*=========================================================================
 *  OPTIONAL SPEED SETTINGS
 *===========================================================================*/
#define GYRO_RANGE_250DPS   (250)
#define GYRO_RANGE_500DPS   (500)
#define GYRO_RANGE_2000DPS  (2000)

#define ANGLE_OFFSET (90.0)

#endif

#if USE_LSM303
/*=============================================================================
 *
 *  ACCEL AND MAGNE
 *
 * ===========================================================================*/
/*=============================================================================
 *  I2C ADDRESS/BITS AND SETTINGS
 *===========================================================================*/
#define LSM303_ADDRESS_ACCEL          (0x32)         // 0011 0010
#define LSM303_ADDRESS_MAG            (0x3C)         // 0011 1100


/*===========================================================================
 *  REGISTERS
 * =========================================================================*/
                                                         // DEFAULT    TYPE
#define LSM303_REGISTER_ACCEL_CTRL_REG1_A       (0x20)   // 00000111   rw
#define LSM303_REGISTER_ACCEL_CTRL_REG2_A       (0x21)   // 00000000   rw
#define LSM303_REGISTER_ACCEL_CTRL_REG3_A       (0x22)   // 00000000   rw
#define LSM303_REGISTER_ACCEL_CTRL_REG4_A       (0x23)   // 00000000   rw
#define LSM303_REGISTER_ACCEL_CTRL_REG5_A       (0x24)   // 00000000   rw
#define LSM303_REGISTER_ACCEL_CTRL_REG6_A       (0x25)   // 00000000   rw
#define LSM303_REGISTER_ACCEL_REFERENCE_A       (0x26)   // 00000000   r
#define LSM303_REGISTER_ACCEL_STATUS_REG_A      (0x27)   // 00000000   r
#define LSM303_REGISTER_ACCEL_OUT_X_L_A         (0x28)
#define LSM303_REGISTER_ACCEL_OUT_X_H_A         (0x29)
#define LSM303_REGISTER_ACCEL_OUT_Y_L_A         (0x2A)
#define LSM303_REGISTER_ACCEL_OUT_Y_H_A         (0x2B)
#define LSM303_REGISTER_ACCEL_OUT_Z_L_A         (0x2C)
#define LSM303_REGISTER_ACCEL_OUT_Z_H_A         (0x2D)
#define LSM303_REGISTER_ACCEL_FIFO_CTRL_REG_A   (0x2E)
#define LSM303_REGISTER_ACCEL_FIFO_SRC_REG_A    (0x2F)
#define LSM303_REGISTER_ACCEL_INT1_CFG_A        (0x30)
#define LSM303_REGISTER_ACCEL_INT1_SOURCE_A     (0x31)
#define LSM303_REGISTER_ACCEL_INT1_THS_A        (0x32)
#define LSM303_REGISTER_ACCEL_INT1_DURATION_A   (0x33)
#define LSM303_REGISTER_ACCEL_INT2_CFG_A        (0x34)
#define LSM303_REGISTER_ACCEL_INT2_SOURCE_A     (0x35)
#define LSM303_REGISTER_ACCEL_INT2_THS_A        (0x36)
#define LSM303_REGISTER_ACCEL_INT2_DURATION_A   (0x37)
#define LSM303_REGISTER_ACCEL_CLICK_CFG_A       (0x38)
#define LSM303_REGISTER_ACCEL_CLICK_SRC_A       (0x39)
#define LSM303_REGISTER_ACCEL_CLICK_THS_A       (0x3A)
#define LSM303_REGISTER_ACCEL_TIME_LIMIT_A      (0x3B)
#define LSM303_REGISTER_ACCEL_TIME_LATENCY_A    (0x3C)
#define LSM303_REGISTER_ACCEL_TIME_WINDOW_A     (0x3D)

#define LSM303_REGISTER_MAG_CRA_REG_M           (0x00)
#define LSM303_REGISTER_MAG_CRB_REG_M           (0x01)
#define LSM303_REGISTER_MAG_MR_REG_M            (0x02)
#define LSM303_REGISTER_MAG_OUT_X_H_M           (0x03)
#define LSM303_REGISTER_MAG_OUT_X_L_M           (0x04)
#define LSM303_REGISTER_MAG_OUT_Z_H_M           (0x05)
#define LSM303_REGISTER_MAG_OUT_Z_L_M           (0x06)
#define LSM303_REGISTER_MAG_OUT_Y_H_M           (0x07)
#define LSM303_REGISTER_MAG_OUT_Y_L_M           (0x08)
#define LSM303_REGISTER_MAG_SR_REG_Mg           (0x09)
#define LSM303_REGISTER_MAG_IRA_REG_M           (0x0A)
#define LSM303_REGISTER_MAG_IRB_REG_M           (0x0B)
#define LSM303_REGISTER_MAG_IRC_REG_M           (0x0C)
#define LSM303_REGISTER_MAG_TEMP_OUT_H_M        (0x31)
#define LSM303_REGISTER_MAG_TEMP_OUT_L_M        (0x32)
/*=========================================================================*/

/*=========================================================================
 * MAGNETOMETER GAIN SETTINGS
 * ==========================================================================*/
#define LSM303_MAGGAIN_1_3      (0x20)  // +/- 1.3
#define LSM303_MAGGAIN_1_9      (0x40) // +/- 1.9
#define LSM303_MAGGAIN_2_5      (0x60)  // +/- 2.5
#define LSM303_MAGGAIN_4_0      (0x80)  // +/- 4.0
#define LSM303_MAGGAIN_4_7      (0xA0)  // +/- 4.7
#define LSM303_MAGGAIN_5_6      (0xC0)  // +/- 5.6
#define LSM303_MAGGAIN_8_1      (0xE0)   // +/- 8.1

unsigned char magGain;
int lsm303Mag_Gauss_LSB_XY;
int lsm303Mag_Gauss_LSB_Z;
static float _lsm303Accel_MG_LSB     = 0.001F;   // 1, 2, 4 or 12 mg per lsb
static float _lsm303Mag_Gauss_LSB_XY = 1100.0F;  // Varies with gain

#endif

#if USE_BMP085
/*=============================================================================
 *
 *  PRESSURE AND Temperature
 *
 * ===========================================================================*/
/*=============================================================================
 *  I2C ADDRESS/BITS AND SETTINGS
 *===========================================================================*/
#define BMP085_ADDRESS      (0xEE)          // 1110 1110

/*===========================================================================
 *  REGISTERS
 * =========================================================================*/
                                                         // DEFAULT    TYPE
#define BMP085_REGISTER_CAL_AC1             (0xAA)  // R   Calibration data (16 bits)
#define BMP085_REGISTER_CAL_AC2             (0xAC)  // R   Calibration data (16 bits)
#define BMP085_REGISTER_CAL_AC3             (0xAE)  // R   Calibration data (16 bits)
#define BMP085_REGISTER_CAL_AC4             (0xB0)  // R   Calibration data (16 bits)
#define BMP085_REGISTER_CAL_AC5             (0xB2)  // R   Calibration data (16 bits)
#define BMP085_REGISTER_CAL_AC6             (0xB4)  // R   Calibration data (16 bits)
#define BMP085_REGISTER_CAL_B1              (0xB6)  // R   Calibration data (16 bits)
#define BMP085_REGISTER_CAL_B2              (0xB8)  // R   Calibration data (16 bits)
#define BMP085_REGISTER_CAL_MB              (0xBA)  // R   Calibration data (16 bits)
#define BMP085_REGISTER_CAL_MC              (0xBC)  // R   Calibration data (16 bits)
#define BMP085_REGISTER_CAL_MD              (0xBE)  // R   Calibration data (16 bits)
#define BMP085_REGISTER_CHIPID              (0xD0)
#define BMP085_REGISTER_VERSION             (0xD1)
#define BMP085_REGISTER_SOFTRESET           (0xE0)
#define BMP085_REGISTER_CONTROL             (0xF4)
#define BMP085_REGISTER_TEMPDATA            (0xF6)
#define BMP085_REGISTER_PRESSUREDATA        (0xF6)
#define BMP085_REGISTER_READTEMPCMD         (0x2E)
#define BMP085_REGISTER_READPRESSURECMD     (0x34)

/*=========================================================================
 * MODE SETTINGS
 * =========================================================================*/
#define BMP085_MODE_ULTRALOWPOWER           (0)
#define BMP085_MODE_STANDARD                (1)
#define BMP085_MODE_HIGHRES                 (2)
#define BMP085_MODE_ULTRAHIGHRES            (3)

/*=========================================================================
 * CALIBRATION DATA
 * ==========================================================================*/
short  ac1;                   //int16_t
short  ac2;                   //int16_t
short  ac3;                   //int16_t
unsigned short ac4;           //uint16_t
unsigned short ac5;           //uint16_t
unsigned short ac6;           //uint16_t
short  b1;                    //int16_t
short  b2;                    //int16_t
short  mb;                    //int16_t
short  mc;                    //int16_t
short  md;                    //int16_t

// static unsigned char     _bmp085Mode = BMP085_MODE_ULTRAHIGHRES;
static unsigned char _bmp085Mode = BMP085_MODE_ULTRALOWPOWER;

#endif


#endif	/* DEFINES004_H */

