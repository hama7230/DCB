/*
 * File:   main_dcb004.c
 * Author: shin
 *
 * Created on 2015/02/23, 1:52
 */



//== Library ================================================================
#include <xc.h>
#include <plib.h>
#include <stdio.h>
#include <math.h>
#include "defines004.h"

// DEVCFG3
// USERID = No Setting
#pragma config FSRSSEL = PRIORITY_7     // Shadow Register Set Priority Select (SRS Priority 7)
#pragma config PMDL1WAY = OFF           // Peripheral Module Disable Configuration (Allow multiple reconfigurations)
#pragma config IOL1WAY = OFF            // Peripheral Pin Select Configuration (Allow multiple reconfigurations)

// DEVCFG2
#pragma config FPLLIDIV = DIV_2        // PLL Input Divider (12x Divider)
#pragma config FPLLMUL = MUL_20         // PLL Multiplier (24x Multiplier)
#pragma config FPLLODIV = DIV_1       // System PLL Output Clock Divider (PLL Divide by 256)

// DEVCFG1
#pragma config FNOSC = FRCPLL           // Oscillator Selection Bits (Fast RC Osc w/Div-by-N (FRCDIV))
#pragma config FSOSCEN = OFF             // Secondary Oscillator Enable (Enabled)
#pragma config IESO = ON                // Internal/External Switch Over (Enabled)
#pragma config POSCMOD = OFF            // Primary Oscillator Configuration (Primary osc disabled)
#pragma config OSCIOFNC = OFF           // CLKO Output Signal Active on the OSCO Pin (Disabled)
#pragma config FPBDIV = DIV_2           // Peripheral Clock Divisor (Pb_Clk is Sys_Clk/8)
#pragma config FCKSM = CSDCMD           // Clock Switching and Monitor Selection (Clock Switch Disable, FSCM Disabled)
#pragma config WDTPS = PS1048576        // Watchdog Timer Postscaler (1:1048576)
#pragma config WINDIS = OFF             // Watchdog Timer Window Enable (Watchdog Timer is in Non-Window Mode)
#pragma config FWDTEN = OFF              // Watchdog Timer Enable (WDT Enabled)
#pragma config FWDTWINSZ = WINSZ_25     // Watchdog Timer Window Size (Window Size is 25%)

// DEVCFG0
#pragma config DEBUG = OFF              // Background Debugger Enable (Debugger is Disabled)
#pragma config JTAGEN = OFF              // JTAG Enable (JTAG Port Enabled)
#pragma config ICESEL = ICS_PGx1        // ICE/ICD Comm Channel Select (Communicate on PGEC1/PGED1)
#pragma config PWP = OFF                // Program Flash Write Protect (Disable)
#pragma config BWP = OFF                // Boot Flash Write Protect bit (Protection Disabled)
#pragma config CP = OFF                 // Code Protect (Protection Disabled)


volatile unsigned int pw[9];
volatile unsigned int rise[9];
static volatile char flag[5] = {0,0,0,0,0};

char gps[] = "-------------------------------------------------------------------------\r\n";
char gps_flag = 0;



/*******************************************************************************
 * PWM Servo
 ******************************************************************************/
void PWMInit(void);
void delay_ms(unsigned int d);

/*******************************************************************************
 * UART GPS
 ******************************************************************************/
void U1string(unsigned char *tex, char count);
void NMEAInit(void);
void getLATILONG(float* latitude, float* longitude);

/*******************************************************************************
 * I2C IMU
 ******************************************************************************/
void write8(unsigned char address, unsigned char reg, unsigned char value);
//== Gyro Sensor =============================================================
#if USE_L3GD20
 void l3gd20Init(void);
 void l3gd20Read(float *x, float *y, float *z);
#endif
#if USE_LSM303
 void lsm303dlhcAccelInit(void);
 void lsm303dlhcAccelRead(float *x, float *y, float *z);
 void lsm303dlhcMagInit(void);
 void lsm303dlhcMagRead(float *x, float *y, float *z);
 void setMagGain(unsigned char gain);
 float getAngle(void);
 float getfusionAngle(void);
#endif
#if USE_BMP085
 void read8(unsigned char reg, unsigned char *value);
 void read16(unsigned char reg, unsigned short *value);
 void readS16(unsigned char reg, short *value);
 void readCoefficients(void);
 int computeB5(int ut);
 void readRawTemperature(int *temperature);
 void readRawPressure(int *pressure);
 void getTemperature(float *temp);
 void getPressure(float *pressure);
 float getAltitude(float *pressure, float *temp);
#endif


void AutoControl(float *lati, float *longi);


//== Main =====================================================================
int main(void) {

    //== OSC =================================================
    SYSTEMConfigPerformance(80000000);  //80MHz


    PWMInit();
    NMEAInit();

    //---I2C Initialize---//
    OpenI2C1(I2C_EN, BRG_VAL);

    // Initialize
    l3gd20Init();
    lsm303dlhcAccelInit();
    lsm303dlhcMagInit();
    readCoefficients();

    //== UART ===================================================
    ANSELG = 0x0000;
    RPG7R = 0b0001;     // TX
    U2RXR = 0b0001;     // G8
    OpenUART2(config1, config2, (unsigned int)(5000000 / Baud - 1));
    ConfigIntUART2(UART_ERR_INT_DIS | UART_RX_INT_EN | UART_INT_PR1 | UART_TX_INT_DIS);

    INTEnableSystemMultiVectoredInt();


    float lati, longi;
    float x, y, z, direct;

    float angle_offset;
    float angle;
    float operate;
    unsigned int pulse;

    ANSELBbits.ANSB3 = 0;
    TRISBbits.TRISB3 = 0;
    LATBbits.LATB3 = 1;
    ANSELBbits.ANSB4 = 0;
    TRISBbits.TRISB4 = 0;
    LATBbits.LATB4 = 1;

    //== Loop ===============================================
    while(1)
    {
       delay_ms(100);


       if(pw[3] > 1720*40)
           AutoControl(&lati, &longi);


        printf("PWM : %6u %6u %6u %6u\r\n", pw[1] / 40, pw[2] / 40, pw[3] / 40, pw[4] / 40);

        if(gps_flag == 1)
        {
            unsigned int start = ReadCoreTimer();
            getGPS(&lati, &longi);
            printf("GPS : %f %f %u\r\n", lati, longi, ReadCoreTimer() - start);
            // printf("%s\r\n", gps);
            gps_flag = 0;
        }
        else
        {
            // printf("GPS Not available\r\n");
            printf("Fail: %s", gps);
        }
        
        lsm303dlhcAccelRead(&x, &y, &z);
        printf("IMUA: %f %f %f\r\n", x, y, z);
        l3gd20Read(&x, &y, &z);
        printf("IMUG: %f %f %f\r\n", x, y, z);
        lsm303dlhcMagRead(&x, &y, &z);
        printf("IMUM: %f %f %f\r\n", x, y, z);

        float t, p;
        unsigned int start = ReadCoreTimer();
        getTemperature(&t);
        getPressure(&p);
        float alt = getAltitude(&p, &t);
        printf("%f %u\r\n", alt, ReadCoreTimer() - start);

        printf("ANGL: %f %f\r\n", getAngle(), getfusionAngle());
    }

    return 0;
}


//== Change Notice Interrupt ===========================================================
void __ISR(_CHANGE_NOTICE_VECTOR, ipl6) ChangeNotice_Handler(void)
{
    unsigned int now = ReadCoreTimer();

    

    //== 1ch =================================================================
    if(CH1_IN == HIGH && flag[1] == 0)
    {
        // rise[1] = ReadCoreTimer();
        rise[1] = now;
        flag[1] = 1;
    }
    else if(CH1_IN == LOW && flag[1] == 1)
    {
        pw[1] = (now - rise[1]);
        flag[1] = 0;
    }

    //= 2ch ===================================================================
    if(CH2_IN == HIGH && flag[2] == 0)
    {
        rise[2] = now;
        flag[2] = 1;
    }
    else if(CH2_IN == LOW && flag[2] == 1)
    {
        pw[2] = (now - rise[2]);
        flag[2] = 0;
    }

#if PWM_INPUT_CHANNEL >= 4

    //== 3ch =================================================================
    if(CH3_IN == HIGH && flag[3] == 0)
    {
        rise[3] = now;
        flag[3] = 1;
    }
    else if(CH3_IN == LOW && flag[3] == 1)
    {
        pw[3] = (now - rise[3]);
        flag[3] = 0;
    }

    //= 4ch ===================================================================
    if(CH4_IN == HIGH && flag[4] == 0)
    {
        rise[4] = now;
        flag[4] = 1;
    }
    else if(CH4_IN == LOW && flag[4] == 1)
    {
        pw[4] = (now - rise[4]);
        flag[4] = 0;
    }
#endif
#if PWM_INPUT_CHANNEL == 8
    //== 5ch =================================================================
    if(CH5_IN == HIGH && flag[5] == 0)
    {
        rise[5] = now;
        flag[5] = 1;
    }
    else if(CH5_IN == LOW && flag[5] == 1)
    {
        pw[5] = (now - rise[5]);
        flag[5] = 0;
    }

    //= 6ch ===================================================================
    if(CH6_IN == HIGH && flag[6] == 0)
    {
        rise[6] = now;
        flag[6] = 1;
    }
    else if(CH6_IN == LOW && flag[6] == 1)
    {
        pw[6] = (now - rise[6]);
        flag[6] = 0;
    }

    //== 7ch =================================================================
    if(CH7_IN == HIGH && flag[7] == 0)
    {
        rise[7] = now;
        flag[7] = 1;
    }
    else if(CH7_IN == LOW && flag[7] == 1)
    {
        pw[7] = (now - rise[7]);
        flag[7] = 0;
    }

    //= 8ch ===================================================================
    if(CH8_IN == HIGH && flag[8] == 0)
    {
        rise[8] = now;
        flag[8] = 1;
    }
    else if(CH8_IN == LOW && flag[8] == 1)
    {
        pw[8] = (now - rise[8]);
        flag[8] = 0;
    }

#endif

    // Clear interrupt flag
    mCNDClearIntFlag();
}

void __ISR(_TIMER_1_VECTOR, ipl2) _Timer1Handler(void) {

    //== Clear Interrupt Flag ===========================================
    mT1ClearIntFlag();

    //== Variable ================================================
    volatile unsigned int rise, pulseWidth;
    char flag_t[5] = {0,0,0,0,0};
    rise = ReadCoreTimer();


#if PWM_OUTPUT_CHANNEL == 2

    CH1_OUT = HIGH;
    CH2_OUT = HIGH;

    //== Loop ===================================================
    while(!((flag_t[1] == 1) && (flag_t[2] == 1) ))
    {
        pulseWidth = ReadCoreTimer() - rise;

        //== 1ch =================================================================
        if(pw[1] < pulseWidth)
        {
            CH1_OUT = LOW;
            flag_t[1] = 1;
        }
        else
            CH1_OUT = HIGH;

        //== 2ch =================================================================
        if(pw[2] < pulseWidth)
        {
            CH2_OUT = LOW;
            flag_t[2] = 1;
        }
        else
            CH2_OUT = HIGH;
    }
#endif
#if PWM_OUTPUT_CHANNEL == 4

    CH1_OUT = HIGH;
    CH2_OUT = HIGH;
    CH3_OUT = HIGH;
    CH4_OUT = HIGH;

    //== Loop ===================================================
    while(!((flag_t[1] == 1) && (flag_t[2] == 1) && (flag_t[3] == 1) && (flag_t[4] == 1)))
    {
        pulseWidth = ReadCoreTimer() - rise;

        //== 1ch =================================================================
        if(pw[1] < pulseWidth)
        {
            CH1_OUT = LOW;
            flag_t[1] = 1;
        }
        else
            CH1_OUT = HIGH;

        //== 2ch =================================================================
        if(pw[2] < pulseWidth)
        {
            CH2_OUT = LOW;
            flag_t[2] = 1;
        }
        else
            CH2_OUT = HIGH;

        //== 3ch =================================================================
        if(pw[3] < pulseWidth)
        {
            CH3_OUT = LOW;
            flag_t[3] = 1;
        }
        else
            CH3_OUT = HIGH;

        //== 4ch =================================================================
        if(pw[4] < pulseWidth)
        {
            CH4_OUT = LOW;
            flag_t[4] = 1;
        }
        else
            CH4_OUT = HIGH;
    }
#endif
#if PWM_OUTPUT_CHANNEL == 8

    CH1_OUT = HIGH;
    CH2_OUT = HIGH;
    CH3_OUT = HIGH;
    CH4_OUT = HIGH;
    CH5_OUT = HIGH;
    CH6_OUT = HIGH;
    CH7_OUT = HIGH;
    CH8_OUT = HIGH;

    //== Loop ===================================================
    while(!((flag_t[1] == 1) && (flag_t[2] == 1) && (flag_t[3] == 1) && (flag_t[4] == 1)
            && (flag_t[5] == 1) && (flag_t[6] == 1) && (flag_t[7] == 1) && (flag_t[8] == 1)))
    {
        pulseWidth = ReadCoreTimer() - rise;

        if(pw[1] < pulseWidth)
        {
            CH1_OUT = LOW;
            flag_t[1] = 1;
        }
        else
            CH1_OUT = HIGH;

        if(pw[2] < pulseWidth)
        {
            CH2_OUT = LOW;
            flag_t[2] = 1;
        }
        else
            CH2_OUT = HIGH;

        if(pw[3] < pulseWidth)
        {
            CH3_OUT = LOW;
            flag_t[3] = 1;
        }
        else
            CH3_OUT = HIGH;

        if(pw[4] < pulseWidth)
        {
            CH4_OUT = LOW;
            flag_t[4] = 1;
        }
        else
            CH4_OUT = HIGH;

        //== 5ch =================================================================
        if(pw[5] < pulseWidth)
        {
            CH5_OUT = LOW;
            flag_t[5] = 1;
        }
        else
            CH5_OUT = HIGH;

        if(pw[6] < pulseWidth)
        {
            CH6_OUT = LOW;
            flag_t[6] = 1;
        }
        else
            CH6_OUT = HIGH;

        if(pw[7] < pulseWidth)
        {
            CH7_OUT = LOW;
            flag_t[7] = 1;
        }
        else
            CH8_OUT = HIGH;

        if(pw[8] < pulseWidth)
        {
            CH8_OUT = LOW;
            flag_t[8] = 1;
        }
        else
            CH8_OUT = HIGH;
    }
#endif

} // Timer Interrupt Ending

void __ISR(_UART_2_VECTOR, ipl1) U2RXHandler(void)
{
    char RcvData;
    int i = 0;

    PORTBbits.RB3 = 1;

    do {
        while(!U2STAbits.URXDA);

        // receive error check
        if(U2STAbits.OERR || U2STAbits.FERR)
        {
            // error flag clear
            U2STA &= 0xfff0;
            gps_flag = 0;
            PORTBbits.RB3 = 0;
            mU2RXClearIntFlag();
            return;
        }
        else
        {
            RcvData = getcUART2();
            gps[i] = RcvData;
            i++;
        }

    } while(RcvData != '\n');


    if(i > 50)
        gps_flag = 1;
    else
        gps_flag = 0;

    PORTBbits.RB3 = 0;

   mU2RXClearIntFlag();
}


void delay_ms(unsigned int d)
{
    unsigned int Delay_Count, StartTime;
    StartTime = ReadCoreTimer();
    Delay_Count = d*CCLK_MS;
    while ((ReadCoreTimer() - StartTime) < Delay_Count);
}

//== PWM =========================================================================================
inline void PWMInit(void)
{
    //== PORT B ================================================
    ANSELB = 0x00000000;                // ALL Digital
    TRISB  = 0x00000000;                // ALL Output
    LATB   = 0;                         // Initialize

    //== PORT D ================================================
    ANSELD = 0x00000000;                // ALL Digital
    // TRISD  = 0xffffffff;                // ALL Input

#if PWM_INPUT_CHANNEL == 2
    TRISDbits.TRISD1 = 1;               // D1-4 Input
    TRISDbits.TRISD2 = 1;
    //== CN INT Config ============================================
    mCNDOpen(CND_ON | CND_IDLE_CON, CND1_ENABLE | CND2_ENABLE, CNB1_PULLUP_ENABLE | CNB2_PULLUP_ENABLE);
#elif PWM_INPUT_CHANNEL >= 4
    TRISDbits.TRISD1 = 1;               // D1-4 Input
    TRISDbits.TRISD2 = 1;
    TRISDbits.TRISD3 = 1;               // D1-4 Input
    TRISDbits.TRISD4 = 1;
    //== CN INT Config ============================================
//    mCNDOpen(CND_ON | CND_IDLE_CON, CND1_ENABLE | CND2_ENABLE | CND3_ENABLE | CND4_ENABLE,
//            CNB1_PULLUP_ENABLE | CNB2_PULLUP_ENABLE | CNB3_PULLUP_ENABLE | CNB4_PULLUP_ENABLE);
    mCNDOpen(CND_ON | CND_IDLE_CON, CND1_ENABLE | CND2_ENABLE | CND3_ENABLE | CND4_ENABLE, 0);
#elif PWM_INPUT_CHANNEL == 8
    TRISDbits.TRISD5 = 1;               // D1-4 Input
    TRISDbits.TRISD6 = 1;
    TRISDbits.TRISD7 = 1;
    TRISDbits.TRISD0 = 1;
    //== CN INT Config ============================================
    mCNDOpen(CND_ON | CND_IDLE_CON, CND1_ENABLE | CND2_ENABLE | CND3_ENABLE | CND4_ENABLE
                                  | CND5_ENABLE | CND6_ENABLE | CND7_ENABLE | CND0_ENABLE
            , CNB1_PULLUP_ENABLE | CNB2_PULLUP_ENABLE | CNB3_PULLUP_ENABLE | CNB4_PULLUP_ENABLE
              CNB5_PULLUP_ENABLE | CNB6_PULLUP_ENABLE | CNB7_PULLUP_ENABLE | CNB0_PULLUP_ENABLE);
#endif

    // Clear mismatch condition
    mPORTDRead();

    ConfigIntCND(CHANGE_INT_ON | CHANGE_INT_PRI_6);

    //Open Timer1 with 1:64 prescaler (80MHz -> 1250000Hz), with period of 10, therefore tick = 500Hz.
    OpenTimer1(T1_ON | T1_PS_1_64 | T1_SOURCE_INT, 25000);
    ConfigIntTimer1(T1_INT_ON | T1_INT_PRIOR_2);

}


//== GPS ==========================================================================================
//== U1string ==============================================================
void U1string(unsigned char *tex, char count)
{
    char i;

    for(i = 0; i < count; i++)
    {
        while(U1STAbits.UTXBF);
        U1TXREG = tex[i];
    }
}

void NMEAInit(void)
{
    ANSELF = 0x0000;
    TRISFbits.TRISF4 = 1;
    TRISFbits.TRISF5 = 0;
    RPF5R = 0b011; //TX
    U1RXR = 0b010; // F4
    U1MODE = 0x00000000;    //Clear
    U1MODEbits.UARTEN = 1;  //15_UART_Enable
    U1MODEbits.RTSMD = 1;   //11_SimplexMode
    U1MODEbits.BRGH = 0;    //3_StandardSpeed*16
    U1STA = 0x00000000;     //Clear
    U1STAbits.UTXEN = 1;    //10_TX_enable
    U1BRG = (unsigned int)(5000000 / BaudNORMAL - 1);            //U1BRG=(Fcy/(16*9600))-1
   // OpenUART1(config1, config2, (unsigned int)(5000000 / BaudHIGH - 1));
    delay_ms(100);
    U1string(CHANGE_BAUDRATE_TO_HIGH, COUNT_BAUD);
    delay_ms(100);
    U1BRG = (unsigned int)(5000000 / BaudHIGH - 1);
    delay_ms(100);
    // OpenUART1(config1, config2, (unsigned int)(5000000 / BaudHIGH - 1));
    delay_ms(100);
    U1string(CHANGE_HELZ_TO_HIGH, COUNT_HELZ);
    delay_ms(100);
    U1string(SELECT_SENTENCE, COUNT_SENTENCE);

}


void getGPS(float* latitude, float* longitude)
{
    float works;
    // $GPGGA,064334.000,3606.7769,N,13614.4859,E,1,06,1.73,0.9,M,34.6,M,,*6C
    // $GPGGA,045007.700,3606.7675,N,13614.5014,E,1,06,1.26,101.1,M,34.6,M,,*6F
    // 01234567890123456789012345678901234567890123456789012345678901234567890123

    if(gps[0] == '$'){
        if(gps[1] == 'G'){
            if(gps[2] = 'P'){
                if(gps[3] = 'G'){
                    if(gps[4] = 'G'){
                        if(gps[5] = 'A'){
                            works =   (gps[20] - '0') * 10.0
                                    + (gps[21] - '0')
                                    + (gps[23] - '0') * 0.1
                                    + (gps[24] - '0') * 0.01
                                    + (gps[25] - '0') * 0.001
                                    + (gps[26] - '0') * 0.0001;
                            *latitude = (float)((gps[18] - '0')*10.0 + (gps[19] - '0') + works/60.0);


                            works =   (gps[33] - '0') * 10.0
                                    + (gps[34] - '0')
                                    + (gps[36] - '0') * 0.1
                                    + (gps[37] - '0') * 0.01
                                    + (gps[38] - '0') * 0.001
                                    + (gps[39] - '0') * 0.0001;
                            *longitude = (float)((gps[30] - '0')*100.0 + (gps[31] - '0')*10.0 + (gps[32] - '0') + works/60.0);

                            return;
                        }
                    }

                }
            }
        }
    }

    *latitude = 0;
    *longitude = 0;


}



void write8(unsigned char address, unsigned char reg, unsigned char value)
 {
    // Start Conditinon
    StartI2C1();
    IdleI2C1();
    // Address
    MasterWriteI2C1(address);
    IdleI2C1();
    // Register
    MasterWriteI2C1(reg);
    IdleI2C1();
    // value
    MasterWriteI2C1(value);
    IdleI2C1();
    // Stop Condition
    StopI2C1();
    IdleI2C1();
 }

#if USE_L3GD20
 void l3gd20Init()
 {
    /* Set CTRL_REG1 (0x20)
    ====================================================================
    BIT  Symbol    Description                                   Default
    ---  ------    --------------------------------------------- -------
    7-6  DR1/0     Output data rate                                   00
    5-4  BW1/0     Bandwidth selection                                00
      3  PD        0 = Power-down mode, 1 = normal/sleep mode          0
      2  ZEN       Z-axis enable (0 = disabled, 1 = enabled)           1
      1  YEN       Y-axis enable (0 = disabled, 1 = enabled)           1
      0  XEN       X-axis enable (0 = disabled, 1 = enabled)           1 */

    /* Reset then switch to normal mode and enable all three channels */
    write8(L3GD20_ADDRESS, GYRO_REGISTER_CTRL_REG1, 0x00);
    write8(L3GD20_ADDRESS, GYRO_REGISTER_CTRL_REG1, 0x0F);
    /* ------------------------------------------------------------------ */

    /* Set CTRL_REG2 (0x21)
    ====================================================================
    BIT  Symbol    Description                                   Default
    ---  ------    --------------------------------------------- -------
    5-4  HPM1/0    High-pass filter mode selection                    00
    3-0  HPCF3..0  High-pass filter cutoff frequency selection      0000 */

    /* Nothing to do ... keep default values */
    /* ------------------------------------------------------------------ */

    /* Set CTRL_REG3 (0x22)
    ====================================================================
    BIT  Symbol    Description                                   Default
    ---  ------    --------------------------------------------- -------
     7  I1_Int1   Interrupt enable on INT1 (0=disable,1=enable)       0
     6  I1_Boot   Boot status on INT1 (0=disable,1=enable)            0
     5  H-Lactive Interrupt active config on INT1 (0=high,1=low)      0
     4  PP_OD     Push-Pull/Open-Drain (0=PP, 1=OD)                   0
     3  I2_DRDY   Data ready on DRDY/INT2 (0=disable,1=enable)        0
     2  I2_WTM    FIFO wtrmrk int on DRDY/INT2 (0=dsbl,1=enbl)        0
     1  I2_ORun   FIFO overrun int on DRDY/INT2 (0=dsbl,1=enbl)       0
     0  I2_Empty  FIFI empty int on DRDY/INT2 (0=dsbl,1=enbl)         0 */

    /* Nothing to do ... keep default values */
    /* ------------------------------------------------------------------ */

    /* Set CTRL_REG4 (0x23)
    ====================================================================
    BIT  Symbol    Description                                   Default
    ---  ------    --------------------------------------------- -------
      7  BDU       Block Data Update (0=continuous, 1=LSB/MSB)         0
      6  BLE       Big/Little-Endian (0=Data LSB, 1=Data MSB)          0
    5-4  FS1/0     Full scale selection                               00
                                        00 = 250 dps
                                        01 = 500 dps
                                        10 = 2000 dps
                                        11 = 2000 dps
      0  SIM       SPI Mode (0=4-wire, 1=3-wire)                       0 */

    /* Adjust resolution if requested */
    write8(L3GD20_ADDRESS, GYRO_REGISTER_CTRL_REG4, 0x00);

    /* Set CTRL_REG5 (0x24)
    ====================================================================
    BIT  Symbol    Description                                   Default
    ---  ------    --------------------------------------------- -------
      7  BOOT      Reboot memory content (0=normal, 1=reboot)          0
      6  FIFO_EN   FIFO enable (0=FIFO disable, 1=enable)              0
      4  HPen      High-pass filter enable (0=disable,1=enable)        0
    3-2  INT1_SEL  INT1 Selection config                              00
    1-0  OUT_SEL   Out selection config                               00 */

    /* Nothing to do ... keep default values */
    /* ------------------------------------------------------------------ */

 }

void l3gd20Read(float *x, float *y, float *z)
{
    // Receive Variable
    unsigned char data[6];
    short xx, yy, zz;

    // Start Conditinon
    StartI2C1();
    IdleI2C1();
    // Write Address
    MasterWriteI2C1(L3GD20_ADDRESS);
    IdleI2C1();
    // Write Register
    MasterWriteI2C1(GYRO_REGISTER_OUT_X_L | 0x80);
    IdleI2C1();
    // Read Data
    RestartI2C1();
    IdleI2C1();
    MasterWriteI2C1(L3GD20_ADDRESS | 1);   //kxp84 Read mode 0x30+1
    IdleI2C1();
    MastergetsI2C1(6, data, 1000);
    IdleI2C1();
    // Ack
    NotAckI2C1();
    // Stop Condition
    StopI2C1();
    IdleI2C1();

    //
    xx = (short)(data[0] | (data[1] << 8));
    yy = (short)(data[2] | (data[3] << 8));
    zz = (short)(data[4] | (data[5] << 8));

    //
    *x = (float)xx * (GYRO_SENSITIVITY_250DPS * (0.017453293F));
    *y = (float)yy * (GYRO_SENSITIVITY_250DPS * (0.017453293F));
    *z = (float)zz * (GYRO_SENSITIVITY_250DPS * (0.017453293F));
}

#endif

#if USE_LSM303
void lsm303dlhcAccelInit(void)
{
    // Enable the accelerometer (100Hz)
    write8(LSM303_ADDRESS_ACCEL, LSM303_REGISTER_ACCEL_CTRL_REG1_A, 0x57);

    // LSM303DLHC has no WHOAMI register so read CTRL_REG1_A back to check
    // if we are connected or not
    // uint8_t reg1_a = read8(LSM303_ADDRESS_ACCEL, LSM303_REGISTER_ACCEL_CTRL_REG1_A);
    // if (reg1_a != 0x57)
    // {
    //     return false;
    // }
}

void lsm303dlhcAccelRead(float *x, float *y, float *z)
{
    unsigned char data[6];
    short xx, yy, zz;

    // Start Conditinon
    StartI2C1();
    IdleI2C1();
    // Write Address
    MasterWriteI2C1(LSM303_ADDRESS_ACCEL);
    IdleI2C1();
    // Write Register
    MasterWriteI2C1(LSM303_REGISTER_ACCEL_OUT_X_L_A | 0x80);
    IdleI2C1();
    // Get Data
    RestartI2C1();
    IdleI2C1();
    MasterWriteI2C1(LSM303_ADDRESS_ACCEL | 1);   //kxp84 Read mode 0x30+1
    IdleI2C1();
    MastergetsI2C1(6, data, 1000);
    IdleI2C1();
    // Ack
    NotAckI2C1();
    // Stop Condition
    StopI2C1();
    IdleI2C1();

    xx = (short)((data[0] | (data[1] << 8)) >> 4);
    yy = (short)((data[2] | (data[3] << 8)) >> 4);
    zz = (short)((data[4] | (data[5] << 8)) >> 4);

    if((xx & 0x800) == 0)
        *x = (int)xx * _lsm303Accel_MG_LSB * (9.80665F);
    else
        *x = (-1) * (((~xx) & 0x000fff) + 1) * _lsm303Accel_MG_LSB * (9.80665F);

    if((yy & 0x800) == 0)
        *y = (int)yy * _lsm303Accel_MG_LSB * (9.80665F);
    else
        *y = (-1) * (((~yy) & 0x000fff) + 1) * _lsm303Accel_MG_LSB * (9.80665F);

    if((zz & 0x800) == 0)
        *z = (int)zz * _lsm303Accel_MG_LSB * (9.80665F);
    else
        *z = (-1) * (((~zz) & 0x000fff) + 1) * _lsm303Accel_MG_LSB * (9.80665F);

}


void lsm303dlhcMagInit(void)
{
    // Enable the magnetometer
    write8(LSM303_ADDRESS_MAG, (unsigned char)LSM303_REGISTER_MAG_MR_REG_M, (unsigned char)0x00);

    // LSM303DLHC has no WHOAMI register so read CRA_REG_M to check
    // the default value (0b00010000/0x10)
    // unsigned char reg1_a = read8(LSM303_ADDRESS_MAG, LSM303_REGISTER_MAG_CRA_REG_M);
    // if (reg1_a != 0x10)
    // {
    //	 // Error Message
    // }

    // Set the magnetometer's gain
    setMagGain((unsigned char)LSM303_MAGGAIN_1_3);

}


void setMagGain(unsigned char gain)
{
    write8(LSM303_ADDRESS_MAG, (unsigned char)LSM303_REGISTER_MAG_CRB_REG_M, gain);

    magGain = gain;
    switch(gain)
    {
        case LSM303_MAGGAIN_1_3:
            lsm303Mag_Gauss_LSB_XY = 1100;
            lsm303Mag_Gauss_LSB_Z  = 980;
            break;
        case LSM303_MAGGAIN_1_9:
            lsm303Mag_Gauss_LSB_XY = 855;
            lsm303Mag_Gauss_LSB_Z  = 760;
            break;
        case LSM303_MAGGAIN_2_5:
            lsm303Mag_Gauss_LSB_XY = 670;
            lsm303Mag_Gauss_LSB_Z  = 600;
            break;
        case LSM303_MAGGAIN_4_0:
            lsm303Mag_Gauss_LSB_XY = 450;
            lsm303Mag_Gauss_LSB_Z  = 400;
            break;
        case LSM303_MAGGAIN_4_7:
            lsm303Mag_Gauss_LSB_XY = 400;
            lsm303Mag_Gauss_LSB_Z  = 355;
            break;
        case LSM303_MAGGAIN_5_6:
            lsm303Mag_Gauss_LSB_XY = 330;
            lsm303Mag_Gauss_LSB_Z  = 295;
            break;
        case LSM303_MAGGAIN_8_1:
            lsm303Mag_Gauss_LSB_XY = 230;
            lsm303Mag_Gauss_LSB_Z  = 205;
            break;
    }
}


void lsm303dlhcMagRead(float *x, float *y, float *z)
{
    unsigned char data[6];
    short xx, yy, zz;

    // Start Conditinon
    StartI2C1();
    IdleI2C1();
    // Write Address
    MasterWriteI2C1(LSM303_ADDRESS_MAG);
    IdleI2C1();
    // Write Register
    MasterWriteI2C1(LSM303_REGISTER_MAG_OUT_X_H_M);
    IdleI2C1();
    // Get Data
    RestartI2C1();
    IdleI2C1();
    MasterWriteI2C1(LSM303_ADDRESS_MAG | 1);   //kxp84 Read mode 0x30+1
    IdleI2C1();
    MastergetsI2C1(6, data, 1000);
    IdleI2C1();
    // Ack
    NotAckI2C1();
    // Stop Condition
    StopI2C1();
    IdleI2C1();

    xx = (int)(data[1] | (int)(data[0] << 8));
    yy = (int)(data[5] | (int)(data[4] << 8));
    zz = (int)(data[3] | (int)(data[2] << 8));

    /* Check if the sensor is saturating or not */
    if ( (xx >= 2040) | (xx <= -2040) |
         (yy >= 2040) | (yy <= -2040) |
         (zz >= 2040) | (zz <= -2040) )
    {
        /* Saturating .... increase the range if we can */
        switch(magGain)
        {
            case LSM303_MAGGAIN_5_6:
                setMagGain(LSM303_MAGGAIN_8_1);
                break;
            case LSM303_MAGGAIN_4_7:
                setMagGain(LSM303_MAGGAIN_5_6);
                break;
            case LSM303_MAGGAIN_4_0:
                setMagGain(LSM303_MAGGAIN_4_7);
                break;
            case LSM303_MAGGAIN_2_5:
                setMagGain(LSM303_MAGGAIN_4_0);
                break;
            case LSM303_MAGGAIN_1_9:
                setMagGain(LSM303_MAGGAIN_2_5);
                break;
            case LSM303_MAGGAIN_1_3:
                setMagGain(LSM303_MAGGAIN_1_9);
                break;
            default:
                break;
        }
    }

    // return data
    *x = (float)xx /  (float)(lsm303Mag_Gauss_LSB_XY) * 100.0;
    *y = (float)yy /  (float)(lsm303Mag_Gauss_LSB_XY) * 100.0;
    *z = (float)zz /  (float)(lsm303Mag_Gauss_LSB_Z)  * 100.0;

 }

float getAngle(void)
{
    float x, y, z ;
    lsm303dlhcMagRead(&x, &y, &z);
    float buf = (atan2(y, x) * 180.0) / Pi;

    if(buf < 0)
        buf += 360.0;

    return buf;
}

float getfusionAngle(void)
{

    float accelX,accelY,accelZ;
    float magX, magY, magZ;
    float roll, pitch, angle;

    lsm303dlhcAccelRead(&accelX, &accelY, &accelZ);
    lsm303dlhcMagRead(&magX, &magY, &magZ);

    roll = (float)atan2(accelY, accelZ);

    float buf = accelY * sin(roll) + accelZ * cos(roll);
    // printf("Buf : %f", buf);
    if (buf < 1.0 || buf > -1.0)
        pitch = accelX > 0 ? (Pi / 2) : (-Pi / 2);
    else
        pitch = (float)atan(-accelX / (buf));

    angle = ((float)atan2(magZ * sin(roll) - magZ * cos(roll), magX * cos(pitch) + magY * sin(pitch) * sin(roll) + magZ * sin(pitch) * cos(roll))) * 180.0 / Pi;

    if(angle < 0)
        angle += 360.0;

    return angle;
}

#endif

#if USE_BMP085
void read8(unsigned char reg, unsigned char *value)
{
    unsigned char data;

    // Start Conditinon
    StartI2C1();
    IdleI2C1();
    // Write Address
    MasterWriteI2C1(BMP085_ADDRESS);
    IdleI2C1();
    // Write Register
    MasterWriteI2C1(reg);
    IdleI2C1();
    // Get Data
    RestartI2C1();
    IdleI2C1();
    MasterWriteI2C1(BMP085_ADDRESS | 1);   //kxp84 Read mode 0x30+1
    IdleI2C1();
    data = MasterReadI2C1();
    IdleI2C1();
    // Ack
    NotAckI2C1();
    // Stop Condition
    StopI2C1();
    IdleI2C1();

    *value = data;

}


void read16(unsigned char reg, unsigned short *value)
{
    unsigned char data[2];

    // Start Conditinon
    StartI2C1();
    IdleI2C1();
    // Write Address
    MasterWriteI2C1(BMP085_ADDRESS);
    IdleI2C1();
    // Write Register
    MasterWriteI2C1(reg);
    IdleI2C1();
    // Get Data
    RestartI2C1();
    IdleI2C1();
    MasterWriteI2C1(BMP085_ADDRESS | 1);   //kxp84 Read mode 0x30+1
    IdleI2C1();
    MastergetsI2C1(2, data, 1000);
    IdleI2C1();
    // Ack
    NotAckI2C1();
    // Stop Condition
    StopI2C1();
    IdleI2C1();

    *value = (unsigned short)((data[0] << 8) | data[1]);

}


void readS16(unsigned char reg, short *value)
{
    unsigned short buffer;
    read16(reg, &buffer);

    *value = (short)buffer;
}


void readCoefficients(void)
{
    readS16(BMP085_REGISTER_CAL_AC1, &ac1);
    readS16(BMP085_REGISTER_CAL_AC2, &ac2);
    readS16(BMP085_REGISTER_CAL_AC3, &ac3);
    read16(BMP085_REGISTER_CAL_AC4, &ac4);
    read16(BMP085_REGISTER_CAL_AC5, &ac5);
    read16(BMP085_REGISTER_CAL_AC6, &ac6);
    readS16(BMP085_REGISTER_CAL_B1, &b1);
    readS16(BMP085_REGISTER_CAL_B2, &b2);
    readS16(BMP085_REGISTER_CAL_MB, &mb);
    readS16(BMP085_REGISTER_CAL_MC, &mc);
    readS16(BMP085_REGISTER_CAL_MD, &md);

}

int computeB5(int ut)
{
    int X1 = (ut - (int)ac6) * ((int)ac5) >> 15;
    int X2 = ((int)mc << 11) / (X1+(int)md);
    return X1 + X2;
}

void readRawTemperature(int *temperature)
{
    unsigned short t;

    write8(BMP085_ADDRESS, BMP085_REGISTER_CONTROL, BMP085_REGISTER_READTEMPCMD);
    delay_ms(5);
    read16(BMP085_REGISTER_TEMPDATA, &t);
    *temperature = t;

}

void readRawPressure(int *pressure)
{
    unsigned char p8;
    unsigned short p16;
    int  p32;

    write8(BMP085_ADDRESS, BMP085_REGISTER_CONTROL, BMP085_REGISTER_READPRESSURECMD + (_bmp085Mode << 6));
    switch(_bmp085Mode)
    {
        case BMP085_MODE_ULTRALOWPOWER:
            delay_ms(5);
            break;
        case BMP085_MODE_STANDARD:
            delay_ms(8);
            break;
        case BMP085_MODE_HIGHRES:
            delay_ms(14);
            break;
        case BMP085_MODE_ULTRAHIGHRES:
        default:
            delay_ms(26);
            break;
    }

    read16(BMP085_REGISTER_PRESSUREDATA, &p16);
    p32 = (unsigned int)p16 << 8;
    read8(BMP085_REGISTER_PRESSUREDATA+2, &p8);
    p32 += p8;
    p32 >>= (8 - _bmp085Mode);

    *pressure = p32;

}

void getTemperature(float *temp)
{
    int UT, B5;     // following ds convention
    float t;

    readRawTemperature(&UT);
    B5 = computeB5(UT);
    t = (float)((B5+8) >> 4);
    t /= 10.0;

    *temp = t;
}

void getPressure(float *pressure)
{
    int  ut = 0, up = 0, compp = 0;
    int  x1, x2, b5, b6, x3, b3, p;
    unsigned int b4, b7;

    /* Get the raw pressure and temperature values */
    readRawTemperature(&ut);
    readRawPressure(&up);

    /* Temperature compensation */
    b5 = computeB5(ut);

    /* Pressure compensation */
    b6 = b5 - 4000;
    x1 = (b2 * ((b6 * b6) >> 12)) >> 11;
    x2 = (ac2 * b6) >> 11;
    x3 = x1 + x2;
    b3 = (((((int)ac1) * 4 + x3) << _bmp085Mode) + 2) >> 2;
    x1 = (ac3 * b6) >> 13;
    x2 = (b1 * ((b6 * b6) >> 12)) >> 16;
    x3 = ((x1 + x2) + 2) >> 2;
    b4 = (ac4 * (unsigned int)(x3 + 32768)) >> 15;
    b7 = ((unsigned int)(up - b3) * (50000 >> _bmp085Mode));

    if (b7 < 0x80000000)
        p = (b7 << 1) / b4;
    else
        p = (b7 / b4) << 1;

    x1 = (p >> 8) * (p >> 8);
    x1 = (x1 * 3038) >> 16;
    x2 = (-7357 * p) >> 16;
    compp = p + ((x1 + x2 + 3791) >> 4);

    /* Assign compensated pressure value */
    *pressure = compp;

}

float getAltitude(float *pressure, float *temp)
{

    return 44330.0 * (1 - powf((*pressure)/1013.25/100.0, 1/5.255));

}
#endif


void AutoControl(float * lati, float *longi)
{
    // wait a minute
    char auto_flag = 0;
    float x, y, z, angle, operate;

    PORTBbits.RB3 = 1;
    delay_ms(100);

    // get angle offset
    mCNDClose();
    //mCNDOpen(CND_ON | CND_IDLE_CON, CND3_ENABLE | CND4_ENABLE, 0);
    // Clear mismatch condition
    //mPORTDRead();
    //ConfigIntCND(CHANGE_INT_ON | CHANGE_INT_PRI_6);
    // pw[2] = 1300 * 40;
    pw[2] = 1420 * 40;

    float angle_offset = getAngle();

    // 1m????????
    // 0.000008983152841 [?] 0.000011095029098 [?]
    float latitude_per_meter = 1.0 / (R * Pi / 180.0);
    float longitude_per_meter = 1.0 / (R * Pi / 180.0 * cos(*lati * Pi/180));


    // GPS dist
    float latitude_dist, longitude_dist, diff_lati, diff_longi;
    latitude_dist = *lati + DISTANCE * latitude_per_meter * cos( angle_offset * Pi/180);
    longitude_dist = *longi + DISTANCE * longitude_per_meter * sin(angle_offset * Pi/180);


    // ??????????????????
    diff_lati = (latitude_dist - *lati) / latitude_per_meter;
    diff_longi = (longitude_dist - *longi) / longitude_per_meter;

    float distance = (float)sqrt((float)(diff_longi*diff_longi + diff_lati*diff_lati));

    printf("\r\n\r\nOFF_SET : %f %f %f %f\r\n\t\n", angle_offset, latitude_dist, longitude_dist, distance);

    float latitude, longitude;

    int k = 0;

    while(distance > 5.0)
    {
        auto_flag = 1;
        if(!(pw[3] > 1720*40))
        {
            auto_flag = 0;
            break;
        }
        delay_ms(10);
        k++;
        if(k > 3000)
        {
            auto_flag = 1;
            break;
        }
        // mag??
        angle = getAngle();
        // ??????
        operate = angle - angle_offset;
        // ????
        if(operate > 180)
            operate -= 360.0;
        else if(operate < -180)
            operate += 360.0;

        // ????
        pw[1] = (1520 - ((int)operate)*9) * 40;
        pw[2] = 1420 * 40;

        if(gps_flag)
            getGPS(&latitude, &longitude);

        // ??????????????????
        diff_lati = (latitude_dist - latitude) / latitude_per_meter;
        diff_longi = (longitude_dist - longitude) / longitude_per_meter;

        // ??????????
        distance = (float)sqrt((float)(diff_longi*diff_longi + diff_lati*diff_lati));
        printf("%f %f %u %u\r\n", angle, distance, pw[1] / 40, pw[2] /40);
    }

    if(auto_flag == 1)
    {
        pw[2] = 1520 * 40;
        PORTBbits.RB3 = 0;
        while(1);
    }

    PWMInit();


}