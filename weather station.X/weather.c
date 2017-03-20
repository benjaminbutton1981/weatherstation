//____________________________________________________________________________________________________________________

//		Weather Station
// This board interfaces with XBEE (rs232), temp-hum sensor (SPI), rain guage (adc), widvel (digital ip), dir sensor (adc)
//____________________________________________________________________________________________________________________

// Device in use PIC 18f4480
// Crystal Speed = 10 MHz
// Program memory 0x0000 to 0x2000 (8K)
//____________________________________________________________________________________________________________________

//--------------------------------------------------------------------------------------------------------------------
//	Include files

#include "p18f4480.h"
#include <string.h>
//#include <sw_spi.h>
//#include <stdio.h>
//#include <stdlib.h>
//#include <math.h>
//--------------------------------------------------------------------------------------------------------------------


//--------------------------------------------------------------------------------------------------------------------
// PIC18F4480 Configuration Bit Settings
//--------------------------------------------------------------------------------------------------------------------

// CONFIG1H
#pragma config OSC = HS         // Oscillator Selection bits (HS oscillator)
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enable bit (Fail-Safe Clock Monitor disabled)
#pragma config IESO = OFF       // Internal/External Oscillator Switchover bit (Oscillator Switchover mode disabled)

// CONFIG2L
#pragma config PWRT = OFF       // Power-up Timer Enable bit (PWRT disabled)
#pragma config BOREN = OFF      // Brown-out Reset Enable bits (Brown-out Reset disabled in hardware and software)
#pragma config BORV = 3         // Brown-out Reset Voltage bits (VBOR set to 2.1V)

// CONFIG2H
#pragma config WDT = OFF        // Watchdog Timer Enable bit (WDT disabled (control is placed on the SWDTEN bit))
#pragma config WDTPS = 32768    // Watchdog Timer Postscale Select bits (1:32768)

// CONFIG3H
#pragma config PBADEN = OFF     // PORTB A/D Enable bit (PORTB<4:0> pins are configured as digital I/O on Reset)
#pragma config LPT1OSC = OFF    // Low-Power Timer 1 Oscillator Enable bit (Timer1 configured for higher power operation)
#pragma config MCLRE = ON       // MCLR Pin Enable bit (MCLR pin enabled; RE3 input pin disabled)

// CONFIG4L
#pragma config STVREN = ON      // Stack Full/Underflow Reset Enable bit (Stack full/underflow will cause Reset)
#pragma config LVP = OFF        // Single-Supply ICSP Enable bit (Single-Supply ICSP disabled)
#pragma config BBSIZ = 1024     // Boot Block Size Select bit (1K words (2K bytes) boot block)
#pragma config XINST = OFF      // Extended Instruction Set Enable bit (Instruction set extension and Indexed Addressing mode disabled (Legacy mode))

// CONFIG5L
#pragma config CP0 = OFF        // Code Protection bit (Block 0 (000800-001FFFh) not code-protected)
#pragma config CP1 = OFF        // Code Protection bit (Block 1 (002000-003FFFh) not code-protected)

// CONFIG5H
#pragma config CPB = OFF        // Boot Block Code Protection bit (Boot block (000000-0007FFh) not code-protected)
#pragma config CPD = OFF        // Data EEPROM Code Protection bit (Data EEPROM not code-protected)

// CONFIG6L
#pragma config WRT0 = OFF       // Write Protection bit (Block 0 (000800-001FFFh) not write-protected)
#pragma config WRT1 = OFF       // Write Protection bit (Block 1 (002000-003FFFh) not write-protected)

// CONFIG6H
#pragma config WRTC = OFF       // Configuration Register Write Protection bit (Configuration registers (300000-3000FFh) not write-protected)
#pragma config WRTB = OFF       // Boot Block Write Protection bit (Boot block (000000-0007FFh) not write-protected)
#pragma config WRTD = OFF       // Data EEPROM Write Protection bit (Data EEPROM not write-protected)

// CONFIG7L
#pragma config EBTR0 = OFF      // Table Read Protection bit (Block 0 (000800-001FFFh) not protected from table reads executed in other blocks)
#pragma config EBTR1 = OFF      // Table Read Protection bit (Block 1 (002000-003FFFh) not protected from table reads executed in other blocks)

// CONFIG7H
#pragma config EBTRB = OFF      // Boot Block Table Read Protection bit (Boot block (000000-0007FFh) not protected from table reads executed in other blocks)

//--------------------------------------------------------------------------------------------------------------------
									//	Definitions
//--------------------------------------------------------------------------------------------------------------------
//ASCII for number to character
#define ascii 48

//--------I2C DSTH01 Chip---------
//register address inside chip
#define register_0 0x00     //status
#define register_1 0x01     //dataH
#define register_2 0x02     //dataL
#define register_3 0x03     //config
#define register_11 0x11
//slave address defined in the chip
#define slave_add 0x80
//data
#define hum_start   0x01
#define temp_start  0x11

//--------------------------------------------------------------------------------------------------------------------
//LCD connectors
//--------------------------------------------------------------------------------------------------------------------
#define LCD_RW PORTBbits.RB4
#define LCD_RS PORTBbits.RB3
#define LCD_EN PORTCbits.RC1
#define LCD_DB0_7 PORTD

#define clear_lcd 0x01
#define return_cur 0x03
#define line2 0xC0
#define D0_C0_B0 0x08
#define D0_C0_B1 0x09
#define D0_C1_B0 0x0A
#define D0_C1_B1 0x0B
#define D1_C0_B0 0x0C
#define D1_C0_B1 0x0D
#define D1_C1_B0 0x0E
#define D1_C1_B1 0x0F


//--------------------------------------------------------------------------------------------------------------------


//--------------------------------------------------------------------------------------------------------------------
									//	GLOBAL VARIABLES
//--------------------------------------------------------------------------------------------------------------------
unsigned char inte_sample_counter=0;
unsigned char inte_data_counter = 0;

unsigned char dsth01_status         ;   //conversion status

unsigned char tempL                 ;   //temperature L
unsigned char tempH                 ;   //temperature H
int temp_dec                        ;   //temperature decimal value as received
unsigned char humL                  ;   //humidity L
unsigned char humH                  ;   //humidity H
int hum_dec                         ;   //humidity decimal value as received

char temp_data                      ;   //temperature data
char temp_buffer[25], temp_buffer_instant                ;   //temperature buffer
unsigned char hum_data              ;   //humidity data
unsigned char hum_buffer[25], hum_buffer_instant        ;   //humidity buffer
unsigned int wind_dir_data         ;   //wind dir data
float wind_dir_voltage              ;   //wind direction in voltage
unsigned int wind_dir_dec           ;   //wind direction decimal value as received
unsigned int wind_dir_buffer[25], wind_dir_instant   ;   //wind dir buffer
char wind_dirH = ' ';
char wind_dirL = ' ';
unsigned char lcd_string[32] ;
unsigned char *ptr_lcd_string;
unsigned char wind_vel              ;   //wind vel data
unsigned int wind_count             ;   //wind buffer
unsigned char rain_                 ;   //rain data
unsigned int rain_count             ;   //rain buffer


//---------------------------------------------------------------------------------------------------------------------



//--------------------------------------------------------------------------------------------------------------------
									//	Function Prototypes
//--------------------------------------------------------------------------------------------------------------------
void main(void) ;
void init(void);
void rs232_init(void) ;
void i2c_init(void);
void setint(void) ;
void bin_io(void);
void dsth01_out_init(void);
void read_dsth01(void) ;
void delay_ms(int);
void delay_us(int);
char i2c_write(char i2c_add, char i2c_data) ;
char i2c_read(char i2c_add, char *i2c_data) ;
void init_lcd(void);
void write_char_lcd(unsigned char);
void write_str_lcd(unsigned char*);
void write_com_lcd(unsigned char);
//--------------------------------------------------------------------------------------------------------------------


//--------------------------------------------------------------------------------------------------------------------
									// Main Program
//--------------------------------------------------------------------------------------------------------------------
void main()
{
    unsigned char c =0;
    init();
    init_lcd();
    rs232_init() ;
    i2c_init();
    setint() ;
    //dsth01_out_init();

    //no cursor
    write_com_lcd(D1_C0_B0);
    //initialize string
    strcpy(lcd_string,"No Data");
    //write string to lcd
    write_str_lcd(&lcd_string);
    
    PORTCbits.RC2       = 0 ;    //DSTH01 chip enabled bar(CS) active low

    SSPCON1bits.SSPEN   = 1 ;    // Enables the serail port

    ADCON0bits.ADON     = 1 ;    //starting ADC module
    T0CONbits.TMR0ON    = 1 ;    //starting timer for sampling
    T1CONbits.TMR1ON    = 1 ;    //starting timer for serial send

    while(1)
    {

    }
}

void delay_ms(int t_delay)
{
    int c;
    for(c=0;c<t_delay*40;c++);
}

void delay_us(int t_delay)
{
    int c;
    t_delay = t_delay/10;
    for(c=0;c<t_delay;c++);
}

void write_char_lcd(unsigned char c)
{
    LCD_RW = 0 ;
    LCD_RS = 1 ;
    delay_us(1);
    LCD_DB0_7 = c;
    LCD_EN = 1 ;
    delay_us(1);
    LCD_EN = 0 ;
    delay_us(40);
}

void write_str_lcd(unsigned char *ptr_char)
{
    char count_char;
    for(count_char=0;count_char<32;count_char++)
    {
        if(*(ptr_char+count_char)=='\0')
            break;
        LCD_RW = 0 ;
        LCD_RS = 1 ;
        delay_us(1);
        LCD_DB0_7 = *(ptr_char+count_char);
        LCD_EN = 1 ;
        delay_us(1);
        LCD_EN = 0 ;
        delay_us(40);
    }
}
void write_com_lcd(unsigned char c)
{
    LCD_RW = 0 ;
    LCD_RS = 0 ;
    delay_us(1);
    LCD_DB0_7 = c;
    LCD_EN = 1 ;
    delay_us(1);
    LCD_EN = 0 ;
    if(c==clear_lcd || c == return_cur)
        delay_ms(1);
    else
        delay_us(40);
}

void init_lcd()
{
    ptr_lcd_string = lcd_string ;

    //schematic
    /*
     * 1 - GND
     * 2 - VCC
     * 3 - 10 k pot center; VCC and GND two ends
     * 4 - RS  - RB3
     * 5 - R/W - RB4
     * 6 - EN RC1
     * 7 - 14 ; 8 bit 7-DB0 and 14 - DB7
     * 15 - 560 ohm to 5V, pull up
     * 16 - GND
     */


    //Initialization sequence for LCD
    //wait 1
   delay_ms(20);

    LCD_RW = 0 ;
    LCD_RS = 0 ;
    delay_us(1);
    LCD_DB0_7 = 0x30 ;
    LCD_EN = 1 ;
    delay_us(1);
    LCD_EN = 0 ;
    delay_ms(10);

    LCD_RW = 0 ;
    LCD_RS = 0 ;
    delay_us(1);
    LCD_DB0_7 = 0x30 ;
    LCD_EN = 1 ;
    delay_us(1);
    LCD_EN = 0 ;
    delay_us(100);

    LCD_RW = 0 ;
    LCD_RS = 0 ;
    delay_us(1);
    LCD_DB0_7 = 0x30 ;
    LCD_EN = 1 ;
    delay_us(1);
    LCD_EN = 0 ;
    delay_ms(10);

    LCD_DB0_7 = 0x38;
    LCD_EN = 1 ;
    delay_us(1);
    LCD_EN = 0 ;
    delay_us(40);

    LCD_DB0_7 = 0x08;
    LCD_EN = 1 ;
    delay_us(1);
    LCD_EN = 0 ;
    delay_us(40);

    LCD_DB0_7 = 0x01;
    LCD_EN = 1 ;
    delay_us(1);
    LCD_EN = 0 ;
    delay_ms(10);

    LCD_DB0_7 = 0x06;
    LCD_EN = 1 ;
    delay_us(1);
    LCD_EN = 0 ;
    delay_us(40);

    LCD_DB0_7 = 0x0e;
    LCD_EN = 1 ;
    delay_us(1);
    LCD_EN = 0 ;
    delay_us(40);

}

void init()
{

    char temporary_i ;          //a temporary variable

    //user data initialization
    tempL           = 0 ;   //temperature L
    tempH           = 0 ;   //temperature H
    humL            = 0 ;   //humidity L
    humH            = 0 ;   //humidity H
    temp_data       = 0 ;   //temperature data
    hum_data        = 0 ;   //humidity data
    wind_dir_data   = 0 ;   //wind dir data
    wind_vel        = 0 ;   //wind vel data
    wind_count      = 0 ;   //wind buffer
    rain_           = 0 ;   //rain data
    rain_count      = 0 ;   //rain buffer
    
    for(temporary_i=0;temporary_i<25;temporary_i++)
    {
        temp_buffer[temporary_i]    = 0 ;   //temperature buffer
        hum_buffer[temporary_i]     = 0 ;   //humidity buffer
        wind_dir_buffer[temporary_i]= 0 ;   //wind dir buffer
    }

    //PORT C
   //RC3 is SCL and RC4 is SDA pins of I2C while RC6 is TX and RC7 is RX pins of UART
   //RC2 is used for outdoor DSTH01 chip select
    TRISCbits.RC2 = 0 ;
    PORTCbits.RC2 = 1 ;     //Chip disabled initially, bar(CS) active low

    //LCD display
    TRISCbits.RC1 = 0 ;
    PORTCbits.RC1 = 0 ;     //Enable LCD; 1 = enable

    // PORT B
    TRISBbits.RB0 = 1 ;         //Interrupt pin for wind speed
    TRISBbits.RB1 = 1 ;         //Interrupt pin for rain guage
    //Interrupt for above enabled in setint function

    //LCD display
    TRISBbits.RB3 = 0 ;
    TRISBbits.RB4 = 0 ;     //R LCD; 0=instruction register, 1= data register
    PORTBbits.RB3 = 0 ;
    PORTBbits.RB4 = 0 ;     //R/_bar_W LCD ; 0= write, 1=read

    //PORTD
    TRISD = 0 ;
    PORTD = 0 ;             //Data LCD

    
    // ADC for wind direction
    // ADC1 is used, pin no 3, RA1
    TRISAbits.RA1 = 1 ;     //Analog pin to be set as input

    //ADCON0: A/D CONTROL REGISTER 0
    //? ? CHS3 CHS2 CHS1 CHS0 GO/DONE ADON
    ADCON0bits.CHS3         = 0 ;   //CHS3:CHS0: Analog Channel Select bits
    ADCON0bits.CHS2         = 0 ;   //0001 = Channel 1 (AN1)
    ADCON0bits.CHS1         = 0 ;
    ADCON0bits.CHS0         = 1 ;
    ADCON0bits.GO_NOT_DONE  = 0 ;   //GO/DONE: A/D Conversion Status bit, 1 = A/D conversion in progress, 0 means idle
    ADCON0bits.ADON         = 0 ;   //ADON: A/D On bit, 1 = A/D converter module is enabled
    
    //ADCON1: A/D CONTROL REGISTER 1
    //? ? VCFG1 VCFG0 PCFG3 PCFG2 PCFG1 PCFG0
    ADCON1bits.VCFG1    = 0 ;       //Voltage Reference Configuration bit, 1 = VREF- (AN2), 0 = AVSS
    ADCON1bits.VCFG0    = 0 ;       //Voltage Reference Configuration bit, 1 = VREF+ (AN3), 0 = AVDD
    ADCON1bits.PCFG3    = 1 ;       // PCFG3:PCFG0: A/D Port Configuration Control bits
    ADCON1bits.PCFG2    = 1 ;       // 1111 all digital, 1110 AN0 analog, 1101 AN0, AN1 analog
    ADCON1bits.PCFG1    = 0 ;       // 0000 all analog
    ADCON1bits.PCFG0    = 1 ;

    //ADCON2: A/D CONTROL REGISTER 2
    //ADFM ? ACQT2 ACQT1 ACQT0 ADCS2 ADCS1 ADCS0
    ADCON2bits.ADFM     = 0 ;       // ADFM: A/D Result Format Select bit, 1 = Right justified, 0 = Left justified
    ADCON2bits.ACQT2    = 0 ;       // ACQT2:ACQT0: A/D Acquisition Time Select bits
    ADCON2bits.ACQT1    = 0 ;       // 111 = 20 TAD
    ADCON2bits.ACQT0    = 0 ;
    ADCON2bits.ADCS2    = 1 ;       // ADCS2:ADCS0: A/D Conversion Clock Select bits
    ADCON2bits.ADCS1    = 1 ;       // 110 = FOSC/64
    ADCON2bits.ADCS0    = 0 ;
    //TAD is adc conversion time per bit
    //Minimum 2 TAD wait is required between each conversion
    //------------------------------------------------------------
    //----------A/D Acquisition Requirements--------------------
    /*Source impedance Rs and sampling switch impedance Rss (depends on Vdd) affect capacitor CHold charge time.
     * CHold = 120 pf, Rs = 2.5 kohm max recommended, Conv error <= 1/2 lsb, Rss = 7 kohm for Vdd = 5 V, temp = 50 degC system max
     * Acquisition time = Amp settling time + CHold charge time + Temp coeff.
     * Tacq = Tamp + Tc + Tcoff
     * Tamp = 5?s, Tcoff = 1.25 ?s, Tc = -(CHOLD)(RIC + RSS + RS) ln(1/2047) ?s = -(120 pF) (1 k? + 7 k? + 2.5 k?) ln(0.0004883) ?s =9.61 ?s
     * TACQ = 5 ?s + 1.25 ?s + 9.61 ?s = 12.86 ?s
     */
    //------------------------------------------------------------
    //----------start conversion--------------------
    /* When the GO/DONE bit is set, sampling is stopped and a conversion begins.
     * Ensure that the TACQ has passed between selecting the desired input channel and setting the GO/DONE bit.
     * In our case conversion is done every 400 ms so we can safely ignore this constraint. Thus, we set automatic acq time ACQT to 0.
     * Else automatic acq time needs to be set in multiple of TAD. After GO aciquition is done for xTAD. Then CHold is disconnected and conversion starts.
     * Conversion requires 11 TADs, 1 TAD after CHold is disconnected and 10 TADs for 10 bits.
     * 2 TADs wait time before next conversion starts. So in total time is Tacq+11TAD+2TAD.
     * Holding capacitor is connected after 11 TADs.
     */

    //TIMER0 CONTROL REGISTER
    //T0CONbits.TMR0ON = 0 ;	//Timer0 Off
    //T0CONbits.T016BIT = 0 ;	//1 = Timer0 is configured as an 8-bit timer/counter, 0 means 16 bit
    //T0CONbits.T0CS = 0 ;	//Internal instruction cycle clock
    //T0CONbits.T0SE = 0 ; 	//Increment on low-to-high transition on T0CKI pin (not use here)
    //T0CONbits.PSA =	0 ;	//0 = Timer0 prescaler is assigned. Timer0 clock input comes from prescaler output
    //T0CONbits.T0PS2 = 0 ;     //Prescale value 1:2 (options 1:2,4,8,16,32,64,128,256)
    //T0CONbits.T0PS1 = 1 ;
    //T0CONbits.T0PS0 = 1 ;
    T0CON = 0X03 ;
    TMR0 = 0xBDC ;
    TMR0 = 0xC2F7;
    //Timer0 interrupt set to (inverse of 2500000/16/62500) = 400 ms
    //OFFSET by 3036 = 65536-62500 i.e. 0xBDC, so that interrupt is raised at 400 ms
    //Timer0 is used for sampling the data every .4 s
    //sampling time is increased in multiple of .4 s by using additional user counter
    //ex. for 10 sec sampling time use counter value 10/0.4 = 25.

    //TIMER1 CONTROL REGISTER
    //T1CONbits.RD16 = 1 ;	//1 = Enables register read/write of TImer1 in one 16-bit operation
    //T1CONbits.T1RUN = 0 ;	//0 = System clock is derived from another source
    //T1CONbits.T1CKPS1 = 1 ;	//1:8 Prescale value (options 1:1, 1:2, 1:4, 1:8)
    //T1CONbits.T1CKPS0 = 1 ;
    //T1CONbits.T1OSCEN = 0 ; 	//Timer1 Oscillator Enable bit
    //T1CONbits.T1SYNC = 1; 	//Do not synchronize external clock input
    //T1CONbits.TMR1CS = 0 ;	//Internal clock (FOSC/4)
    //T1CONbits.TMR1ON = 0 ;	//Timer1 disabled
    T1CON = 0XB4 ;
    TMR1 = 0x0BDC ;
    //Timer1 interrupt set to (inverse of 2500000/8/62500) = 200 ms
    //OFFSET by 3036 = 65536-62500 i.e. 0xBDC, so that interrupt is raised at 200 ms
    //Timer1 is used for serial transmit every .2 s
    //transmit interval time is increased in multiple of .2 s by using additional user counter
    //ex. for 10 sec sampling time use counter value 10/0.2 = 50.

    //T2CON: TIMER2 CONTROL REGISTER
    //? T2OUTPS3 T2OUTPS2 T2OUTPS1 T2OUTPS0 TMR2ON T2CKPS1 T2CKPS0
    //bit 7 Unimplemented: Read as ?0?
    T2CONbits.T2OUTPS3 = 0 ;  //bit 6-3 T2OUTPS3:T2OUTPS0: Timer2 Output Postscale Select bits 0000 = 1:1 Postscale to 1111 = 1:16 Postscale
    T2CONbits.T2OUTPS2 = 0 ;
    T2CONbits.T2OUTPS1 = 0 ;
    T2CONbits.T2OUTPS0 = 0 ;
    T2CONbits.TMR2ON = 0 ;    //bit 2 TMR2ON: Timer2 On bit 1 = Timer2 is on
    T2CONbits.T2CKPS1 = 1 ;//bit 1-0 T2CKPS1:T2CKPS0: Timer2 Clock Prescale Select bits 00 = Prescaler is 1, 01 = Prescaler is 4, 1x = Prescaler is 16
    T2CONbits.T2CKPS0 = 0 ;

    TMR2 = 0 ;
    PR2 = 250 ;
}

void rs232_init()
{
    //  init_RS232 ;
    TRISCbits.TRISC7 = 1 ;
    TRISCbits.TRISC6 = 0 ;

    TXSTA = 0b10100100 ;
    RCSTA = 0b10010000 ;
    BAUDCON = 0b00000000 ;
    //SPBRG = 64 ;    //9600
    SPBRG = 31 ;  //19200
}

void setint()
{
    //INTERRUPT CONTROL REGISTER 1
    //INTCON (7:GIE/GIEH 6:PEIE/GIEL 5:TMR0IE 4:INT0IE 3:RBIE 2:TMR0IF 1:INT0IF 0:RBIF)
    INTCONbits.GIE      = 1 ;   //1 = Enables all unmasked interrupts
    INTCONbits.PEIE     = 1 ;   //1 = Enables all unmasked peripheral interrupts
    INTCONbits.TMR0IE   = 1 ;   //1 = Enables the TMR0 overflow interrupt
    INTCONbits.INT0E    = 1 ;   //1 = Enables the INT0 external interrupt
    INTCONbits.RBIE     = 0 ;   //1 = Enables the RB port change interrupt
    INTCONbits.T0IF     = 0 ;   //1 = TMR0 register has overflowed (must be cleared in software)
    INTCONbits.INT0F    = 0 ;   //1 = The INT0 external interrupt occurred (must be cleared in software)
    INTCONbits.RBIF     = 0 ;   //1 = At least one of the RB7:RB4 pins changed state (must be cleared in software)

    //INTERRUPT CONTROL REGISTER 2
    //INTCON2 (7:RBPU 6:INTEDG0 5:INTEDG1 4:INTEDG2 3:— 2:TMR0IP 1:— 0:RBIP)
    INTCON2bits.RBPU    = 0 ;   //All PORTB pull-ups are disabled
    INTCON2bits.INTEDG0 = 0 ;   //Interrupt 0 on rising edge
    INTCON2bits.INTEDG1 = 0 ;   //Interrupt 1 on rising edge
    INTCON2bits.INTEDG2 = 0 ;   //Interrupt 2 on rising edge
    INTCON2bits.TMR0IP  = 1 ;   //TMR0 Low Priority
    INTCON2bits.RBIP    = 0 ;   //RB Port Change Interrupt Low Priority

    //INTERRUPT CONTROL REGISTER 3
    //INTCON2 (7:INT2IP 6:INT1IP 5:— 4:INT2IE 3:INT1IE 2:— 1:INT2IF 0:INT1IF)
    INTCON3bits.INT2IP  = 0 ;    //INT2 External Interrupt Priority Low
    INTCON3bits.INT1IP  = 1 ;    //INT1 External Interrupt Priority Low
    INTCON3bits.INT2IE  = 0 ;    //INT2 External Interrupt Enable
    INTCON3bits.INT1IE  = 1 ;    //INT1 External Interrupt Enable
    INTCON3bits.INT2IF  = 0 ;    //INT2 External Interrupt Flag bit
    INTCON3bits.INT1IF  = 0 ;    //INT1 External Interrupt Flag bit

    //PIR1 — 7:PSPIF 6:ADIF 5:RCIF 4:TXIF 3:SSPIF 2:CCP1IF 1:TMR2IF 0:TMR1IF
    PIR1	= 0b00000000 ;

    //PIR3 — — — 4:PTIF 3:IC3DRIF 2:IC2QEIF 1:IC1IF 0:TMR5IF
    //PIR3   &= 0X00 ;

    //PIE1 — 7:PSPIE 6:ADIE 5:RCIE 4:TXIE 3:SSPIE 2:CCP1IE 1:TMR2IE 0:TMR1IE
    PIE1 	= 0b00000001 ;

    //	PIE2 	= 0b00000000 ;

    //PIE3: PERIPHERAL INTERRUPT ENABLE REGISTER 3
    //7:IRXIE 6:WAKIE 5:ERRIE 4:TXB2IE 3:TXB1IE 2:TXB0IE 1:RXB1IE 0:RXB0IE
    PIE3 	= 0b00000000 ;

    //IPR1 — 7:PSPIP 6:ADIP 5:RCIP 4:TXIP 3:SSPIP 2:CCP1IP 1:TMR2IP 0:TMR1IP
    IPR1 	= 0b00000000 ;
    //	IPR2 	= 0b00000000 ;
    //IPR3: PERIPHERAL INTERRUPT PRIORITY REGISTER 3
    //IRXIP WAKIP ERRIP TXB2IP TXB1IP TXB0IP RXB1IP RXB0IP
    IPR3 	= 0b00000000 ;

    RCON 	= 0b10000000 ;
}

void i2c_init()
{
    TRISCbits.RC3 = 1 ;
    TRISCbits.RC4 = 1 ;
    //MSSP Status Register
    //SSPSTAT
    SSPSTATbits.SMP = 1; //slew rate control enabled for High-Speed mode (400 kHz)
    SSPSTATbits.CKE = 0;

    //MSSP Control Register 1
    //SSPCONN1
    //SSPCON1bits.WCOL
    //SSPCON1bits.SSPOV
    SSPCON1bits.SSPEN = 0;  // 1 Enables the serail port
    //SSPCON1bits.CKP
    SSPCON1bits.SSPM3 = 1;  //0110 configures I2C in Master mode
    SSPCON1bits.SSPM2 = 0;
    SSPCON1bits.SSPM1 = 0;
    SSPCON1bits.SSPM0 = 0;

    //MSSP Control Register 2
    //SSPCONN2
    //SSPCON2bits.GCEN =
    //SSPCON2bits.ACKSTAT
    //SSPCON2bits.ACKDT
    SSPCON2bits.ACKEN = 0;
    SSPCON2bits.RCEN = 0;
    SSPCON2bits.PEN = 0;
    SSPCON2bits.RSEN = 0;
    SSPCON2bits.SEN = 0;

    SSPADD = 0x06 ;     //for 10 MHz crystal and SCL = 400 kHz
}

void dsth01_out_init()
{
    //CONFIG

    //
}

char i2c_write(char i2c_add, char i2c_data)
{
     /*----Write operation----
     * StartBit--SlaveAddress--W--ACK--Address--ACK--Data--ACK--StopBit
     * 1) Master issues start bit (S) followed by slave address (0X40) and then W bit (0) indicating write operation
     * 2) Slave acknowledges (A) by pulling SDA line low for high duration of the ninth SCL cycle
     * 3) Master issues register address where to write the data
     * 4) Slave acknowledges (A)
     * 5) Master issues data to be written in the register
     * 6) Slave acknowledges (A)
     * 7) Master issues stop bit (P)
    */
    
    SSPCON2bits.RSEN    = 0 ;       //receive idle
    PIR1bits.SSPIF      = 0 ;       //reset flag

    //---1) start initiated
    SSPCON2bits.SEN     = 1 ;       //Start initiated (1)
    //timer used to exit infinite while loop in case communication hangs
    T2CONbits.TMR2ON = 0;
    PIR1bits.TMR2IF = 0;
    TMR2 = 0 ;
    T2CONbits.TMR2ON = 1;
    while(!PIR1bits.SSPIF)         //Wait till start sent
    {
        //if bus time out occcurs then stop and return
        if(PIR1bits.TMR2IF)
        {
            SSPCON2bits.PEN     = 1 ;           //Stop initiate (7)
            return 0;
        }
    }
    PIR1bits.SSPIF      = 0 ;               //reset flag

    //---2) Slave address sent
    SSPBUF              = slave_add;     //slave address (1)
    while(!PIR1bits.SSPIF);         //Wait till end of ninth cycle
    //timer used to exit infinite while loop in case communication hangs
    T2CONbits.TMR2ON = 0;
    PIR1bits.TMR2IF = 0;
    TMR2 = 0 ;
    T2CONbits.TMR2ON = 1;
    while(!PIR1bits.SSPIF)                 //Wait till end of ninth cycle
    {
        if(PIR1bits.TMR2IF)
        {
            SSPCON2bits.PEN     = 1 ;           //Stop initiate (7)
            return 0;
        }
    }
    PIR1bits.SSPIF      = 0 ;               //reset flag

     //---3) Proceed if slave detects the address and acknowledges else retrun
    if(!SSPCON2bits.ACKSTAT)         //proceed if ack received (2)
    {
        SSPCON2bits.ACKSTAT = 1 ;           //reset ack flag

        //---4) Data (REGISTER ADDRESS) sent to slave
        SSPBUF              = i2c_add ;  //register address (3)
        //timer used to exit infinite while loop in case communication hangs
        T2CONbits.TMR2ON = 0;
        PIR1bits.TMR2IF = 0;
        TMR2 = 0 ;
        T2CONbits.TMR2ON = 1;
        while(!PIR1bits.SSPIF)             //Wait till end of ninth cycle
        {
            if(PIR1bits.TMR2IF)
            {
                SSPCON2bits.PEN     = 1 ;           //Stop initiate (7)
                return 0;
            }
        }
        PIR1bits.SSPIF      = 0 ;           //reset flag

        //---5) Proceed if slave receives the data and acknowledges else retrun
        if(!SSPCON2bits.ACKSTAT)             //proceed if ack received (4)
        {
            SSPCON2bits.ACKSTAT = 1 ;           //reset ack flag

            //---6) Data sent to slave
            SSPBUF              = i2c_data ;   //start conversion (5)
            //timer used to exit infinite while loop in case communication hangs
            T2CONbits.TMR2ON = 0;
            PIR1bits.TMR2IF = 0;
            TMR2 = 0 ;
            T2CONbits.TMR2ON = 1;
            while(!PIR1bits.SSPIF)             //Wait till end of ninth cycle
            {
                if(PIR1bits.TMR2IF)
                {
                    SSPCON2bits.PEN     = 1 ;           //Stop initiate (7)
                    return 0;
                }
            }
            PIR1bits.SSPIF      = 0 ;           //reset flag

             //---7) Proceed if slave receives the data and acknowledges else retrun
            if(!SSPCON2bits.ACKSTAT)             //proceed if ack received (6)
            {
                SSPCON2bits.ACKSTAT = 1 ;           //reset ack flag
                
                //---6) Stop command
                SSPCON2bits.PEN     = 1 ;           //Stop initiate (7)
                while(!PIR1bits.SSPIF);             //Wait till end of ninth cycle
                PIR1bits.SSPIF      = 0 ;           //reset flag
            }
            else
                return 0 ;
        }
        else
            return 0 ;
    }
    else
        return 0 ;

    return 1;
}

char i2c_read(char i2c_add, char *i2c_data)
{
     /*----Read operation--one byte----
     * StartBit--SlaveAddress--W--ACK--Address--ACK--Sr--SlaveAddress--R--ACK--Data--bar(MACK)--StopBit
     * 1) Master issues start bit (S) followed by 7-bit (so remember left shifting 8 bit address by 1) slave address (0X40) and then W bit (0) indicating write operation
     * 2) Slave acknowledges (A) by pulling SDA line low for high duration of the ninth SCL cycle
     * 3) Master issues register address from where to read the data
     * 4) Slave acknowledges (A)
     * 5) Master issues repeated start command (Sr) followed by slave address (0X40) and then R bit (1) indicating read operation
     * 6) Slave acknowledges (A)
     * 7) Slave outputs data from the selected register under SCL control of master
     * 8) Master should not acknowledge
     * 9) Master issues stop bit (P)
     */

    SSPCON2bits.RCEN    = 0 ;           //receive idle
    PIR1bits.SSPIF      = 0 ;           //reset flag
    SSPCON2bits.SEN     = 1 ;           //Start initiated

    //timer used to exit infinite while loop in case communication hangs
    T2CONbits.TMR2ON = 0;
    PIR1bits.TMR2IF = 0;
    TMR2 = 0 ;
    T2CONbits.TMR2ON = 1;
    while(!PIR1bits.SSPIF)                  //Wait till start sent
    {
        if(PIR1bits.TMR2IF)
        {
            SSPCON2bits.PEN     = 1 ;       //Stop initiate (7)
            return 0;
        }
    }
    PIR1bits.SSPIF      = 0 ;     
    SSPBUF              = slave_add ;   //slave address

    while(!PIR1bits.SSPIF);             //Wait till end of ninth cycle
    PIR1bits.SSPIF      = 0 ;           //reset flag

    if(!SSPCON2bits.ACKSTAT)             //proceed if ack received
    {
        SSPCON2bits.ACKSTAT = 1 ;           //reset ack flag
        SSPBUF              = i2c_add ;  //register address
        while(!PIR1bits.SSPIF);             //Wait till end of ninth cycle
        PIR1bits.SSPIF      = 0 ;           //reset flag

        if(!SSPCON2bits.ACKSTAT)             //proceed if ack received
        {
            SSPCON2bits.ACKSTAT = 1 ;           //reset ack flag
            SSPCON2bits.RSEN    = 1 ;           //repeat start
            while(!PIR1bits.SSPIF);             //Wait till end of ninth cycle
            PIR1bits.SSPIF      = 0 ;           //reset flag

            SSPBUF              = 0x81 ;   //start conversion
            while(!PIR1bits.SSPIF);             //Wait till end of ninth cycle
            PIR1bits.SSPIF      = 0 ;           //reset flag

            if(!SSPCON2bits.ACKSTAT)             //proceed if ack received
            {
                SSPCON2bits.ACKSTAT = 1 ;           //reset ack flag
                 while(SSPSTATbits.R_NOT_W);             //Wait till end of ninth cycle
                SSPCON2bits.RCEN    = 1 ;           //receive enable
                while(!PIR1bits.SSPIF);             //Wait till end of ninth cycle
                PIR1bits.SSPIF      = 0 ;           //reset flag
                *i2c_data = SSPBUF ;
                //while(!SSPSTATbits.BF);             //Wait till end of ninth cycle
                
                
                SSPCON2bits.ACKDT=1;
                SSPCON2bits.ACKEN=1;
                while(!PIR1bits.SSPIF);             //Wait till end of ninth cycle
                PIR1bits.SSPIF      = 0 ;           //reset flag

                SSPCON2bits.PEN     = 1 ;           //Stop initiate
                while(!PIR1bits.SSPIF);             //Wait till end of ninth cycle
                PIR1bits.SSPIF      = 0 ;           //reset flag
            }
            else
                return 0 ;
        }
        else
            return 0 ;
    }
    else
        return 0 ;

    return 1 ;
}


void read_dsth01()
{
    //----------------------------------------------------------------
    //Temperature & Humidity
    /*----Write operation----
     * StartBit--SlaveAddress--W--ACK--Address--ACK--Data--ACK--StopBit
     * 1) Master issues start bit (S) followed by slave address (0X40) and then W bit (0) indicating write operation
     * 2) Slave acknowledges (A) by pulling SDA line low for high duration of the ninth SCL cycle
     * 3) Master issues register address where to write the data
     * 4) Slave acknowledges (A)
     * 5) Master issues data to be written in the register
     * 6) Slave acknowledges (A)
     * 7) Master issues stop bit (P)
    */
    /*----Read operation--one byte----
     * StartBit--SlaveAddress--W--ACK--Address--ACK--Sr--SlaveAddress--R--ACK--Data--bar(MACK)--StopBit
     * 1) Master issues start bit (S) followed by 7-bit (so remember left shifting 8 bit address by 1) slave address (0X40) and then W bit (0) indicating write operation
     * 2) Slave acknowledges (A) by pulling SDA line low for high duration of the ninth SCL cycle
     * 3) Master issues register address from where to read the data
     * 4) Slave acknowledges (A)
     * 5) Master issues repeated start command (Sr) followed by slave address (0X40) and then R bit (1) indicating read operation
     * 6) Slave acknowledges (A)
     * 7) Slave outputs data from the selected register under SCL control of master
     * 8) Master should not acknowledge
     * 9) Master issues stop bit (P)
     */
      /*----Read operation--two bytes----
     * StartBit--SlaveAddress--W--ACK--Address--ACK--Sr--SlaveAddress--R--ACK--DataH--MACK--DataL--bar(MACK)--StopBit
     * 1) Master issues start bit (S) followed by 7-bit (so remember left shifting 8 bit address by 1) slave address (0X40) and then W bit (0) indicating write operation
     * 2) Slave acknowledges (A) by pulling SDA line low for high duration of the ninth SCL cycle
     * 3) Master issues register address from where to read the data
     * 4) Slave acknowledges (A)
     * 5) Master issues repeated start command (Sr) followed by 7-bit slave address (0X40) and then R bit (1) indicating read operation
     * 6) Slave acknowledges (A)
     * 7) Slave outputs dataH from the selected register under SCL control of master 
     * 8) Master should acknowledges first high byte
     * 9) Slave outputs dataL from the selected register under SCL control of master  
     * 10)Master should not acknowledge second low byte
     * 11)Master issues stop bit (P)          
     */
    // We will read only one byte in one operation so ignore two byte operation. To read two bytes we will repeat one byte operation.
    //-------------------------------------------------------
    //----read temperature data
    //tell the device to start conversion by writing 0x11 in CONFIG register
    if(!i2c_write(register_3,temp_start))
        return ;
    dsth01_status = 1;

    while(dsth01_status)    //poll the STATUS register until it is low
    {
        if(!i2c_read(register_0, &dsth01_status))
            return ;
    }

    // return ;
    //once the status = 0 read high byte and then low byte
    if(!i2c_read(register_1, &tempH))
        return ;

    if(!i2c_read(register_2, &tempL))
        return ;


    /*temp_dec = tempH;
    temp_dec <<= 8 ;
    temp_dec += tempL ;
    temp_dec >>= 2;
    temp_buffer[0] = temp_dec/32 - 50 ;
        
    TXREG = ',';
          while(TXSTAbits.TRMT == 0 );
    TXREG = temp_buffer[0]/100+48;
          while(TXSTAbits.TRMT == 0 );
    TXREG = temp_buffer[0]/10+48;
          while(TXSTAbits.TRMT == 0 );
    TXREG = temp_buffer[0]%10+48;
         while(TXSTAbits.TRMT == 0 );
    TXREG = 'C';
          while(TXSTAbits.TRMT == 0 );
   */
    //-----------------------------

    //----read humidity data
    if(!i2c_write(register_3,hum_start))
        return ;
    dsth01_status = 1;
    while(dsth01_status)
    {
       if(!i2c_read(register_0, &dsth01_status))
            return ;
    }
    if(!i2c_read(register_1, &humH))
        return ;
    if(!i2c_read(register_2, &humL))
        return ;

    /*hum_dec = humH;
    hum_dec <<= 8 ;
    hum_dec += humL ;
    hum_dec >>= 4;
    hum_buffer[0] = (hum_dec-16)/24 ;

    TXREG = ' ';
          while(TXSTAbits.TRMT == 0 );
    TXREG = hum_buffer[0]/100+48;
          while(TXSTAbits.TRMT == 0 );
    TXREG = hum_buffer[0]/10+48;
          while(TXSTAbits.TRMT == 0 );
    TXREG = hum_buffer[0]%10+48;
         while(TXSTAbits.TRMT == 0 );
    TXREG = '%';
         while(TXSTAbits.TRMT == 0 );*/
    //-----------------------------
}

void bin_io()
{

    write_com_lcd(clear_lcd);
    strcpy(lcd_string,"Wind=");
    write_str_lcd(&lcd_string);
    write_char_lcd((wind_vel/10)%10+48);
    write_char_lcd(wind_vel%10+48);
    strcpy(lcd_string,"m/s,");
    write_str_lcd(&lcd_string);
    write_char_lcd(wind_dirH);
    write_char_lcd(wind_dirL);

    write_com_lcd(line2);
    
    strcpy(lcd_string,"T=");
    write_str_lcd(&lcd_string);
    write_char_lcd(temp_data/100+48);
    write_char_lcd(temp_data/10+48);
    write_char_lcd(temp_data%10+48);
    write_char_lcd(0xDF);
    write_char_lcd('C');

    write_char_lcd(' ');
    strcpy(lcd_string,"RH=");
    write_str_lcd(&lcd_string);
    write_char_lcd(hum_data/100+48);
    write_char_lcd(hum_data/10+48);
    write_char_lcd(hum_data%10+48);
    write_char_lcd('%');


    TXREG = 'V';
    while(TXSTAbits.TRMT == 0 );
    TXREG = 'c';
    while(TXSTAbits.TRMT == 0 );
    TXREG = 'c';
    while(TXSTAbits.TRMT == 0 );
    TXREG = '=';
    while(TXSTAbits.TRMT == 0 );
    TXREG = '0';
    while(TXSTAbits.TRMT == 0 );
    TXREG = '0';
    while(TXSTAbits.TRMT == 0 );
    TXREG = '.';
    while(TXSTAbits.TRMT == 0 );
    TXREG = '0';
    while(TXSTAbits.TRMT == 0 );
    TXREG = '0';
    while(TXSTAbits.TRMT == 0 );
    TXREG = 'v';
    while(TXSTAbits.TRMT == 0 );
    TXREG = ' ';
    while(TXSTAbits.TRMT == 0 );

    TXREG = 'W';
    while(TXSTAbits.TRMT == 0 );
    TXREG = 'i';
    while(TXSTAbits.TRMT == 0 );
    TXREG = 'n';
    while(TXSTAbits.TRMT == 0 );
    TXREG = 'd';
    while(TXSTAbits.TRMT == 0 );
    TXREG = ' ';
    while(TXSTAbits.TRMT == 0 );
    TXREG = 'v';
    while(TXSTAbits.TRMT == 0 );
    TXREG = 'e';
    while(TXSTAbits.TRMT == 0 );
    TXREG = 'l';
    while(TXSTAbits.TRMT == 0 );
    TXREG = '=';
    while(TXSTAbits.TRMT == 0 );
    TXREG = (wind_vel/10)%10+48;
    while(TXSTAbits.TRMT == 0 );
    TXREG = wind_vel%10+48;
    while(TXSTAbits.TRMT == 0 );
    TXREG = 'm';
    while(TXSTAbits.TRMT == 0 );
    TXREG = '/';
    while(TXSTAbits.TRMT == 0 );
    TXREG = 's';

    TXREG = ' ';
    while(TXSTAbits.TRMT == 0 );

    TXREG = 'W';
    while(TXSTAbits.TRMT == 0 );
    TXREG = 'i';
    while(TXSTAbits.TRMT == 0 );
    TXREG = 'n';
    while(TXSTAbits.TRMT == 0 );
    TXREG = 'd';
    while(TXSTAbits.TRMT == 0 );
    TXREG = ' ';
    while(TXSTAbits.TRMT == 0 );
    TXREG = 'd';
    while(TXSTAbits.TRMT == 0 );
    TXREG = 'i';
    while(TXSTAbits.TRMT == 0 );
    TXREG = 'r';
    while(TXSTAbits.TRMT == 0 );
    TXREG = '=';
    TXREG = wind_dirH;
    while(TXSTAbits.TRMT == 0 );
    TXREG = wind_dirL;
    while(TXSTAbits.TRMT == 0 );

    TXREG = ' ';
          while(TXSTAbits.TRMT == 0 );
    TXREG = 'T';
          while(TXSTAbits.TRMT == 0 );
    TXREG = 'e';
          while(TXSTAbits.TRMT == 0 );
    TXREG = 'm';
          while(TXSTAbits.TRMT == 0 );
    TXREG = 'p';
          while(TXSTAbits.TRMT == 0 );
    TXREG = '=';
          while(TXSTAbits.TRMT == 0 );
    TXREG = temp_data/100+48;
          while(TXSTAbits.TRMT == 0 );
    TXREG = temp_data/10+48;
          while(TXSTAbits.TRMT == 0 );
    TXREG = temp_data%10+48;
         while(TXSTAbits.TRMT == 0 );
    TXREG = 'd';
          while(TXSTAbits.TRMT == 0 );
    TXREG = 'e';
          while(TXSTAbits.TRMT == 0 );
    TXREG = 'g';
          while(TXSTAbits.TRMT == 0 );
    TXREG = 'C';
          while(TXSTAbits.TRMT == 0 );

    TXREG = ' ';
          while(TXSTAbits.TRMT == 0 );
    TXREG = 'H';
          while(TXSTAbits.TRMT == 0 );
    TXREG = 'u';
          while(TXSTAbits.TRMT == 0 );
    TXREG = 'm';
          while(TXSTAbits.TRMT == 0 );
    TXREG = 'i';
          while(TXSTAbits.TRMT == 0 );
    TXREG = 'd';
          while(TXSTAbits.TRMT == 0 );
    TXREG = 'i';
          while(TXSTAbits.TRMT == 0 );
    TXREG = 't';
          while(TXSTAbits.TRMT == 0 );
    TXREG = 'y';
          while(TXSTAbits.TRMT == 0 );
    TXREG = '=';
          while(TXSTAbits.TRMT == 0 );
    TXREG = hum_data/100+48;
          while(TXSTAbits.TRMT == 0 );
    TXREG = hum_data/10+48;
          while(TXSTAbits.TRMT == 0 );
    TXREG = hum_data%10+48;
         while(TXSTAbits.TRMT == 0 );
    TXREG = '%';
         while(TXSTAbits.TRMT == 0 );

    TXREG = '\n';
    while(TXSTAbits.TRMT == 0 );
    TXREG = '\r';
    while(TXSTAbits.TRMT == 0 );

    // TXREG = wind_dir_buffer[2]%10+48;
   // while(TXSTAbits.TRMT == 0 );

}


//---------------------------------------------------------------------------------------------------------------------
// High priority interrupt
void interrupt high_isr (void)
{
    char temporary_i;
    //Timer0 interrupt set to (inverse of 2500000/16/62500) = 400 ms
    if(INTCONbits.TMR0IF)
    {
        INTCONbits.TMR0IF = 0;
        TMR0 = 0xBDC ;
        TMR0 = 0xC2F7 ;  //100 ms


        //-----------------------------------------------------------------
        //Wind direction
        //-----------------------------------------------------------------
        //----wind direction is sampled using ADC
        ADRESH=0;
        ADRESL=0;
        ADCON0bits.GO_NOT_DONE = 1;     //start conversion
        while(ADCON0bits.GO_NOT_DONE);  //wait for conversion
        //read 10 bit buffer
        wind_dir_dec = ADRESH ;
        wind_dir_dec <<= 8 ;
        wind_dir_dec += ADRESL ;
        wind_dir_dec >>= 6 ;

        wind_dir_voltage = wind_dir_dec*3.35/1023;

        if(wind_dir_voltage>=0.2)
            wind_dir_buffer[inte_sample_counter] = 90 ;   //wind dir buffer
        if(wind_dir_voltage>=0.5)
            wind_dir_buffer[inte_sample_counter] = 135 ;   //wind dir buffer
        if(wind_dir_voltage>=.9)
            wind_dir_buffer[inte_sample_counter] = 180 ;   //wind dir buffer
        if(wind_dir_voltage>=1.4)
            wind_dir_buffer[inte_sample_counter] = 45 ;   //wind dir buffer
        if(wind_dir_voltage>=2)
            wind_dir_buffer[inte_sample_counter] = 225 ;   //wind dir buffer
        if(wind_dir_voltage>=2.5)
            wind_dir_buffer[inte_sample_counter] = 0 ;   //wind dir buffer
        if(wind_dir_voltage>=2.8)
            wind_dir_buffer[inte_sample_counter] = 315 ;   //wind dir buffer
        if(wind_dir_voltage>=3.0)
            wind_dir_buffer[inte_sample_counter] = 270 ;   //wind dir buffer
        if(wind_dir_voltage>=3.25||wind_dir_voltage<0.2)
        {
            if(inte_sample_counter>0)
                wind_dir_buffer[inte_sample_counter] = wind_dir_buffer[inte_sample_counter-1] ;   //wind dir buffer
            else
                wind_dir_buffer[inte_sample_counter] = wind_dir_buffer[24] ;   //wind dir buffer
        }

       // wind_dir_buffer[inte_sample_counter] = wind_dir_voltage;

       //temperature & Humidity
       //------------temperature and humidity is sampled using I2C
        read_dsth01();

        temp_dec = tempH;
        temp_dec <<= 8 ;
        temp_dec += tempL ;
        temp_dec >>= 2;
        temp_buffer[inte_sample_counter] = temp_dec/32 - 50 ;

        hum_dec = humH;
        hum_dec <<= 8 ;
        hum_dec += humL ;
        hum_dec >>= 4;
        hum_buffer[inte_sample_counter] = hum_dec/16 - 24 ;

        wind_dir_instant= wind_dir_buffer[inte_sample_counter];
        temp_buffer_instant =  temp_buffer[inte_sample_counter];
        hum_buffer_instant =  hum_buffer[inte_sample_counter] ;

        inte_sample_counter++;          //interrupt loop counter

        // integrate the data over 10 sec
        if(inte_sample_counter==10)     //at 25 time = 400ms*25 = 10 sec
        {
            inte_sample_counter=0;

            wind_dir_dec = 0 ;
            temp_dec = 0 ;
            hum_dec = 0 ;

            for(temporary_i=0;temporary_i<10;temporary_i++)
            {    
               wind_dir_dec += wind_dir_buffer[temporary_i] ;
               temp_dec     += temp_buffer[temporary_i]     ;
               hum_dec      += hum_buffer[temporary_i]      ;                       
            }

          
            wind_dir_data   = 0.1*wind_dir_dec ;       //wind dir data
            temp_data       = 0.1*temp_dec      ;      //temperature data
            hum_data        = 0.1*hum_dec       ;      //humidity data
            //wind_vel        = 0.24*wind_count  ;        //wind vel data
            wind_vel        = 0.67*wind_count  ;        //wind vel data
            wind_count      = 0 ;                       //wind buffer
            rain_           = 0.2794*rain_count;        //rain data
            rain_count      = 0 ;                       //rain buffer

           // wind_dir_data = wind_dir_instant;
          //  temp_data     = temp_buffer_instant     ;
           // hum_data      = hum_buffer_instant      ;


            if(wind_dir_data==0)
            {
                wind_dirH=' ' ;
                wind_dirL='N' ;
            }
            else if(wind_dir_data==45)
            {
                wind_dirH='N' ;
                wind_dirL='E' ;
            }
            else if(wind_dir_data==90)
            {
                wind_dirH=' ' ;
                wind_dirL='E' ;
            }
            else if(wind_dir_data==135)
            {
                wind_dirH='S' ;
                wind_dirL='E' ;
            }
            else if(wind_dir_data==180)
             {
                wind_dirH=' ' ;
                wind_dirL='S' ;
            }
            else if(wind_dir_data==225)
             {
                wind_dirH='S' ;
                wind_dirL='W' ;
            }
            else if(wind_dir_data==270)
             {
                wind_dirH=' ' ;
                wind_dirL='W' ;
            }
            else if(wind_dir_data==315)
            {
                wind_dirH='N' ;
                wind_dirL='W' ;
            }

           // bin_io();

           //test pin 10 sec
        }

        //test pin .4 sec
    }

    if(INTCONbits.INT0IF)
    {
        //Wind speed
        INTCONbits.INT0IF = 0;
        wind_count++ ;
        //test pin wind speed interrupt
    }

    if(INTCON3bits.INT1F)
    {
        //Rain
        INTCON3bits.INT1F = 0;
        rain_count++ ;
        //test pin rain guage interrupt
    }
    
    //test pin any interrupt
}

//----------------------------------------------------------------------------


//----------------------------------------------------------------------------
// Low priority interrupt routine
void interrupt low_priority low_isr (void)
{
    //Timer1 interrupt set to (inverse of 2500000/8/62500) = 200 ms
    if(PIR1bits.TMR1IF)
    {
        TMR1 = 0x0BDC ;

        inte_data_counter++;        //interrupt loop counter

        if(inte_data_counter==2)   //at 50 time = 200ms*50 = 10 sec
        {
            inte_data_counter=0;
            bin_io();

             //test pin 10 sec
             //test pin any interrupt

        }

        //test pin .2 sec
    }
}
//---------------------------------------------------------------------------------------------------------------------
//---------------------------------------------------------------------------------------------------------------------

