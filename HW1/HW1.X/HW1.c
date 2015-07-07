// HW1.c
// Steven Stachowiak
// ME 433 2015

#include<xc.h> // processor SFR definitions
#include<sys/attribs.h> // __ISR macro

// DEVCFGs here

// DEVCFG0
#pragma config DEBUG = OFF // no debugging
#pragma config JTAGEN = OFF // no jtag
#pragma config ICESEL = ICS_PGx1 // use PGED1 and PGEC1
#pragma config PWP = OFF // no write protect
#pragma config BWP = OFF // no boot write protect
#pragma config CP = OFF // no code protect

// DEVCFG1
#pragma config FNOSC = PRIPLL // use primary oscillator with pll
#pragma config FSOSCEN = OFF // turn off secondary oscillator
#pragma config IESO = OFF // no switching clocks
#pragma config POSCMOD = HS // high speed crystal mode
#pragma config OSCIOFNC = OFF // free up secondary osc pins
#pragma config FPBDIV = DIV_1 // divide CPU freq by 1 for peripheral bus clock
#pragma config FCKSM = CSDCMD // do not enable clock switch
#pragma config WDTPS = PS1 // slowest wdt (1:1)
#pragma config WINDIS = OFF // no wdt window
#pragma config FWDTEN = OFF // wdt off by default
#pragma config FWDTWINSZ = WINSZ_25 // wdt window at 25%

// DEVCFG2 - get the CPU clock to 40MHz
#pragma config FPLLIDIV = DIV_2 // divide input clock to be in range 4-5MHz (8MHz/2)
#pragma config FPLLMUL = MUL_20 // multiply clock after FPLLIDIV (4MHz*20)
#pragma config FPLLODIV = DIV_2 // divide clock after FPLLMUL to get 40MHz (80MHz/2)
#pragma config UPLLIDIV = DIV_2 // divide 8MHz input clock = 4MHz, then USB PLL multiplies this by 12 = 48MHz
#pragma config UPLLEN = ON // USB clock on

// DEVCFG3
#pragma config USERID = 0001000000010101 // some 16bit userid base-2 = 4117 base-10
#pragma config PMDL1WAY = ON // allow only one reconfiguration
#pragma config IOL1WAY = ON // allow only one reconfiguration
#pragma config FUSBIDIO = OFF // USB ID pin not controlled by USB module
#pragma config FVBUSONIO = OFF // USB BUSON not controlled by USB module

int readADC(void);

int main() {
    int press = 1;
    int potent = 0;

    // startup

    //Startup code to run as fast as possible and get pins back from bad defaults

    __builtin_disable_interrupts();

    // set the CP0 CONFIG register to indicate that
    // kseg0 is cacheable (0x3) or uncacheable (0x2)
    // see Chapter 2 "CPU for Devices with M4K Core"
    // of the PIC32 reference manual
    __builtin_mtc0(_CP0_CONFIG, _CP0_CONFIG_SELECT, 0xa4210583);

    // no cache on this chip!

    // 0 data RAM access wait states
    BMXCONbits.BMXWSDRM = 0x0;

    // enable multi vector interrupts
    INTCONbits.MVEC = 0x1;

    // disable JTAG to be able to use TDI, TDO, TCK, TMS as digital
    DDPCONbits.JTAGEN = 0;

    __builtin_enable_interrupts();

    // set up USER pin as input
    // Analog (ANx) pins default to be analog input. Use ANSELA or ANSELB to make them digital
    ANSELBbits.ANSB13 = 0; // makes pin B13 digital I/O
    TRISBbits.TRISB13 = 1; // makes pin B13 digital input
    
    // set up LED1 pin as a digital output
    PORTBbits.RB7 = 1; // activates pin B7
    TRISBbits.TRISB7 = 0; // makes pin B7 digital output

    // set up LED2 as OC1 using Timer2 at 1kHz
    ANSELBbits.ANSB15 = 0; // makes pin B15 digital I/O
    RPB15Rbits.RPB15R = 0b0101; // set B15 to OC1
    T2CONbits.TCKPS = 0 ; // prescaler 1:1
    PR2 = 39999 ; // period
    TMR2 = 0; // initialize Timer2 count to 0
    OC1CONbits.OC32 = 0; // select 16 bit timer (2 or 3)
    OC1CONbits.OCTSEL = 0; // select Timer2
    OC1CONbits.OCM = 0b110 ; // select pwm mode without fault pin
    OC1RS = 20000; // duty cycle = OC1RS/(PR2+1) = 50% gives medium bright LED2
    OC1R = 20000;
    T2CONbits.ON = 1; // turn Timer2 on
    OC1CONbits.ON = 1; // OC1 enabled

    // set up A0 as AN0

    ANSELAbits.ANSA0 = 1;
    AD1CON3bits.ADCS = 3;
    AD1CHSbits.CH0SA = 0;
    AD1CON1bits.ADON = 1;

    while (1) {
        _CP0_SET_COUNT(0); // set core timer to zero
        while (_CP0_GET_COUNT() < 5000000){ // create every-half-second timing for LED1 (core timer freq = cpu freq / 8)
            if (PORTBbits.RB13 == 0){ // USER button is pressed, toggle LED1 as fast as possible = with each run through this loop
                press = !press; // invert LED1
                LATBbits.LATB7 = press; // write change to pin
            }
            potent = readADC(); // read adc for potentiometer value
            potent = (potent * 20000) / 1023; // convert this for OC1RS
            OC1RS = potent; // set PWM duty cycle based on potentiometer
        }
        press = !press; // change LED1 status
        LATBbits.LATB7 = press; // write change to pin
    }
}

int readADC(void) {
    int elapsed = 0;
    int finish_t = 0;
    int sample_t = 20;
    int output = 0;

    AD1CON1bits.SAMP = 1;
    elapsed = _CP0_GET_COUNT();
    finish_t = elapsed + sample_t;
    while (_CP0_GET_COUNT() < finish_t) {
    }
    AD1CON1bits.SAMP = 0;
    while (!AD1CON1bits.DONE) {
    }
    output = ADC1BUF0;
    return output;
}