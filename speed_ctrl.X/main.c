/*
 * File:   main.c
 * Author: Jose Molina
 * Speed control
 *
 * Created on 12 de febrero de 2020, 06:23 PM
 */

// FBS


#pragma config BWRP = WRPROTECT_OFF     // Boot Segment Write Protect (Boot Segment may be written)
#pragma config BSS = NO_FLASH           // Boot Segment Program Flash Code Protection (No Boot program Flash segment)
#pragma config RBS = NO_RAM             // Boot Segment RAM Protection (No Boot RAM)

// FSS
#pragma config SWRP = WRPROTECT_OFF     // Secure Segment Program Write Protect (Secure segment may be written)
#pragma config SSS = NO_FLASH           // Secure Segment Program Flash Code Protection (No Secure Segment)
#pragma config RSS = NO_RAM             // Secure Segment Data RAM Protection (No Secure RAM)

// FGS
#pragma config GWRP = OFF               // General Code Segment Write Protect (User program memory is not write-protected)
#pragma config GSS = OFF                // General Segment Code Protection (User program memory is not code-protected)

// FOSCSEL
#pragma config FNOSC = FRCPLL           // Oscillator Mode (Internal Fast RC (FRC) w/ PLL)
#pragma config IESO = ON             // Internal External Switch Over Mode (Start-up device with user-selected oscillator source)

// FOSC
#pragma config POSCMD = NONE            // Primary Oscillator Source (Primary Oscillator Disabled)
#pragma config OSCIOFNC = ON            // OSC2 Pin Function (OSC2 pin has digital I/O function)
#pragma config IOL1WAY = ON            // Peripheral Pin Select Configuration (Allow Multiple Re-configurations)
#pragma config FCKSM = CSDCMD           // Clock Switching and Monitor (Both Clock Switching and Fail-Safe Clock Monitor are enabled)


// FPOR
#pragma config FPWRT = PWR1             // POR Timer Value (Disabled)
#pragma config ALTI2C = OFF             // Alternate I2C  pins (I2C mapped to SDA1/SCL1 pins)

// FICD
#pragma config ICS = PGD2               // Comm Channel Select (Communicate on PGC2/EMUC2 and PGD2/EMUD2)
#pragma config JTAGEN = OFF              // JTAG Port Enable (JTAG is Enabled)


#include <xc.h>
#include <math.h>
#include <stdlib.h>
#include <stdbool.h>
#include <stdio.h>
#include <libpic30.h>
#include <p33FJ128GP802.h>


#define DELAY_105uS asm volatile ("REPEAT, #4201"); Nop(); // 105uS delay
#define DELAY_uS1  asm volatile ("REPEAT, #4000"); Nop(); 


#define     BRGVAL      (((FP/BAUDRATE)/16)-1)
#define     FP          40000000
#define     BAUDRATE    115200


#define     STBY        LATAbits.LATA0
#define     A_1         LATAbits.LATA1
#define     A_2         LATAbits.LATA2
#define     TEST        LATAbits.LATA3 

#define     IP_REMOTE   192.168.1.13
#define     IP_LOCAL    192.168.1.160
#define     QUOTE       34

uint16_t pwm_counter = 0;
uint16_t dt_encoder = 0;
uint16_t test;
float rpm; 
char received_char;
char send_char[20], s1[20]; 
char ip_local[13] = "192.168.1.160", ip_remote[14] = "192.168.1.13";  
uint16_t port = 900; 

void init_osc(void);
void init_pins(void);
void init_timer(uint8_t timer, uint16_t count_time);
void init_pwm(void);
void init_input_capture(void);
void init_uart(void);
void write_uart(char *data);
char read_uart(void);
void send_uart(char data);
void delay_us(uint16_t us);
void init_wifi(void);
float ic_getrpm(void); 

int main(void)
{
    init_osc();
    init_pins();
    init_timer(2, 3999);
    init_pwm();
    init_uart();
    init_wifi(); 

    while (1)
    {
        /*
                pwm_counter++;
                if(pwm_counter == 6000)
                {
                   OC1RS++; 
                }
                if(OC1RS == 4000)
                {
                    OC1RS = 0; 
                }
         * */
        TEST = 0;
        delay_us(200);
        OC1RS = 4000;
        write_uart("hola\r\n");

        sprintf(send_char, "%u \r\n", 4593);
        test = atoi(send_char);
        delay_us(200);
        write_uart(send_char);
        delay_us(200);
        sprintf(s1, "%u \r\n", test);
        write_uart(s1);

        if (U1STAbits.URXDA == 1)
        {
            received_char = U1RXREG;
        }

    }

    return 0;
}

/*
 * Funciones 
 */

void
init_pins(void)
{
    TRISA = 0x0; //1 input
    TRISBbits.TRISB0 = 0;
    AD1PCFGL = 0b111111111;
    A_1 = 0;
    A_2 = 1;
    STBY = 1;
}

void
init_osc(void)
{
    // Oscilador: 40Mhz
    // Fosc= Fin*M/(N1*N2), Fcy=Fosc/2
    // Fosc= 8M*40/(2*2)=80Mhz para 8MHZ input del primario
    PLLFBD = 38; // M=40
    CLKDIVbits.PLLPOST = 0; // N1=2
    CLKDIVbits.PLLPRE = 0; // N2=2
    OSCTUN = 0; // Si se usa el FRC

    // Watch Dog Timer
    RCONbits.SWDTEN = 0;

    // Clock Switch --> PLL
    __builtin_write_OSCCONH(0x01); // NOSC = 001 para pasar a FRC + PLL

    __builtin_write_OSCCONL(0x01); // Switch
    while (OSCCONbits.COSC != 0b001); // Esperar hasta que este en PLL

    // Esperar hasta que PLL esté activo
    while (OSCCONbits.LOCK != 1)
    {
    };
}

void init_pwm(void)
{
    //Output Compare Modulo
    OC1CONbits.OCM = 0b000; // Desactivar modulo OC
    OC1R = 0; // Ciclo de trabajo de primer pulso
    OC1RS = 0; // Ciclo de trabajo de segundo pulso
    OC1CONbits.OCTSEL = 0; // Seleccionar Timer 2
    OC1R = 0; // Se carga el valor del registro
    OC1CONbits.OCM = 0b110; // PWM + Activar

    RPOR3bits.RP6R = 0b10010; //RB6 conectado a salida PWM
}

void
init_timer(uint8_t timer, uint16_t count_time)
{
    switch (timer)
    {

    case 1:
        /*
           Segmento de codigo para configurar el registro TxCON
         */
        T1CONbits.TON = 0;
        T1CONbits.TCS = 0;
        T1CONbits.TSIDL = 0;
        T1CONbits.TCKPS = 0b00; //1:1 prescaler
        T1CONbits.TGATE = 0;
        T1CONbits.TSYNC = 0;

        /*
           Segmento de codigo para configurar la duracion del timer
         */
        TMR1 = 0;
        PR1 = count_time;
        //Interrupcion cada: (500+1)*(1/FOSC)s

        /*
           Segmento de codigo para configurar interrupciones
           INTCON1 = INTCON2 = 0

         */
        IFS0bits.T1IF = 0;
        IPC0bits.T1IP = 7; //0 deshabilitado, 7 maxima prioridad
        IEC0bits.T1IE = 1;


        //Se enciende el timerx
        T1CONbits.TON = 1;


    case 2:
        /*
           Segmento de codigo para configurar el registro TxCON
         */
        T2CONbits.TON = 0;
        T2CONbits.TCS = 0;
        T2CONbits.TSIDL = 0;
        T2CONbits.TCKPS = 0b00; //1:1 prescaler
        T2CONbits.TGATE = 0;
        /*
           Segmento de codigo para configurar la duracion del timer
         */
        TMR2 = 0;
        PR2 = count_time; //cada 0.1 ms
        //Interrupcion cada: (PRx+1)*(1/FOSC)s

        /*
           Segmento de codigo para configurar interrupciones
           INTCON1 = INTCON2 = 0
         */
        IFS0bits.T2IF = 0;
        IPC1bits.T2IP = 7; //0 deshabilitado, 7 maxima prioridad
        IEC0bits.T2IE = 1;


        //Se enciende el timerx
        T2CONbits.TON = 1;
    }
}

void
init_input_capture1(void)
{
    // Initialize the Input Capture Module
    IC1CONbits.ICM = 0b00; // Disable Input Capture 1 module    
    RPINR7bits.IC1R = 0b00101; //Input capture tied to RP5 -IC1
    IC1CONbits.ICTMR = 1; // Select Timer2 as the IC1 Time base
    IC1CONbits.ICI = 1; // Interrupt on every second capture event
    IC1CONbits.ICM = 0b011; // Generate capture event on every Rising edge

    IPC0bits.IC1IP = 0b100; // Setup IC1 interrupt priority level  nivel superior al OC
    IFS0bits.IC1IF = 0; // Clear IC1 Interrupt Status Flag
    IEC0bits.IC1IE = 1; // Enable IC1 interrupt
}

void
init_input_capture2(void)
{
    // Initialize the Input Capture Module
    IC2CONbits.ICM = 0b00; // Disable Input Capture 1 module
    RPINR7bits.IC2R = 1; //Input capture tied to RP1 -IC1
    IC2CONbits.ICTMR = 1; // Select Timer2 as the IC1 Time base
    IC2CONbits.ICI = 0; // Interrupt on every  capture event
    IC2CONbits.ICM = 0b011; // Generate capture event on every Rising edge

    //Habilitamos interrupcion de captura y timer 2
    IPC1bits.IC2IP = 0b100; // Setup IC2 interrupt priority level  nivel superior al OC
    IFS0bits.IC2IF = 0; // Clear IC1 Interrupt Status Flag
    IEC0bits.IC2IE = 1; // Enable IC2 interrupt
}

void
init_uart(void)
{
    U1MODEbits.STSEL = 0; // 1-Stop bit
    U1MODEbits.PDSEL = 0; // Paridad nula

    U1MODEbits.ABAUD = 0; // Auto-BaudRate disable
    U1MODEbits.BRGH = 0; // Standard-Speed mode
    U1BRG = BRGVAL; // 115200 segun valor de U1BRG
    U1STAbits.UTXISEL0 = 0; // Interrupcion después de cada caracter en TX
    U1STAbits.UTXISEL1 = 0; // Interrupcion después de cada caracter en TX

    U1STAbits.URXISEL = 0; // Interrupcion después de cada caracter en RX


    IEC0bits.U1TXIE = 0; // TX interrupt - desactivada
    IFS0bits.U1TXIF = 0;
    IPC3bits.U1TXIP = 4;

    IEC0bits.U1RXIE = 0; // RX interrupt - desactivada
    IFS0bits.U1RXIF = 0;
    IPC2bits.U1RXIP = 3;

    RPINR18bits.U1RXR = 8; //Pin para  RX = RB8
    RPOR4bits.RP9R = 0b00011; ////Pin para  TX = RB9

    U1MODEbits.UARTEN = 1; // Enable UART
    U1STAbits.UTXEN = 1; // Enable UART TX
    DELAY_105uS //Delay para estabilizar BR
    U1TXREG = 'J'; // Prueba de transmision
}

char
read_uart(void)
{
    while (!U1STAbits.URXDA);
    return U1RXREG;
}

void
send_uart(char data)
{
    U1TXREG = data;
    Nop();
    while (!U1STAbits.TRMT);
}

void
write_uart(char *data)
{
    uint8_t j = 0;
    while (data[j] != '\0')
    {
        send_uart(data[j]);
        j++;
    }
}

void
delay_us(uint16_t us)
{
    uint16_t x;

    for (x = 0; x < (us * 10); x++)
    {
        DELAY_uS1
    }
}

void
init_wifi(void)
{
    /*
     * \r\n
     * 1. SET WIFI MODE: 
     *  AT+CWMODE=3
     * 2. Connect to a router. 
     *  AT+CWJAP="SSID","password"
     * 3. SET IP
     *  AT+CIPSTA="192.168.1.160","192.168.1.1","255.255.255.0"
     * 4. CONNECT TO TCP SERVER
     *  AT+CIPSTART="TCP","192.168.1.13",900
     * 5. WAIT FOR OK
     * 6. IF OK: 
     *      SEND: AT+CIPSEND=#CHARS
     *            MESSAGE (1S)
     *      RECEIVE: +IPD,4:HOLA   
     */
    char send[55]; 
    char resp; 
    write_uart("AT+CWMODE=3\r\n" ); 
    delay_us(500); 
    sprintf(send,"AT+CIPSTA=%c%s%c,%c192.168.1.1%c,%c255.255.255.0%c\r\n",QUOTE,ip_local,QUOTE,QUOTE,QUOTE,QUOTE,QUOTE); 
    write_uart(send); 
    delay_us(500);
   
    sprintf(send,"AT+CIPSTART=%cTCP%c,%c%s%c,%d\r\n,",QUOTE,QUOTE,QUOTE,ip_remote,QUOTE,port); 
    write_uart(send); 
    
}


float
ic_getrpm(void)
{
    uint16_t t1 = 0, t2 = 0, dt_encoder = 0;
    float temp;
    t1 = IC1BUF; //Tiempo senal 1
    t2 = IC1BUF; //Tiempo senal 2
    //calculamos la diferencia de tiempo
    if (t2 > t1)
        dt_encoder = t2 - t1;
    else
        dt_encoder = (PR2 - t1) + t2;

    //Calculamos RPM actual de la llanta
    temp = 1000 / dt_encoder * 12;
    return temp; 
}
/*
 Interrupts 
 */

//Timer-Interrupts

void __attribute__((__interrupt__, no_auto_psv)) _T1Interrupt(void)
{
    /* Interrupt Service Routine code goes here */
    IFS0bits.T1IF = 0; // Clear Timer 2 interrupt flag


}

void __attribute__((__interrupt__, no_auto_psv)) _T2Interrupt(void)
{
    /* Interrupt Service Routine code goes here */
    TMR2 = 0;
    IFS0bits.T2IF = 0; // Clear Timer 2 interrupt flag

}



//IC-Interrupt

void __attribute__((__interrupt__, no_auto_psv)) _IC1Interrupt(void)
{

    IFS0bits.IC1IF = 1; //reiniciamos la captura del IC1
    rpm = ic_getrpm(); 
}

/*
 * UART Interrupts 
 */
void __attribute__((__interrupt__, no_auto_psv)) _U1RXInterrupt(void)
{
    received_char = U1RXREG;
    /*
     * Clear error flags 
     */
    if (U1STAbits.OERR == 1)
    {
        U1STAbits.OERR = 0;
    }

    IFS0bits.U1RXIF = 0;

}

void __attribute__((__interrupt__, no_auto_psv)) _U1TXInterrupt(void)
{
    IFS0bits.U1TXIF = 0; // Clear TX Interrupt flag
    
}



