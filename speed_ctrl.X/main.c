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
#include <string.h>
#include <libpic30.h>
#include <p33FJ128GP802.h>



#define DELAY_105uS asm volatile ("REPEAT, #4201"); Nop(); // 105uS delay
#define DELAY_uS1  asm volatile ("REPEAT, #4000"); Nop(); 


#define     BRGVAL      (((FP/BAUDRATE)/16)-1)
#define     FP          40000000
#define     BAUDRATE    115200

#define     PULSES      39999
#define     INT_TIME    0.001 

#define     PULSES3     (65535*8)
#define     TIME3       0.0128


#define     STBY        LATAbits.LATA0
#define     A_1         LATAbits.LATA1
#define     A_2         LATAbits.LATA2
#define     TEST        LATAbits.LATA3 

#define     IP_REMOTE   192.168.1.13
#define     IP_LOCAL    192.168.1.160
#define     QUOTE       34

// Constantes para PID Digital utilizando forward Euler
#define     TS          INT_TIME 

#define     KP          (25)
#define     KI          (10*TS) 
#define     KD          (1/TS) 


#define     RPM_MAX     60 
#define     SUP         5 
#define     INF         0 
#define     M           (RPM_MAX/SUP)
#define     K           (PULSES/RPM_MAX)


uint16_t pwm_counter = 0;
uint16_t dt_encoder = 0;
uint16_t test;
double rpm, rpm_1;
char received_char[50];
char send_char[20], s1[20];
char ip_local[14] = "192.168.1.160";
char ip_remote[14] = "192.168.1.13";
char * p_point1;
char in_bytes[20], output[20];

char b[5];
uint16_t ba, bb, bc, bd;
uint16_t port = 900;
uint8_t i = 0;

bool b_flag_uart = false;


// variables para PID
double ref, feedback;
float e, E, ed, e_old, pid_out;


uint16_t pid_int;

void init_osc(void);
void init_pins(void);
void init_timer(uint8_t timer, uint16_t count_time);
void init_pwm(void);
void init_input_capture1(void);
void init_uart(void);
void write_uart(char *data);
char read_uart(void);
void send_uart(char data);
void delay_us(uint16_t us);
void init_wifi(void);
double ic_getrpm(void);
void read_text(void);

int main(void)
{
    init_osc();
    init_pins();
    init_timer(2, PULSES);
    init_timer(3, 65535);
    init_pwm();
    init_uart();
    //write_uart("START\r\n"); 
    delay_us(500);
    //init_wifi();
    init_input_capture1();
    write_uart("START\n");

    /*p_my_array = &a[0];
     *p_my_array = 23;
    p_my_array++;
     *p_my_array = 23;*/

    while (1)
    {
        OC1RS = PULSES/2;
        delay_us(500);
        read_text();
        write_uart(in_bytes);

        //delay_us(1000); 


        //strncpy(compare, ref_wifi, 4);

        /*if(strcmp("+IPD",compare)==0)
        {
            write_uart("AT+CIPSEND=5");
            delay_us(500); 
            write_uart("hallo");
        }*/


        /*if(received_char != '\0')
        {
            write_uart(received_char); 
        }*/
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
    TRISBbits.TRISB8 = 1;
    AD1PCFGL = 0b111111111111;
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
    OC1RS = PULSES / 2; // Ciclo de trabajo de segundo pulso
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
        IPC0bits.T1IP = 6; //0 deshabilitado, 7 maxima prioridad
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
        PR2 = count_time; //cada 1 ms
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

    case 3:
        /*
           Segmento de codigo para configurar el registro TxCON
         */
        T3CONbits.TON = 0; //apagamos timer
        T3CONbits.TCS = 0; //Reloj interno (fosc/2)
        T3CONbits.TSIDL = 0;
        T3CONbits.TCKPS = 0b01; //1:8 prescaler
        T3CONbits.TGATE = 0;
        //T3CONbits.TSYNC = 0;    //No sincronizar

        //Segmento de codigo para configurar la duracion del timer
        TMR3 = 0;
        PR3 = count_time;
        //Interrupcion cada: ((500+1)*(1/FOSC))*prescaler s

        /*
           Segmento de codigo para configurar interrupciones
           INTCON1 = INTCON2 = 0
         */
        IFS0bits.T3IF = 0; //Limpiamos bandera de interrupcion
        IPC2bits.T3IP = 7; //nivel de prioridad
        IEC0bits.T3IE = 1; //Habilitamos interrupcion  Timer1

        //Se enciende el timerx
        T1CONbits.TON = 1;
    }
}

void
init_input_capture1(void)
{
    // Initialize the Input Capture Module
    IC1CONbits.ICM = 0b00; // Disable Input Capture 1 module    
    RPINR7bits.IC1R = 0b00101; //Input capture tied to RP5 -IC1
    IC1CONbits.ICTMR = 0; // Select Timer3 as the IC1 Time base
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
    IC2CONbits.ICI = 2; // Interrupt on every 2nd capture event
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

    IEC0bits.U1RXIE = 1; // RX interrupt - desactivada
    IFS0bits.U1RXIF = 0;
    IPC2bits.U1RXIP = 5;

    RPINR18bits.U1RXR = 8; //Pin para  RX = RB8
    RPOR4bits.RP9R = 0b00011; ////Pin para  TX = RB9

    U1MODEbits.UARTEN = 1; // Enable UART
    U1STAbits.UTXEN = 1; // Enable UART TX
    DELAY_105uS //Delay para estabilizar BR
            //U1TXREG = 'J'; // Prueba de transmision
}

/*
void
read_text(void)
{
    sprintf(received_char, "\0");
    sprintf(a, "\0");
    b = 0;
    bool flag_read = true;
    IFS0bits.U1RXIF = 0;
    while (!U1STAbits.URXDA);
    while (U1STAbits.RIDLE == 0 || U1STAbits.URXDA == 1)
    {

        b = U1RXREG;
        sprintf(a, "%c", b);
        strcat(received_char, a);
        if (b == 'z')
        {
            break;
        }
        if (U1STAbits.RIDLE == 0)
        {
            while (!IFS0bits.U1RXIF);
        }
        if (U1STAbits.URXDA == 1)
        {
            continue;
        }
        if (U1STAbits.OERR == 1)
        {
            U1STAbits.OERR = 0;
            continue;
        }
        b = 0;
    }
    b_flag_uart = true;
}
 */
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
     *  AT+CIPSTART="TCP","192.168.1.13",900\r\n
     * 5. WAIT FOR OK
     * 6. IF OK: 
     *      SEND: AT+CIPSEND=#CHARS
     *            MESSAGE (1S)
     *      RECEIVE: +IPD,1:H  10chars 
     * 
     * List available APS
     *      AT+CWLAP
     */
    write_uart("AT\r\n");
    delay_us(1000);
    write_uart("AT+RST\r\n");
    delay_us(2000);
    char send1[55];
    write_uart("AT+CWMODE=3\r\n");
    delay_us(2000);
    sprintf(send1, "AT+CIPSTA=%c%s%c,%c192.168.1.1%c,%c255.255.255.0%c\r\n", QUOTE, ip_local, QUOTE, QUOTE, QUOTE, QUOTE, QUOTE);
    write_uart(send1);
    delay_us(2000);
    char send2[40];
    sprintf(send2, "AT+CIPSTART=%cTCP%c,%c%s%c,800\r\n", QUOTE, QUOTE, QUOTE, ip_remote, QUOTE);
    write_uart(send2);
    delay_us(2000);
}

double
ic_getrpm(void)
{
    uint16_t t1 = 0, t2 = 0, dt_encoder = 0;
    double T = 0, rpms, temp = 0, freq = 0, rpm_old;
    rpm_old = rpms;
    t1 = IC1BUF; //Tiempo senal 1
    t2 = IC1BUF; //Tiempo senal 2
    //calculamos la diferencia de tiempo
    if (t2 > t1)
        dt_encoder = t2 - t1;
    else
        dt_encoder = (PR3 - t1) + t2;

    temp = dt_encoder * (TIME3 / (2.5 * PULSES3))*1000;

    T = (temp * 12.0);
    freq = 1 / T; //Hz
    rpms = 60.0 * freq;
    if (rpms > 100)
    {
        rpms = rpm_old;
    }
    char ic[20];
    sprintf(ic, "RPM: %u\n", t2);
    write_uart(ic);
    return rpms;
}

void
read_text(void)
{
    write_uart("Leyendo\n");
    uint8_t i, i_old;
    unsigned char temp;
    bool flag1 = true;
    bool flag_in = false;
    while (!U1STAbits.URXDA);
    temp = U1RXREG;
    in_bytes[0] = temp;
    if (temp == '+')
    {
        in_bytes[0] = temp;

        for (i = 1; i < 10; i++) //Loop over range i = 1 to 10 inclusively
        {
            while (!U1STAbits.URXDA); //Wait until at least one byte is available
            in_bytes[i] = U1RXREG;

        }
    }
    else
    {
        for (i = 0; i < 101; i++)
        {
            temp = U1RXREG;
            U1STAbits.OERR = 0;
            if (temp == 0)
            {
                break;
            }
            send_uart(temp);
        }
    }
    write_uart(in_bytes);
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
    IFS0bits.T2IF = 0; // Clear Timer 2 interrupt flag
    TMR2 = 0;
    //OC1RS = PULSES; 
}

void __attribute__((__interrupt__, no_auto_psv)) _T3Interrupt(void)
{
    IFS0bits.T3IF = 0; // Clear Timer 3 interrupt flag
    TMR3 = 0;
    //OC1RS = PULSES; 
}



//IC-Interrupt

void __attribute__((__interrupt__, no_auto_psv)) _IC1Interrupt(void)
{
    rpm = ic_getrpm();
    IFS0bits.IC1IF = 0; //reiniciamos la captura del IC1
}

/*
 * UART Interrupts 
 */
void __attribute__((__interrupt__, no_auto_psv)) _U1RXInterrupt(void)
{
    //received_char = U1RXREG;
    /*
     * Clear error flags 
     */

    IFS0bits.U1RXIF = 0;

}

void __attribute__((__interrupt__, no_auto_psv)) _U1TXInterrupt(void)
{
    IFS0bits.U1TXIF = 0; // Clear TX Interrupt flag

}
