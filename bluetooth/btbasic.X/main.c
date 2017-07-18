#define FCY 29641200L               //number of instructions per milisec
#define FOSC (FCY*4)                //number of clock cycles
#define UART_BUFFER_SIZE 256
#define BAUDRATE 115200
#define BRGVAL ((FCY/BAUDRATE)/16)-1
#define SETUP_BUF_LENGTH 20
#define ANAL_BUF_LENGTH  15
#define M_SEC FCY*0.01 //0.001

#pragma config FCKSMEN=CSW_FSCM_OFF
#pragma config FOS=PRI //fonte cristal
#pragma config FPR=XT_PLL16 //oscilador a 16x cristal
#pragma config WDT=WDT_OFF // watchdog timer off 
#pragma config MCLRE=MCLR_EN // Turn MCLR pin ON and
#pragma config FPWRT=PWRT_OFF

#include <libpic30.h> //C30 compiler definitions
#include <p30F4011.h> // Defines os dspic registers
//#include <uart.h>
#include <string.h>
#include <math.h>
#include <stdlib.h>
#include <stdio.h>
//#include "UART.h"
//#include "CAN.h"
//#include <delay.h>

volatile unsigned int UART_write = 0;
volatile unsigned int UART_read = 0;
volatile unsigned char UARTbuffer[UART_BUFFER_SIZE];

void UART1_config(void);
void UART_send(char *UARTdata, int n);
void __attribute__((interrupt, auto_psv)) _U1RXInterrupt(void);
void timer1_init(void);

void timer1_init(void){

	T1CONbits.TCS   = 0;		/* use internal clock: Fcy               */
	T1CONbits.TGATE = 0;		/* Gated mode off                        */
	T1CONbits.TCKPS = 0;		/* prescale 1:1                          */
	T1CONbits.TSIDL = 0;		/* don't stop the timer in idle          */

	TMR1 = 0;					/* clears the timer register             */
	PR1 = M_SEC;				/* value at which the register overflows *
								 * and raises T1IF                       */

	/* interruptions */
	IPC0bits.T1IP = 2;			/* Timer 1 Interrupt Priority 0-7        */
	IFS0bits.T1IF = 0;			/* clear interrupt flag                  */
	IEC0bits.T1IE = 1;			/* Timer 1 Interrupt Enable              */


	T1CONbits.TON = 1;			/* starts the timer                      */
	return;
}

/*MAIN*/
int main (void){
    
    RCONbits.SWDTEN=1; //Enable Watchdog timer
    char st[14];
    char st_valor[14];
    int temperatura = 20;
    int tensao = 5;
    int corre= 10;
    
    _TRISD3 = 0; // output
    _TRISD1 = 0; //output
    _LATD3 = 1; //High 5v
    _LATD1 = 0; //Low, gnd
   
     
    UART1_config();    
    strcpy(st, "t");

    sprintf(st_valor, "t%d;v%d;i%d\n",temperatura,tensao,corre);
    strcat(st, st_valor);
   
    UART_send("+\r\n", strlen("+\r\n"));
    __delay_ms(50);
    UART_send("S-,BMS\r\n", strlen("S-,BMS\r\n"));
    __delay_ms(50);
     UART_send("SB,4\r\n", strlen("SB,4\r\n"));
      __delay_ms(50);
      UART_send("SF,1\r\n", strlen("SF,1\r\n"));
     __delay_ms(50);
      UART_send("SS,C0000001\r\n", strlen("SS,C0000001\r\n"));
     __delay_ms(10);
     UART_send("PZ\r\n", strlen("PZ\r\n"));
     __delay_ms(10);
     UART_send("PS,11223344556677889900AABBCCDDEEFF\r\n", strlen("PS,11223344556677889900AABBCCDDEEFF\r\n"));
     __delay_ms(10);
     UART_send("PC,010203040506070809000A0B0C0D0E0F,02,05\r\n", strlen("PC,010203040506070809000A0B0C0D0E0F,02,05\r\n"));
     __delay_ms(10);
     UART_send("PC,111213141516171819101A1B1C1D1E1F,18,02\r\n", strlen("PC,111213141516171819101A1B1C1D1E1F,18,02\r\n"));
     __delay_ms(10);
     
     
     UART_send("SR,00000000\r\n", strlen("SR,00000000\r\n"));//perioheral, autoadvertise, enable MLDP , UART flow control
     __delay_ms(50);
     // UART_send("SF,1\r\n", strlen("SF,1\r\n"));
     //__delay_ms(50);
    // UART_send("SN,BMS\r\n", strlen("SN,BMS\r\n"));
    //__delay_ms(50);
    //char dev_info[SETUP_BUF_LENGTH]="SS,C0000000\n\r"; //enable support of the Device Information
    
    UART_send("R,1\r\n", strlen("R,1\r\n"));
    __delay_ms(100);
    __delay_ms(6000); //give it time to Restart
    UART_send("LS\r\n", strlen("LS\r\n"));
    __delay_ms(3000);
    UART_send("A\r\n", strlen("A\r\n"));
    __delay_ms(1000);
     UART_send("SUW,010203040506070809000A0B0C0D0E0F,1234\r\n", strlen("SUW,010203040506070809000A0B0C0D0E0F,1234\r\n"));
     __delay_ms(1000);
     UART_send("SHW,001C,5678\r\n", strlen("SHW,001C,5678\r\n"));
     

  
    
   while (1)
   {
       __delay_ms(1000);    
    UART_send("Drugs\r\n", strlen("l\r\n"));
   
   }
//    UART_send("ola\n\r", strlen("ola\n\r"));
//   //}
}

//#define BRGVAL 4



/**********************************************************************
 * Name:    UART1_config
 * Args:    -
 * Return:  -
 * Desc:    Configures UART channel 1.
 **********************************************************************/
void UART1_config(void){

	/* configures LED to denote USD/UART activity*/
	TRISFbits.TRISF2 = 1;
    TRISFbits.TRISF3 = 0;
	LATFbits.LATF2 = 0;


	U1BRG = BRGVAL;
	U1MODEbits.PDSEL = 0;		/* 8-bit data, no parity */
	U1MODEbits.STSEL = 0;		/* 1 Stop-bit */
	U1MODEbits.USIDL = 0;		/* Continue operation in idle mode */
	U1MODEbits.ALTIO = 0;		/* Use U1TX and U1RX only*/
	IEC0bits.U1TXIE = 0;		/* No interrupt when transmitting */
	U1STAbits.UTXISEL = 0;		/* Interrupt when a character is transferred to the Transmit Shift register */

	IEC0bits.U1RXIE = 1;		/* Enable UART Receive interrupt */
	IPC2bits.U1RXIP = 5;		/* UART1 Receiver Interrupt Priority is 5 */
	U1STAbits.URXISEL = 0;		/* Interrupt flag bit is set when a character is received */
	IFS0bits.U1RXIF = 0;		/* clear the Rx Interrupt Flag */


	U1MODEbits.UARTEN = 1;		/* Enanble UART */
	U1STAbits.UTXEN = 1;		/* UART transmitter enabled, UxTX pin controlled by UART (if UARTEN = 1) */


	return;
}



/**********************************************************************
 * Name:    UART_send
 * Args:    char *UARTdata, int n
 * Return:  -
 * Desc:    Sends n bytes by UART. No security checks!
 **********************************************************************/
void UART_send(char *UARTdata, int n){

	int i=0;

	while(i<n){
		while(U1STAbits.UTXBF == 1); /* hold while buffer is full */
		U1TXREG = UARTdata[i];
		i++;
	}
}


/**********************************************************************
 * Assign UART1 interruption
 **********************************************************************/
void __attribute__((interrupt, auto_psv)) _U1RXInterrupt(void){
    

	UARTbuffer[UART_write] = U1RXREG;
	UART_write = (UART_write+1)%UART_BUFFER_SIZE;
    
	IFS0bits.U1RXIF = 0;		/* clear the Rx Interrupt Flag */
	return;
}

void init_BLE(){
 
    char name_int[SETUP_BUF_LENGTH] = "S-,BMS\r\n"; // Set device name to ?Central?
    char baudrate[SETUP_BUF_LENGTH] = "SB,3\r\n"; //BAUD RATE 38400
   
    char echo[SETUP_BUF_LENGTH] = "+\r\n"; //echo
    char mode[SETUP_BUF_LENGTH]="SR,32000000\r\n"; //central, enable MLPD, UART Flow Control
    
    char save_data[SETUP_BUF_LENGTH] = "SF,1\r"; //Save most of the configurable configurations in the next reboot 
    char name_public[SETUP_BUF_LENGTH] = "SN,BMS\r\n"; //name to appear in the network
    
    char dev_info[SETUP_BUF_LENGTH]="SS,C0000000\n\r"; //enable support of the Device Information
    
    char reboot[SETUP_BUF_LENGTH] = "R,1\r\n"; //restart

    
    /*
     * char ILT[SETUP_BUF_LENGTH] = "ST,0017,0002,0064\r\n";
     * char timer1_ble[SETUP_BUF_LENGTH] = "SM,1,000f4240\r\n"; // Start Timer1 to expire in 1 second CONFIRM
     */


    //UART_send(save_data, strlen(save_data));
    UART_send(reboot,strlen(reboot));
    UART_send(echo, strlen(echo));
    UART_send(save_data, strlen(save_data));
    
    UART_send(name_int, strlen(name_int));
    UART_send(baudrate, strlen(baudrate));
 
    UART_send(mode, strlen(mode));
    UART_send(name_public,strlen(name_public));
    
    
    UART_send(reboot,strlen(reboot));
    
   
}

void __attribute__((interrupt, auto_psv, shadow)) _T1Interrupt(void){
    
//    execution_time++;

	IFS0bits.T1IF = 0;		/* clears interruption flag */
    UART_send("+\r\n", strlen("+\r\n"));
	return;
}