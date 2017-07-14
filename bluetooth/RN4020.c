#define FCY 29641200L               //number of instructions per milisec
#define FOSC (FCY*4)                //number of clock cycles
#define UART_BUFFER_SIZE 256
#define BAUDRATE 115200
#define BRGVAL ((FCY/BAUDRATE)/16)-1

#pragma config FCKSMEN=CSW_FSCM_OFF
#pragma config FOS=PRI //fonte cristal
#pragma config FPR=XT_PLL16 //oscilador a 16x cristal
#pragma config WDT=WDT_OFF // watchdog timer off 
#pragma config MCLRE=MCLR_EN // Turn MCLR pin ON and
#pragma config FPWRT=PWRT_OFF

#include <libpic30.h> //C30 compiler definitions
#include <p30F4011.h> // Defines os dspic registers
#include <uart.h>
#include <string.h>
#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#include "UART.h"
#include "CAN.h"

volatile unsigned int UART_write = 0;
volatile unsigned int UART_read = 0;
volatile unsigned char UARTbuffer[UART_BUFFER_SIZE];

void UART1_config(void);
void UART_send(char *UARTdata, int n);
void __attribute__((interrupt, auto_psv)) _U1RXInterrupt(void);

/*MAIN*/
int main (void){
    
    RCONbits.SWDTEN=1;
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
    while (1){
        UART_send(st, strlen(st));
    }
    
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
