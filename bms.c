#include <p30F4011.h>		//defines dspic registers
#include <stdio.h>          //standart IO library C
#include <libpic30.h>		//C30 compiler definitions
#include <uart.h>           //UART (serial port) function and utilities library
#include <timer.h>          //timer library
#include <string.h>
#include <math.h>
#include "funcoes_controlo_celulas.h"
#include <limits.h>

#define RX_BUFF_LENGTH 60
#define FCY 29491200L 		// 30MPI / 4
#define FOSC (FCY*4);

//Configuration settings
#pragma config FCKSMEN=CSW_FSCM_OFF
#pragma config FOS=PRI                  //fonte é o cristal
#pragma config FPR=XT_PLL16             //oscilador a 16x cristal
#pragma config WDT=WDT_OFF              //watchdog timer off
#pragma config MCLRE=MCLR_EN            //turn MCLR pin ON and
#pragma config FPWRT=PWRT_OFF


unsigned int UMODEvalue, U2STAvalue, str_pos = 0;           //auxiliary UART config variables
char RXbuffer[RX_BUFF_LENGTH];
int myVoltage = 0;
int myTemp = 0;
int temp_ref = 40;
int check_flag = 0;
int interrupt_1 = 0;
int interrupt_3 = 0;


//Função para limpar o RXbuffer
void cleanRX(){ 
                                            
	int i = 0;
	for(i = 0;i < RX_BUFF_LENGTH;i++){
		
		RXbuffer[i] = '\0';	//Inicializa cada posicao no RXbuffer.
	}      				
	
	str_pos = 0;            //Inicializa a posicao no buffer.
}


void config_timer1(){

    T1CONbits.TSIDL = 0; //continue in idle mode
    T1CONbits.TGATE = 0; //disable gated time accumulation
    T1CONbits.TCS = 0;   //use internal clock (TSYNC is ignored)
    T1CONbits.TCKPS = 1; //prescaler

    IEC0bits.T1IE = 1; //interrupt enable timer1
    IFS0bits.T1IF = 0; //clear interrupt flag

    PR1 = 10000; //valor final do timer1 (FCY/PRESCALE)
    TMR1 = 0;    //valor inicial do timer1

    T1CONbits.TON = 1; //starts timer1
}


void config_timer2(){

    T2CONbits.TSIDL = 0; //continue in idle mode
    T2CONbits.TGATE = 0; //disable gated time accumulation
    T2CONbits.TCS = 0;   //use internal clock (TSYNC is ignored)
    T2CONbits.TCKPS = 1; //prescaler

    IEC0bits.T2IE = 0; //interrupt disable timer2
    IFS0bits.T2IF = 0; //clear interrupt flag   

    PR2 = 2929; //valor final do timer2 (FCY/PRESCALE)
    TMR2 = 0;   //valor inicial do timer2

    T2CONbits.TON = 1; //starts timer2
}


void init_TMR3() {

    T3CON = 0;              // Clear the Timer 1 configuration
    T3CONbits.TCKPS = 3;    // internal Fcy divider (pre-scaler)
	
	IEC0bits.T3IE = 1;      //Enable interrupt
	IFS0bits.T3IF = 0;
	
	TMR3 = 0x0000;          // Sets timer value to zero
    PR3 = 32000;            // Timer Period

    T3CONbits.TON = 1;      // turn on timer 1
}


void config_PWM(){

    OC1CONbits.OCSIDL = 1; //disable output compare in idle
    OC1CONbits.OCTSEL = 0; //use timer2
    OC1CONbits.OCM = 6;    //pwm without fault mode
    OC1R = 0;              //valor inicial
    OC1RS = PR2;             //duty cycle

    OC2CONbits.OCSIDL = 1; //disable output compare in idle
    OC2CONbits.OCTSEL = 0; //use timer2
    OC2CONbits.OCM = 6;    //pwm without fault mode
    OC2R = 0;              //valor inicial
    OC2RS = PR2/2;         //duty cycle

    OC3CONbits.OCSIDL = 1; //disable output compare in idle
    OC3CONbits.OCTSEL = 0; //use timer2
    OC3CONbits.OCM = 6;    //pwm without fault mode
    OC3R = 0;              //valor inicial
    OC3RS = PR2/3;         //duty cycle

    OC4CONbits.OCSIDL = 1; //disable output compare in idle
    OC4CONbits.OCTSEL = 0; //use timer2
    OC4CONbits.OCM = 6;    //pwm without fault mode
    OC4R = 0;              //valor inicial
    OC4RS = PR2/4;         //duty cycle
}


void init_UART2(){
   // init_bib();

    /* Serial port config */
    UMODEvalue = UART_EN & UART_IDLE_CON & UART_NO_PAR_8BIT;                        //activates the uart in continuos mode (no sleep) and 8bit no parity mode
    U2STAvalue = UART_INT_TX & UART_TX_ENABLE & UART_INT_RX_CHAR & UART_RX_TX;      //activates interrupt of pin Tx + enables Tx + enable Rx interrupt for every char
    OpenUART2(UMODEvalue, U2STAvalue, 15);                                          //configures and activates UART2 at 115000 bps
    
    //BRG = 15 (value changed to several values)
    U2STAbits.URXISEL = 1;
    _U2RXIE = 1; //0-Interruption off, 1-Interruption on

    U2MODEbits.LPBACK = 0; //disables hardware loopback on UART2. Enable only for tests

    __C30_UART = 2; //define UART2 as predefined for use with stdio library, printf etc

    printf("\n\rSerial port ONLINE \n"); //to check if the serial port is working
}


void configure_adc() { // ADC 12-bits
    //****************************//
    //ADCON1: A/D Control Register 1
    //****************************//
    ADCON1bits.ADON = 1; //**A/D Operating Mode bit**//
    // 0-A/D converter is off
    // 1-A/D converter module is operating

    ADCON1bits.ADSIDL = 0; //**Stop in Idle Mode bit**//
    // 0-Continue module operation in Idle mode
    // 1-Discontinue module operation when device enters Idle mode

    ADCON1bits.FORM = 0; //**Sata Output Format bits**//
    // 0-Integer
    // 1-Signed integer
    // 2-Fractional
    // 3-Singed fractional

    ADCON1bits.SSRC = 7; //**Conversion Trigger Source Select bits**//
    // 0-Clearing SAMP bit ends sampling and starts conversion
    // 1-Active transition on INT0 pin ends sampling and starts conversion
    // 2-General purpose Timer3 compare ends sampling and starts conversion
    // 3-Motor Control PWM interval ends sampling and starts conversion
    // 4-Reserved
    // 5-Reserved
    // 6-Reserved
    // 7-Internal counter ends sampling and starts conversion (auto convert)

    ADCON1bits.ASAM = 0; //**A/D Sample Auto-Start bit**//
    // 0-Sampling begins when SAMP bit set
    // 1-Sampling begins immediately agter last conversion completes. SAMP bit is auto set.

    ADCON1bits.SAMP = 0; //**A/D Sample Enable bit**//
    // 0-A/D sample/hold amplifiers are holding
    // 1-At least one A/D sample/hold amplifier is sampling


    //****************************//
    //ADCON2: A/D Control Register 2
    //****************************//
    ADCON2bits.VCFG = 0; //**Voltage Reference Configuration bits**//
    // 0- AVdd        AVss
    // 1- External_Vref+_pin    AVss
    // 2- AVdd        Esternal_Vref-_pin
    // 3- External_Vref+_pin    External_Vref-_pin
    // (4-7)- AVdd        AVss

    _CSCNA = 0; //**Scan Input Selections for CH0+ S/H Input for MUX A Multiplexer Setting bit**/
    // 0-Do not scan inputs
    // 1-Scan inputs

    _BUFS = 0; //**Buffer Fill Status bit**//
    // 0-A/D is currently filling buffer 0x0-0x7, user should access data in 0x8-0xF
    // 1-A/D is currently filling buffer 0x8-0xF, user should access data in 0x0-0x7

    _SMPI = 0; //**Sample/Convert Sequences Per Interrupt Selection bits**//
    // 0-Interrupts at the completion of conversion for each sample/convert sequence
    // 1-Interrupts at the completion of conversion for each 2nd sample/convert sequence
    // ...
    // 14-Interrupts at the completion of conversion for each 15th sample/convert sequence
    // 15-Interrupts at the completion of conversion for each 16th sample/convert sequence

    _BUFM = 0; //**Buffer Mode Select bit**//
    // 0-Buffer configured as one 16-word buffer ADCBUF(15...0)
    // 1-Buffer configured as one 8-word buffer ADCBUF(15...8), ADCBUF(7...0)

    _ALTS = 0; //**Alternate Input Sample Mode Select bit**//
    // 0-Always use MUX A input multiplexer settings
    /* 1-Uses MUX A input multiplexer settings for first sample, then alternate between MUX B and
    MUX A input multiplexer settings for all subsequent samples*/



    //****************************//
    //ADCON3: A/D Control Register 3
    //****************************//
    _SAMC = 31; //**Auto Sample Time bits**//
    // (0-31) Tad

    _ADRC = 0; //**A/D Conversion Clock Source bit**//
    // 0-Clock derived from system clock
    // 1-A/D internal RC clock

    _ADCS = 63; //**A/D Conversion Clock Select bits**//
    // (1-64)*Tcy/2

    // Usou-se Tad=1000ns, logo obtem-se ADCS=14


    //******************************//
    //ADCHS: A/D Input Select Register
    //******************************//
    _CH0NB = 0; //**Channel 0 Negative Input Select

    _CH0SB = 0;

    _CH0NA = 0;

    _CH0SA = 0;



    //*************************************//
    //ADPCFG: A/D Port Configuration Register
    //*************************************//

    // Este registo e configurado na funçao configure_adc_channel //



    //*************************************//
    //ADCSSL: A/D Input Scan Select Register
    //*************************************//

    // Este registo nao precisa ser configurado, pois o CSCNA é 0 //
    
    //TRISB= 0b000000000;
    //ADPCFG= 0b111111111;
}


void configure_adc_channel(int channel){
	
    TRISB |= (1 << channel);            // ADC_CHANNEL defined as input
    ADPCFG &= ~(1 << channel);          // ADC_CHANNEL defined as analog
}


int read_adc(int channel){
    
	int x;
	
    _CH0SA = channel;           //CH0SA = channel vai ler o AN(Channel), caso seja 0 lê o valor do AN0
    _SAMP = 1;
	
    while (_SAMP);
    while (!_DONE);
	
    x = ADCBUF0;

    return x;
}


//Função que faz a análise da tensão
/*
void analise_tensao(int vec[4]){
    
    int tensao_total = 0;
    int tensao_med = 0;
    int tensao_min = dados[0].tensao;
    int k,l,m;
    
    for(k=1;k<4;k++){
        if(dados[k].tensao < tensao_min){
            tensao_min = dados[k].tensao;
        }
    }
    
    for(l=0;l<4;l++){
        tensao_total = tensao_total + dados[l].tensao;
    }

    tensao_med = tensao_total/4;

    for(m=0;m<4;m++){
       
        if(dados[m].tensao > tensao_med){
            
            while(dados[m].tensao > tensao_min){
                vec[m] = 1;
            }
        }
    }
    return;
}
*/


//Função que converte o valor da temperatura de analógico para digital
int get_temp(unsigned int ADCvalue){

	float R = 0;
	float Vo;
	float Vin = 4.95;
	float Ro = 10000;
	float To = 298.15;
	float B = 3435;
	float T;

	/*
	 * Note: enable and switching time can take up to ~20ns which is faster than the clock frequency.
	 * Still, consider adding a delay here if measurements are not consistent.
	 */


	/*
	 * Note: Acquisition and calculations may take too long to be worthwhile.
	 * Measure/compare gains and use table if needed.
	 */

	
	/*New Code for temperature Calculation*/
    Vo = Vin*(((float)ADCvalue)/1024);              
	R =  Ro/((Vin - Vo)/(float)Vo);
	T = (float)(B*To)/(float)(To*log(R/Ro)+B);
        	
    return (int)(((T - 273.15))*10);                //Valor da temperatura em graus celcius
}


//Função que verifica se alguma célula está acima da temperatura de referência
int analise_temp(){
    
    int temp_max = 0;
    int j = 0;
    
    for(j=0;j<4;j++){

        if(dados[j].temp >= temp_max){
            temp_max = dados[j].temp;
        }
    }
	
    if (temp_max > temp_ref){
        //SHUT DOWN
        _LATB0 = 1;   //abrir mosfet (deixa de passar corrente)
        check_flag = 1;
    }
	
    return check_flag;
}


int main(){
    
    cleanRX();
    init_UART2();
    
    init_TMR3();                            //inicia a funçao do timer e cada vez que chega a 3200 vai ao interrupt e mete a flag a 0
    configure_adc();
    config_timer1();
    config_timer2();
    config_PWM();    
    
	int k = 0;
    int pl = 0;
    int temp = 0;
    int abrir_celulas[4];
    int aux = 0;

    for(aux = 0;aux<4;aux++){
        
		abrir_celulas[aux] = 0;
    }
    
    //while(1){
		
        //if (interrupt_3){
            interrupt_3=0;
			
            for (k=0;k<4;k++){
                
				configure_adc_channel(pl);
                configure_adc_channel(pl+1);

                //myVoltage = get_voltage(read_adc(pl));
                temp = read_adc(pl+1);
                
                myTemp = get_temp(temp);
                
                dados[k].tensao = read_adc(pl);
                dados[k].temp = myTemp;
                //printf("\n\rleu:\t\t %d\t %d", dados[k].tensao, dados[k].temp);

                //printf("\n\rasdas: %d\t %d\n",pl, pl+1);
                printf("\n\restrutura:\t %d\t %d\n", dados[k].tensao, dados[k].temp);
                
                pl = pl+2;
            }
            
            while(1){
				
				analise_temp();         //compara o valor maximo da temperatura com o valor obtido e abre ou fecha o mosfet geral
    
				printf("Check flag: %d\n", check_flag);
				//check_flag=0;
            
            
				if(abrir_celulas[0] == 0){
					OC1RS = 0;
				}
            
			//analise_tensao(abrir_celulas);
            }      
        //}
    //}
    
    return 0;
}


/****************************************************
 ************     Interruptions   *******************
 ****************************************************/

/* This is UART2 receive ISR */
// UART Interruption handler
void __attribute__((__interrupt__,auto_psv)) _U2RXInterrupt(void){
	
	IFS1bits.U2RXIF = 0;  //resets and reenables the Rx2 interrupt flag

	//Read the receive buffer until at least one or more character can be read
	while(U2STAbits.URXDA){
		
		RXbuffer[str_pos] = U2RXREG;  //stores the last received char in the buffer
		//printf("%c", RXbuffer[str_pos]);  //prints the last received char
		str_pos++;                        	//increments the position in the buffer to store the next char
		
		if(str_pos >= 80){
			
			str_pos = 0;	//if the last position is reached then return to initial position
			}    
    }
}


//Timer 1 Interrupt handler
void __attribute__((interrupt, auto_psv, shadow)) _T1Interrupt(void){

    IFS0bits.T1IF = 0; //clear interrupt flag
    interrupt_1 = 1;
}


//Timer 3 Interrupt handler
void __attribute__((interrupt, no_auto_psv)) _T3Interrupt(void) {
       
    IFS0bits.T3IF = 0; //Clears interrupt flag
	interrupt_3 = 1;
}
