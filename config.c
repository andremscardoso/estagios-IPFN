#define FCY 29491200L       //number of instructions per milisecond
#define FOSC (FCY*4)        //number of clock cycles

#pragma config FCKSMEN=CSW_FSCM_OFF 
#pragma config FOS=PRI      //fonte é o cristal
#pragma config FPR=XT_PLL16 //oscilador a 16x cristal
#pragma config WDT=WDT_OFF  //watchdog timer off
#pragma config MCLRE=MCLR_EN//Turn MCLR pin ON and 
#pragma config FPWRT=PWRT_OFF

#include <libpic30.h>       //C30 compiler definitions
#include <p30F4011.h>       //defines os dspic registers

void config_timer2(){
    
    T2CONbits.TSIDL = 0; //continue in idle mode
    T2CONbits.TGATE = 0; //disable gated time accumulation
    T2CONbits.TCS = 0;   //use internal clock (TSYNC is ignored)
    T2CONbits.TCKPS = 1; //prescaler
    
    IEC0bits.T2IE = 0; //interrupt enable timer1
    IFS0bits.T2IF = 0; //clear interrupt flag   

    PR2 = 2929; //valor final do timer 1 (FCY/PRESCALE)
    TMR2 = 0;   //valor inicial do timer1

    T2CONbits.TON = 1; //starts timer2
}

void config_PWM(){
    
    OC1CONbits.OCSIDL = 1; //disable output compare in idle
    OC1CONbits.OCTSEL = 0; //use timer2
    OC1CONbits.OCM = 6;    //pwm without fault mode
    OC1R = 0;              //valor inicial
    OC1RS = PR2;           //duty cycle

    OC2CONbits.OCSIDL = 1; //disable output compare in idle
    OC2CONbits.OCTSEL = 0; //use timer2
    OC2CONbits.OCM = 6;    //pwm without fault mode
    OC2R = 0;              //valor inicial
    OC2RS = PR2/2;          //duty cycle
    
    OC3CONbits.OCSIDL = 1; //disable output compare in idle
    OC3CONbits.OCTSEL = 0; //use timer2
    OC3CONbits.OCM = 6;    //pwm without fault mode
    OC3R = 0;              //valor inicial
    OC3RS = PR2/3;          //duty cycle
    
    OC4CONbits.OCSIDL = 1; //disable output compare in idle
    OC4CONbits.OCTSEL = 0; //use timer2
    OC4CONbits.OCM = 6;    //pwm without fault mode
    OC4R = 0;              //valor inicial
    OC4RS = PR2/4;         //duty cycle
}

int main(){
    
    config_timer2();
    config_PWM();
    
    while(1){
       
        /*for(a=0;a<PR2;a++){
            
            OC1RS = a;
            __delay_ms(1);
        }*/ //colocar o duty cycle a variar
    }
   
    return 0;
}
