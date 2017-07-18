#ifndef __FUNCOES_CONTROLO_CELULAS_H__
#define __FUNCOES_CONTROLO_CELULAS_H__

typedef struct Caracteristicas{
    int temp;
    int tensao;
    //int corr;
}caracteristicas;

caracteristicas dados[4];

void cleanRX();

void config_timer1();

void config_timer2();

void init_TMR3();

void config_PWM();

void init_UART2();

void configure_adc();

void configure_adc_channel(int channel);

int read_adc(int channel);

void analise_tensao(int vec[4]);

int get_temp(unsigned int ADCvalue);

int analise_temp();

#endif