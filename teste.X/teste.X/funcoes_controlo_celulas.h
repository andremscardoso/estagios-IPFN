#ifndef __FUNCOES_CONTROLO_CELULAS_H__
#define __FUNCOES_CONTROLO_CELULAS_H__

#define SIZE_TEMP 2
#define SIZE_TENSAO 4


float tensao[4];        //tensao

float temp[2];

float curr;

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

float get_temp(unsigned int ADCvalue);

float analise_temp();

void send2BLE();

float get_curr(int ADCvalue);

#endif