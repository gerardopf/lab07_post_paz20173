/*
 * lab07_post
 * File:   main.c
 * Author: Gerardo Paz - 20173
 * Contador en puerto A con botones en puerto B
 * Display 7 segmentos en puerto C
 * Selectores en puerto D
 * Created on April 5, 2022, 8:52 PM
 */

#pragma config FOSC = INTRC_NOCLKOUT// Oscillator Selection bits (INTOSCIO oscillator: I/O function on RA6/OSC2/CLKOUT pin, I/O function on RA7/OSC1/CLKIN)
#pragma config WDTE = OFF       // Watchdog Timer Enable bit (WDT disabled and can be enabled by SWDTEN bit of the WDTCON register)
#pragma config PWRTE = OFF      // Power-up Timer Enable bit (PWRT disabled)
#pragma config MCLRE = OFF      // RE3/MCLR pin function select bit (RE3/MCLR pin function is digital input, MCLR internally tied to VDD)
#pragma config CP = OFF         // Code Protection bit (Program memory code protection is disabled)
#pragma config CPD = OFF        // Data Code Protection bit (Data memory code protection is disabled)
#pragma config BOREN = OFF      // Brown Out Reset Selection bits (BOR disabled)
#pragma config IESO = OFF       // Internal External Switchover bit (Internal/External Switchover mode is disabled)
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enabled bit (Fail-Safe Clock Monitor is disabled)
#pragma config LVP = OFF        // Low Voltage Programming Enable bit (RB3 pin has digital I/O, HV on MCLR must be used for programming)
// CONFIG2
#pragma config BOR4V = BOR40V   // Brown-out Reset Selection bit (Brown-out Reset set to 4.0V)
#pragma config WRT = OFF        // Flash Program Memory Self Write Enable bits (Write protection off)
// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

#include <xc.h>         // registros del PIC
#include <stdio.h>
#include <stdlib.h>

/*------------ CONSTANTES -----------------*/

#define _tmr0_value 236   // TMR0 value
#define _XTAL_FREQ 4000000  // oscilador 

/*------------- VARIABLES GLOBALES ------------------*/

uint8_t contador = 0, cod = 0, selector = 0;
uint8_t vector[3] = {0,0,0};

/*--------------- FUNCIONES DE INTERRUPCIONES -----------------*/
void codificar_7seg(uint8_t valor){
    switch(valor){
        case 0:
            PORTC = 0b00111111;
            break;
        case 1:
            PORTC = 0b00000110;
            break;
        case 2:
            PORTC = 0b01011011;
            break;
        case 3:
            PORTC = 0b01001111;
            break;   
        case 4:
            PORTC = 0b01100110;
            break;
        case 5:
            PORTC = 0b01101101;
            break;
        case 6:
            PORTC = 0b01111101;
            break;
        case 7:
            PORTC = 0b00000111;
            break;   
        case 8:
            PORTC = 0b01111111;
            break;
        case 9:
            PORTC = 0b01101111;
            break;
        default:
            PORTC = 0b00000000;
            break;
    }
    return;
}

void multiplexado(){
    if(selector < 4)
    {
        if(selector == 0)
            selector++;         // siempre tiene que estar encendido un bit
        else
            selector *= 2;      // encender bits de uno en uno 
    }
    else
        selector = 1;       // al hacer overflow de los selectores, se coloca en 1

    switch(selector){
        case 1:{
            codificar_7seg(vector[0]);  //mostrar valor
            PORTD = selector;
            break;
        }
        case 2:{
            codificar_7seg(vector[1]);  //mostrar valor
            PORTD = selector;
            break;
        }
        case 4:{
            codificar_7seg(vector[2]);  //mostrar valor
            PORTD = selector;
            break;
        }
        default:
            break;
    }      
    return;
}

void __interrupt() isr (void){
    if (RBIF){
        if(PORTBbits.RB0){      // ver qué pin ocasionó la interrupción
            contador++;
        }
        else if(PORTBbits.RB1){      // ver qué pin ocasionó la interrupción
            contador--;
        }
        RBIF = 0; // limpiar bandera
    }
    if(T0IF){
        TMR0 = _tmr0_value;
        PORTD = 0;          // siempre hay que limpiar el puerto del selector para que no haya prolemas
        multiplexado();     // comienza el multiplexado
        T0IF = 0;           // limpiar bandera
    }   
    return;
}

/*---------------- FUNCIONES PRINCIPALES ---------------*/

void timer0(void){
    // Timer0
    //OPTION_REG = 0b00000111;     // Timer0 con prescaler de 256
    OPTION_REGbits.T0CS = 0;    // temporizador
    OPTION_REGbits.PSA = 0;     // asignar prescaler
    OPTION_REGbits.PS2 = 1;
    OPTION_REGbits.PS1 = 1;
    OPTION_REGbits.PS0 = 1;     // ps 1:256
    INTCONbits.T0IF = 0;
    TMR0 = _tmr0_value;
    return;
}

void setup_int(void){
    INTCON = 0b10101000; // globales, onchange y bandera limpia de RB y Timer0
    IOCB =  0b00000011;  // onchange de RB0 y RB1
    return;
}

void setup(void){
    ANSEL = 0;  
    ANSELH = 0;     // entradas digitales
    
    TRISA = 0;  // PORTA out
    TRISC = 0;
    TRISD = 0;
    TRISE = 0;
    TRISB = 0b00000011; // RB0 in
    
    PORTA = 0;
    PORTC = 0;
    PORTE = 0;
    PORTD = 0b00000001;
    PORTB = 0;  // puertos limpios
    return;
}

void clk(void){
    OSCCONbits.IRCF = 0b0110;   // Tiempo
    OSCCONbits.SCS = 1;         // oscilador interno
    return;
}

void separar(uint8_t valor, uint8_t* vector){
    uint8_t temp = 0;
    
    temp = valor;
    vector[2] = temp/100;
    temp-= vector[2]*100;
    vector[1] = temp/10;
    temp-= vector[1]*10;
    vector[0] = temp; 
    return;
}

/*------------ CÓDIGO PRINCIPAL ---------------*/
void main(void){
    
    setup();
    setup_int();
    clk();
    timer0();
    while(1){   // principal loop
        separar(contador, vector);
        PORTA = contador;           // mostrar contador en puerto A siempre
    }
}