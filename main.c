/**
 * @file main.c
 * @author Patrycja Nazim
 * @date January 2024
 * @brief File containing the main function.
 * @ver 0.1
 */
#include "MKL05Z4.h"
#include "frdm_bsp.h"
#include "led.h"
#include "lcd1602.h"
#define RED_LED_POS 8 
#define BUTTON_POS 9
/**
 * @brief The main loop.
 *
 * @return NULL
 */
int main (void) {
	// configure portb pin 8 as red led output
	SIM->SCGC5 |= SIM_SCGC5_PORTB_MASK; /* Enable clock for GPIO B */ 
 	PORTB->PCR[RED_LED_POS] |= PORT_PCR_MUX(1);  /* MUX config. Set Pin 8 of PORT B as GPIO */  	 
 	PTB->PDDR |= (1 << RED_LED_POS); /* Set pin 8 of GPIO B as output */ 
	PTB->PSOR |= (1 << RED_LED_POS); /* Turn off RED LED */ // led is active low 

	// configure porta pin 9 as button input
	SIM->SCGC5 |= SIM_SCGC5_PORTA_MASK; /* Enable clock for GPIO A */ 
 	PORTA->PCR[BUTTON_POS] |= PORT_PCR_MUX(1)| PORT_PCR_PE_MASK | PORT_PCR_PS_MASK;  /* MUX config. Set Pin 9 of PORT A as GPIO */  	 
 	PTA->PDDR &= ~(1UL << BUTTON_POS); /* Set pin 9 of GPIO A as input */ 
	
 LED_Init (); /* initialize all LEDs */
 LCD1602_Init(); /* initialize LCD */
 LCD1602_Backlight(TRUE);

 LCD1602_SetCursor(1,0);
 LCD1602_Print("MORSE CODE");
 LCD1602_SetCursor(3,1);
 LCD1602_Print("DECODER");

 DELAY(2000)
	
 LCD1602_ClearAll();
 LCD1602_SetCursor(0,0);
 LCD1602_Print("START ENCODING:");

 while(1){
			if( ( PTA->PDIR & (1<<BUTTON_POS) ) ==0 ){ /* Test if button pressed */
			PTB->PTOR|=(1<<RED_LED_POS); /* Toggle RED LED */
			while( ( PTA->PDIR & (1<<BUTTON_POS) ) == 0 ) /* Wait for release */
			DELAY(100); /* Debouncing */
 }
			}
};