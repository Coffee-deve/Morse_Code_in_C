/**
 * @file main.c
 * @author Patrycja Nazim
 * @date January 2024
 * @brief File containing the main function.
 * @ver 0.1
 */

#include "MKL05Z4.h"
#include "frdm_bsp.h"
#include "UART_COMMUNICATION.h"
#include <stdio.h>
#include <string.h>

#define RED_LED_POS 8 
#define BUTTON_POS 3
#define BUZZER_POS 13
#define BUTTON_POS 3
#define BUTTON2_POS 4
#define MAX_MORSE_LENGTH 10
#define MAX_LEN 100  // Maximum input size

volatile char received_char = '\0';
volatile int char_received_flag = 0;

char rx_sentence[MAX_LEN];  // Buffer for storing user input
uint8_t rx_index = 0;  // Index in buffer
uint8_t rx_READY = 0;  // Flag when sentence is ready


// Morse code table
const char* morse[27] = {
    ".-",        // A
    "-...",      // B
    "-.-.",      // C
    "-..",       // D
    ".",         // E
    "..-.",      // F
    "--.",       // G
    "....",      // H
    "..",        // I
    ".---",      // J
    "-.-",       // K
    ".-..",      // L
    "--",        // M
    "-.",        // N
    "---",       // O
    ".--.",      // P
    "--.-",      // Q
    ".-.",       // R
    "...",       // S
    "-",         // T
    "..-",       // U
    "...-",      // V
    ".--",       // W
    "-..-",      // X
    "-.--",      // Y
    "--..",      // Z
    " "          // SPACE
};

const char alphabet[27] = {
    65,  // 'A'
    66,  // 'B'
    67,  // 'C'
    68,  // 'D'
    69,  // 'E'
    70,  // 'F'
    71,  // 'G'
    72,  // 'H'
    73,  // 'I'
    74,  // 'J'
    75,  // 'K'
    76,  // 'L'
    77,  // 'M'
    78,  // 'N'
    79,  // 'O'
    80,  // 'P'
    81,  // 'Q'
    82,  // 'R'
    83,  // 'S'
    84,  // 'T'
    85,  // 'U'
    86,  // 'V'
    87,  // 'W'
    88,  // 'X'
    89,  // 'Y'
    90,  // 'Z'
    32   // ' ' (space, ASCII value 32)
};

const char* alphabet_table[27] = {
    "A", "B", "C", "D", "E", "F", "G", "H", "I", "J",
    "K", "L", "M", "N", "O", "P", "Q", "R", "S", "T",
    "U", "V", "W", "X", "Y", "Z", " "
};



/**
 * @brief Delay function for simple timing.
 * @param n Number of milliseconds to delay.
 */
void delay_ms(int n) {
    volatile int i, j;
    for (i = 0; i < n; i++)
        for (j = 0; j < 3500; j++) {}
}

/**
 * @brief The interupt function
 *
 * @return NULL
 */
 
/**
 * @brief Send a string via UART0.
 * @param str The string to send.
 */
void UART0_SendString(const char* str) {
    while (*str) {
        while (!(UART0->S1 & UART0_S1_TDRE_MASK));  // Wait until TX is ready
        UART0->D = *str++;                          // Send the character
    }
}
 
void PORTB_IRQHandler(void) {
        if (PORTB->ISFR & (1 << BUTTON_POS)) {  // Check if interrupt is from Button Pin
					delay_ms(20); // for debouncing 
        if ((FPTB->PDIR & (1 << BUTTON_POS)) == 0) {  // Button is pressed
	//				  code <<= 1; // Shift left
					  UART0_SendString("-\r\n"); // Send "-" to Termite
            PTB->PCOR = (1 << RED_LED_POS);  // Turn ON the LED (active low)
				  	PTB->PSOR = (1 << BUZZER_POS);  // Turn ON Buzzer (Button Released)

        } else {  // Button is released
						delay_ms(300);
            PTB->PSOR = (1 << RED_LED_POS);  // Turn OFF the LED (active low)
					  PTB->PCOR = (1 << BUZZER_POS);  // Turn OFF Buzzer (Button Pressed)
        }
        PORTB->ISFR = (1 << BUTTON_POS);  // Clear the interrupt flag
    }else if (PORTB->ISFR & (1 << BUTTON2_POS)){
			delay_ms(20); // for debouncing 
			if ((FPTB->PDIR & (1 << BUTTON2_POS)) == 0) {  // Button is pressed
		//		    code = (code << 1) | 1; // Shift left and add 1
				    UART0_SendString(".\r\n"); // Send "-" to Termite
            PTB->PCOR = (1 << RED_LED_POS);  // Turn ON the LED (active low)
				  	PTB->PSOR = (1 << BUZZER_POS);  // Turn OFF Buzzer (Button Released)
							
        } else {  // Button is released
					 delay_ms(100);
            PTB->PSOR = (1 << RED_LED_POS);  // Turn OFF the LED (active low)
					  PTB->PCOR = (1 << BUZZER_POS);  // Turn ON Buzzer (Button Pressed)
        }
				PORTB->ISFR = (1 << BUTTON2_POS);  // Clear the interrupt flag
}
}


/**
 * @brief UART0 interrupt handler. Reads the received character and sets a flag.
 */
void UART0_IRQHandler() {
    if (UART0->S1 & UART0_S1_RDRF_MASK) {  // Check if data is received
        received_char = UART0->D;          // Read the received character
			
			
			
			
        char_received_flag = 1;           // Set the flag
    }
}



/**
 * @brief Main function.
 */
int main(void) {
	    __enable_irq();  // Enable global interrupts
    // Enable UART0 and configure it
    UART0_Init();
	
	// Configure PORTB Pin 13 for Buzzer
    SIM->SCGC5 |= SIM_SCGC5_PORTB_MASK;       // Enable clock for GPIO B
    PORTB->PCR[BUZZER_POS] |= PORT_PCR_MUX(1); // Set Pin 9 as GPIO
    PTB->PDDR |= (1 << BUZZER_POS);           // Set Pin 9 as output
    PTB->PCOR = (1 << BUZZER_POS);            // Ensure Buzzer is OFF initially
	

    // configure portb pin 8 as red led output
    SIM->SCGC5 |= SIM_SCGC5_PORTB_MASK;        // Enable clock for PORTB
    PORTB->PCR[RED_LED_POS] = PORT_PCR_MUX(1); // Set pin as GPIO
    PTB->PDDR |= (1 << RED_LED_POS);           // Set pin as output
    PTB->PSOR = (1 << RED_LED_POS);            // Turn off LED

		// configure portb pin 3 as button input

 	PORTB->PCR[BUTTON_POS] |= PORT_PCR_MUX(1)| PORT_PCR_PE_MASK | PORT_PCR_PS_MASK | PORT_PCR_IRQC(0xB);  /* MUX config. Set Pin 13 of PORT B as GPIO, interrupt on falling edge */  	 
  PTB->PDDR &= ~(1UL << BUTTON_POS); /* Set pin 13 of GPIO B as input */ 
	NVIC_ClearPendingIRQ(31);
	NVIC_EnableIRQ(31);
  NVIC_SetPriority(31, 0);
//	*/	// configure portb pin 4 as button input

	PORTB->PCR[BUTTON2_POS] |= PORT_PCR_MUX(1)| PORT_PCR_PE_MASK | PORT_PCR_PS_MASK | PORT_PCR_IRQC(0xB);  /* MUX config. Set Pin 13 of PORT B as GPIO, interrupt on falling edge */  	 
	PTB->PDDR &= ~(1UL << BUTTON2_POS); /* Set pin 13 of GPIO B as input */ 


    // Infinite loop
    while (1) {
				
        if (char_received_flag) {  
            char_received_flag = 0;

            // Convert to uppercase
            if (received_char >= 'a' && received_char <= 'z') {
                received_char -= 32;  // Convert to uppercase
            }

            // Check if it's a valid letter or space
            if ((received_char >= 'A' && received_char <= 'Z') || received_char == ' ') {
                uint8_t index = (received_char == ' ') ? 26 : (received_char - 65); // index is either space or index of lookup table for given letter (65 is first index, A letter)
                const char* morse_code = morse[index]; // takes value from lookup table
							
							                PTB->PCOR = (1 << RED_LED_POS);  // Turn on LED
							 // Send the Morse code to UART
														  UART0->D = received_char;
                UART0_SendString("\r\nMorse: ");
                UART0_SendString(morse_code);
                UART0_SendString("\r\n");

					  for (int i = 0; morse_code[i] != '\0'; i++) {
                if (morse_code[i] == '.') {
                    PTB->PSOR |= (1 << BUZZER_POS); // Buzzer ON
                    delay_ms(200);  // Short beep for dot
                } 
                else if (morse_code[i] == '-') {
                    PTB->PSOR |= (1 << BUZZER_POS); // Buzzer ON
                    delay_ms(600);  // Long beep for dash
                }

                PTB->PCOR |= (1 << BUZZER_POS); // Buzzer OFF
                delay_ms(200);  // Short pause between symbols
            }
						                PTB->PSOR = (1 << RED_LED_POS);  // Turn off LED
							delay_ms(600);
							
         
						 

                 

            }
        } 
    }
	}