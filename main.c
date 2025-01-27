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
#include "lcd1602.h"  
#include "UART_COMMUNICATION.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#define RED_LED_POS 8 
#define BUTTON_POS 3
#define BUTTON2_POS 4
#define BUTTON6_POS 6
#define BUZZER_POS 13
#define BUTTON_DOWN 0
#define BUTTON_UP 1

char rx_buf[]={0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20};
char temp;
uint8_t rx_FULL=0;
int index;

// code = 0b0000000000100000;

/*
 * @brief delay function
 *
 * @return NULL
 */


void delay_ms( int n) {
volatile int i;
volatile int j;
for( i = 0 ; i < n; i++)
for(j = 0; j < 3500; j++) {}
}



/*
 * @brief The look up tables
 *
 * @return NULL
 */


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



const int morse_table[27] = {
    0b1000001,  // A -> .-       (01)
    0b100001000,  // B -> -...     (1000)
    0b10000110,  // C -> -.-.     (1010)
    0b10000100,   // D -> -..      (100)
    0b1000000,      // E -> .        (0)
    0b00100010,  // F -> ..-.     (0010)
    0b11100,   // G -> --.      (110)
    0b1000000000,   // H -> ....     (0000)
    0b10000000,     // I -> ..       (00)
    0b1000011,  // J -> .---     (0111)       
    0b1000011,    // K -> -.-      (101)
    0b100000100,  // L -> .-..     (0100)
    0b100010,     // M -> --       (11)
    0b1000010,     // N -> -.       (10)
    0b100011,    // O -> ---      (111)
    0b10000100,  // P -> .--.     (0110)
    0b1000101,  // Q -> --.-     (1101)
    0b10000010,    // R -> .-.      (010)
    0b100000000,    // S -> ...      (000)
    0b1000001,      // T -> -        (1)
    0b10000001,    // U -> ..-      (001)
    0b100000001,   // V -> ...-     (0001)
    0b1000010,    // W -> .--      (011)
    0b10000101,   // X -> -..-     (1001)
    0b1000100,   // Y -> -.--     (1011)
    0b10001000,   // Z -> --..     (1100)
		0b1000010000   // SPACE -> -.... (10000)
};


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




const char* alphabet_table[27] = {
    "A", "B", "C", "D", "E", "F", "G", "H", "I", "J",
    "K", "L", "M", "N", "O", "P", "Q", "R", "S", "T",
    "U", "V", "W", "X", "Y", "Z", " "
};


/**
 * @brief Translates Morse code in `morse_buffer` to a character using `morse_lookup`.
 * @return The corresponding letter or '?' if the code is not valid.
 */
 




// interrupt function for UART0



void UART0_IRQHandler()
{
	if(UART0->S1 & UART0_S1_RDRF_MASK) // if flag is set
	{
		temp=UART0->D;	// Read data into temp
		 if (temp >= 'A' && temp <= 'Z')
        {
            // Get the index for the letter (A = 0, B = 1, ..., Z = 25)
            uint8_t index = temp - 'A';

            // Send the corresponding Morse code for the letter
            const char* morse_code = morse[index];

            // Send the Morse code via UART
            while (*morse_code)
            {
                // Wait until UART TX is ready
                while (!(UART0->S1 & UART0_S1_TDRE_MASK));
                UART0->D = *morse_code++;  // Send the next character in Morse code
            }

            // Optionally, send a newline after the Morse code
            while (!(UART0->S1 & UART0_S1_TDRE_MASK));
            UART0->D = '\n';  // Send a newline after the code
        }

        rx_FULL = 1;  // Data received
    }
	}








/**
 * @brief The interupt function
 *
 * @return NULL
 */
 
 
void PORTB_IRQHandler(void) {
        if (PORTB->ISFR & (1 << BUTTON_POS)) {  // Check if interrupt is from Button Pin
					delay_ms(20); // for debouncing 
        if ((FPTB->PDIR & (1 << BUTTON_POS)) == 0) {  // Button is pressed

            PTB->PCOR = (1 << RED_LED_POS);  // Turn ON the LED (active low)
				  	PTB->PSOR = (1 << BUZZER_POS);  // Turn OFF Buzzer (Button Released)
	//				LCD1602_SetCursor(0,0);
		//				code = code<<1;
        } else {  // Button is released
					
            PTB->PSOR = (1 << RED_LED_POS);  // Turn OFF the LED (active low)
					  PTB->PCOR = (1 << BUZZER_POS);  // Turn ON Buzzer (Button Pressed)
			//	    LCD1602_Print(".");
        }
        PORTB->ISFR = (1 << BUTTON_POS);  // Clear the interrupt flag
    }else if (PORTB->ISFR & (1 << BUTTON2_POS)){
			delay_ms(20); // for debouncing 
			if ((FPTB->PDIR & (1 << BUTTON2_POS)) == 0) {  // Button is pressed
            PTB->PCOR = (1 << RED_LED_POS);  // Turn ON the LED (active low)
				  	PTB->PSOR = (1 << BUZZER_POS);  // Turn OFF Buzzer (Button Released)
							
	//					LCD1602_SetCursor(0,0);
	//					code += code;
        } else {  // Button is released
					 
            PTB->PSOR = (1 << RED_LED_POS);  // Turn OFF the LED (active low)
					  PTB->PCOR = (1 << BUZZER_POS);  // Turn ON Buzzer (Button Pressed)
				//    LCD1602_Print("-");
        }
				PORTB->ISFR = (1 << BUTTON2_POS);  // Clear the interrupt flag
	//	}else if (PORTB->ISFR & (1 << BUTTON6_POS)){
//			if ((FPTB->PDIR & (1 << BUTTON6_POS)) == 0) { 				// Button is pressed
	//					delay_ms(100);
	//			for (int x = 0; x < 28; x++){
		//			if(morse_table[x] == code){
			//		int	index = x;
			//		LCD1602_SetCursor(0,1);
			//		LCD1602_Print(alphabet_table[index]);
	//				} 		
	//			}
 //       } else {         // Button is released
//		uint16_t code = 0b0000000000100000;	
	//			}
	//			PORTB->ISFR = (1 << BUTTON6_POS);  // Clear the interrupt flag
	//	}
}
}

/**
 * @brief The main loop.
 *
 * @return NULL
 */

int main (void) {

	__enable_irq();
	    // Configure PORTB Pin 9 for Buzzer
    SIM->SCGC5 |= SIM_SCGC5_PORTB_MASK;       // Enable clock for GPIO B
    PORTB->PCR[BUZZER_POS] |= PORT_PCR_MUX(1); // Set Pin 9 as GPIO
    PTB->PDDR |= (1 << BUZZER_POS);           // Set Pin 9 as output
    PTB->PCOR = (1 << BUZZER_POS);            // Ensure Buzzer is OFF initially
	
	
	// configure portb pin 8 as red led output
	
SIM->SCGC5 |= SIM_SCGC5_PORTB_MASK; /* Enable clock for GPIO B */ 
	PORTB->PCR[RED_LED_POS] |= PORT_PCR_MUX(1);  /* MUX config. Set Pin 8 of PORT B as GPIO */  	 
	PTB->PDDR |= (1 << RED_LED_POS); /* Set pin 8 of GPIO B as output */ 
	PTB->PSOR |= (1 << RED_LED_POS); /* Turn off RED LED */ // led is active low 

	// configure portb pin 1 as button input

 	PORTB->PCR[BUTTON_POS] |= PORT_PCR_MUX(1)| PORT_PCR_PE_MASK | PORT_PCR_PS_MASK | PORT_PCR_IRQC(0xB);  /* MUX config. Set Pin 13 of PORT B as GPIO, interrupt on falling edge */  	 
  PTB->PDDR &= ~(1UL << BUTTON_POS); /* Set pin 13 of GPIO B as input */ 
	NVIC_ClearPendingIRQ(31);
	NVIC_EnableIRQ(31);
  NVIC_SetPriority(31, 0);
//	*/	// configure portb pin 2 as button input

	PORTB->PCR[BUTTON2_POS] |= PORT_PCR_MUX(1)| PORT_PCR_PE_MASK | PORT_PCR_PS_MASK | PORT_PCR_IRQC(0xB);  /* MUX config. Set Pin 13 of PORT B as GPIO, interrupt on falling edge */  	 
	PTB->PDDR &= ~(1UL << BUTTON2_POS); /* Set pin 13 of GPIO B as input */ 

	// configure portb pin 5 as button input

// 	PORTB->PCR[BUTTON6_POS] |= PORT_PCR_MUX(1)| PORT_PCR_PE_MASK | PORT_PCR_PS_MASK | PORT_PCR_IRQC(0xB);  /* MUX config. Set Pin 13 of PORT B as GPIO, interrupt on falling edge */  	 
//	PTB->PDDR &= ~(1UL << BUTTON6_POS); /* Set pin 13 of GPIO B as input */ 

	
	UART0_Init();		// Init UART0

/*
		// initialization of TPM1 on porta
  SIM->SCGC6 |= SIM_SCGC6_TPM1_MASK;		// ToDo 2.1.1: Enable TPM1 mask in SCGC6 register
	SIM->SOPT2 |= SIM_SOPT2_TPMSRC(01);// ToDo 2.1.1: Choose MCGFLLCLK clock source
	
	SIM->SCGC5 |= SIM_SCGC5_PORTA_MASK; // ToDo 2.1.2: Connect port A to clock
	PORTA->PCR[0] = PORT_PCR_MUX(3);  // ToDo 2.1.2: Set multiplekser to TPM1 for PTA0, get channel number (page 148 of the Reference Manual)
	
	TPM1->SC |= TPM_SC_PS(7);  				// ToDo 2.1.3: Set prescaler to 128
	TPM1->SC |= TPM_SC_CMOD(1);					// ToDo 2.1.4: For TMP1, select the internal input clock source
	 Connect correct channel from TPM1 for "input capture" mode
	// TPM1->SC &= ~TPM_SC_CPWMS_MASK; 		up counting
//	
//	TPM1->CONTROLS[0].CnSC &= ~ (TPM_CnSC_MSA_MASK | TPM_CnSC_MSB_MASK);
//	TPM1->CONTROLS[0].CnSC |= (TPM_CnSC_ELSA_MASK | TPM_CnSC_ELSB_MASK);  capture on both edges  
 // 
//	TPM1->CONTROLS[0].CnSC |= TPM_CnSC_CHIE_MASK; // ToDo 2.1.6: Enable interrupt on selected channel
//	
*/
	//NVIC_SetPriority(TPM1_IRQn, 1); //  TPM1 interrupt priority level  
	//NVIC_ClearPendingIRQ(TPM1_IRQn); 
	//NVIC_EnableIRQ(TPM1_IRQn);	//Enable Interrupts 


/*
 LCD1602_Init();  initialize all LEDs 
 LCD1602_Backlight(FALSE);

 LCD1602_SetCursor(1,0);
 LCD1602_Print("MORSE CODE");
 LCD1602_SetCursor(3,1);
 LCD1602_Print("DECODER");

 DELAY(2000);
	
 LCD1602_ClearAll();
 LCD1602_SetCursor(0,0);
 LCD1602_Print("START ENCODING!");
 DELAY(1500);
 LCD1602_ClearAll();
 LCD1602_SetCursor(0,0);
*/

 while(1){
 	  if(rx_FULL)		// if there is data in rx_FULL
		{
			while(!(UART0->S1 & UART0_S1_TDRE_MASK));	// Is TX ready?
			UART0->D = rx_buf[0];
			rx_FULL=0;	// Data taken
		}
			}
};
