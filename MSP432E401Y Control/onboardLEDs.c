#include <stdint.h>
#include "tm4c1294ncpdt.h"
#include "PLL.h"
#include "SysTick.h"
#include "onboardLEDs.h"

// SysTick Function
#define NVIC_ST_CTRL_R          (*((volatile uint32_t *)0xE000E010))
#define NVIC_ST_RELOAD_R        (*((volatile uint32_t *)0xE000E014))
#define NVIC_ST_CURRENT_R       (*((volatile uint32_t *)0xE000E018))
#define NVIC_ST_CTRL_COUNT      0x00010000  // Count flag
#define NVIC_ST_CTRL_CLK_SRC    0x00000004  // Clock Source
#define NVIC_ST_CTRL_INTEN      0x00000002  // Interrupt enable
#define NVIC_ST_CTRL_ENABLE     0x00000001  // Counter mode
#define NVIC_ST_RELOAD_M        0x00FFFFFF  // Counter load value

void SysTick_Init(void){
  NVIC_ST_CTRL_R = 0;                   // disable SysTick during setup
  NVIC_ST_RELOAD_R = NVIC_ST_RELOAD_M;  // maximum reload value
  NVIC_ST_CURRENT_R = 0;                // any write to current clears it
                                        // enable SysTick with core clock
  NVIC_ST_CTRL_R = NVIC_ST_CTRL_ENABLE+NVIC_ST_CTRL_CLK_SRC;
}

void SysTick_Wait(uint32_t delay){
  volatile uint32_t elapsedTime;
  uint32_t startTime = NVIC_ST_CURRENT_R;
  do{
    elapsedTime = (startTime-NVIC_ST_CURRENT_R)&0x00FFFFFF;
  }
  while(elapsedTime <= delay);
}

void SysTick_Wait10ms(uint32_t delay){
  uint32_t i;
  for(i=0; i<delay; i++){
    SysTick_Wait(960);  // wait 10us (assumes 96 MHz clock)
  }
}
//
#define DELAY 1

//Flash D1
void FlashLED1(int count) {
		while(count--) {
			GPIO_PORTN_DATA_R ^= 0b00000010; 								//hello world!
			SysTick_Wait10ms(DELAY * 100);														//.05s delay
			GPIO_PORTN_DATA_R ^= 0b00000010;			
			//SysTick_Wait10ms(DELAY);														//.05s delay
		}
}

//Flash D2
void FlashLED2(int count) {
		while(count--) {
			GPIO_PORTN_DATA_R ^= 0b00000001; 								//hello world!
			SysTick_Wait10ms(DELAY);														//.05s delay
			GPIO_PORTN_DATA_R ^= 0b00000001;			
			SysTick_Wait10ms(DELAY);														//.05s delay			
		}
}

//Flash D3
void FlashLED3(int count) {
		while(count--) {
			GPIO_PORTF_DATA_R ^= 0b00010000; 								//hello world!
			SysTick_Wait10ms(DELAY);														//.05s delay
			GPIO_PORTF_DATA_R ^= 0b00010000;			
			//SysTick_Wait10ms(DELAY);														//.05s delay			
		}
}

//Flash D4
void FlashLED4(int count) {
		while(count--) {
			GPIO_PORTF_DATA_R ^= 0b00000001; 								//hello world!
			SysTick_Wait10ms(DELAY);														//.05s delay
			GPIO_PORTF_DATA_R ^= 0b00000001;			
			//SysTick_Wait10ms(DELAY);														//.05s delay			
		}
}

void FlashAllLEDs(){
		GPIO_PORTN_DATA_R ^= 0b00000011; 								//hello world!
		GPIO_PORTF_DATA_R ^= 0b00010001; 								//hello world!	
		SysTick_Wait10ms(25);														//.25s delay
		GPIO_PORTN_DATA_R ^= 0b00000011;			
		GPIO_PORTF_DATA_R ^= 0b00010001; 								//goodbye world!	
		//SysTick_Wait10ms(25);														//.25s delay			
}

void FlashI2CTx(){
	FlashLED3(1);
}

void FlashI2CRx(){
	FlashLED4(1);
}



//Flash Error D1&D2&D3&D4
void FlashI2CError(int count) {
		while(count--) {
			FlashAllLEDs();
		}
}






// Initialize onboard LEDs
void onboardLEDs_Init(void){
	//Use PortN onboard LEDs	
	SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R12;				// activate clock for Port N
	while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R12) == 0){};	// allow time for clock to stabilize
	GPIO_PORTN_DIR_R |= 0x03;        								// make PN0 out (PN0 built-in LED1)
  GPIO_PORTN_AFSEL_R &= ~0x03;     								// disable alt funct on PN0,PN1
  GPIO_PORTN_DEN_R |= 0x03;        								// enable digital I/O on PN0,PN1
																									// configure PN1,PN0 as GPIO
  //GPIO_PORTN_PCTL_R = (GPIO_PORTN_PCTL_R&0xFFFFFF00)+0x00000000;
  GPIO_PORTN_AMSEL_R &= ~0x03;     								// disable analog functionality on PN0,PN1

	//Use PortF onboard LEDs	
	SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R5;				// activate clock for Port N
	while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R5) == 0){};	// allow time for clock to stabilize
	GPIO_PORTF_DIR_R |= 0x11;        								// make PN4,0 out 
  GPIO_PORTF_AFSEL_R &= ~0x11;     								// disable alt funct on PF4,PF0
  GPIO_PORTF_DEN_R |= 0x11;        								// enable digital I/O on PF4,PF0
																									// configure PF4,PF0 as GPIO
  //GPIO_PORTN_PCTL_R = (GPIO_PORTN_PCTL_R&0xFFF0FFF0)+0x00000000;
  GPIO_PORTF_AMSEL_R &= ~0x011;     								// disable analog functionality on PF4,PF0
		
	FlashAllLEDs();
	return;
}




