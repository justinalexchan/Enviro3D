//Justin Chan
#include <stdint.h>
#include "tm4c1294ncpdt.h"
#include "vl53l1x_api.h"
#include "PLL.h"
#include "SysTick.h"
#include "uart.h"
#include "onboardLEDs.h"
#include <stdio.h>
#include <stdlib.h>

uint16_t	dev=0x52;

int status=0;

volatile int IntCount;

//device in interrupt mode (GPIO1 pin signal)
#define isInterrupt 1 /* If isInterrupt = 1 then device working in interrupt mode, else device working in polling mode */

void I2C_Init(void);
void UART_Init(void);
void PortG_Init(void);
void VL53L1X_XSHUT(void);

void PortN_Init(void);
void PortL_Init(void);
void PortM_Init(void);
void motorStart(void);
//capture values from VL53L1X for inspection
uint16_t debugArray[100];



//Interupt

// Enable interrupts
void EnableInt(void)
{    __asm("    cpsie   i\n");
}

// Disable interrupts
void DisableInt(void)
{    __asm("    cpsid   i\n");
}


// global variable visible in Watch window of debugger
// increments at least once per button press
// GPIO Port J = Vector 67
// Bit in interrupt register = 51

void ExternalButton_Init(void){
	SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R8;				// activate clock for Port J
	while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R8) == 0){};	// allow time for clock to stabilize

  GPIO_PORTJ_DIR_R &= ~0x02;    // (c) make PJ1 in 

  GPIO_PORTJ_DEN_R |= 0x02;     //     enable digital I/O on PJ1
	GPIO_PORTJ_PCTL_R &= ~0x000000F0; //� configure PJ1 as GPIO 
	GPIO_PORTJ_AMSEL_R &= ~0x02;	//�� disable analog functionality on PJ1		
	GPIO_PORTJ_PUR_R |= 0x02;			//	enable weak pull up resistor
  GPIO_PORTJ_IS_R &= ~0x02;     // (d) PJ1 is edge-sensitive 
  GPIO_PORTJ_IBE_R &= ~0x02;    //     PJ1 is not both edges 
  GPIO_PORTJ_IEV_R &= ~0x02;    //     PJ1 falling edge event 
  GPIO_PORTJ_ICR_R = 0x02;      // (e) clear flag1
  GPIO_PORTJ_IM_R |= 0x02;      // (f) arm interrupt on PJ1
  NVIC_PRI13_R = (NVIC_PRI13_R&0xFF00FFFF)|0x000A0000; // (g) priority 5
  NVIC_EN1_R |= 0x00080000;              // (h) enable interrupt 67 in NVIC
  EnableInt();           				// lets go
}

void GPIOJ_IRQHandler(void){
	GPIO_PORTJ_ICR_R = 0x02;      // acknowledge flag
	SysTick_Wait(1200000);

	while((GPIO_PORTM_DATA_R&0b00000001)== 0){
		
	}
}

int main(void) {
  uint8_t byteData, sensorState=0, myByteArray[10] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF} , i=0;
  uint16_t wordData;
  uint8_t ToFSensor = 1; // 0=Left, 1=Center(default), 2=Right
  uint16_t Distance;
  uint16_t SignalRate;
  uint16_t AmbientRate;
  uint16_t SpadNum; 
  uint8_t RangeStatus;
  uint8_t dataReady;

	//initialize
	PLL_Init();	
	SysTick_Init();
	onboardLEDs_Init();
	I2C_Init();
	UART_Init();
	
	PortM_Init();
	PortL_Init();
	
	ExternalButton_Init();
	// hello world!
	UART_printf("Program Begins\r\n");
	int mynumber = 1;
	sprintf(printf_buffer,"2DX4 Final Project Code \r\n");
	UART_printf(printf_buffer);


/* Those basic I2C read functions can be used to check your own I2C functions */
  status = VL53L1_RdByte(dev, 0x010F, &byteData);					// This is the model ID.  Expected returned value is 0xEA
  myByteArray[i++] = byteData;

  status = VL53L1_RdByte(dev, 0x0110, &byteData);					// This is the module type.  Expected returned value is 0xCC
  myByteArray[i++] = byteData;
	
	status = VL53L1_RdWord(dev, 0x010F, &wordData);
	status = VL53L1X_GetSensorId(dev, &wordData);

	sprintf(printf_buffer,"Model_ID=0x%x , Module_Type=0x%x\r\n",myByteArray[0],myByteArray[1]);
	UART_printf(printf_buffer);

	// Booting ToF chip
	while(sensorState==0){
		status = VL53L1X_BootState(dev, &sensorState);
		SysTick_Wait10ms(10);
  }
	FlashAllLEDs();
	UART_printf("ToF Chip Booted!\r\n");
 	UART_printf("One moment...\r\n");
	
	status = VL53L1X_ClearInterrupt(dev); /* clear interrupt has to be called to enable next interrupt*/

	
  /* This function must to be called to initialize the sensor with the default setting  */
  status = VL53L1X_SensorInit(dev);
	Status_Check("SensorInit", status);

  status = VL53L1X_StartRanging(dev);   /* This function has to be called to enable the ranging */
	Status_Check("StartRanging", status);

	UART_printf("Push Button to begin...\r\n");
	int aglCount = 0;
	while(1){
		aglCount = 0;
		GPIO_PORTL_DATA_R |= 0b00010;
		if(((GPIO_PORTM_DATA_R&0b00000001)== 1)){	//Detects if button is pushed
			GPIO_PORTL_DATA_R = 0b00000;
			for(int i = 0; i < 64; i++) {	//Takes 64 Distance measurement
				while (dataReady == 0){
					status = VL53L1X_CheckForDataReady(dev, &dataReady);
							//FlashLED3(1);
							VL53L1_WaitMs(dev, 5);
				}

					dataReady = 0;
				status = VL53L1X_GetRangeStatus(dev, &RangeStatus);
				status = VL53L1X_GetDistance(dev, &Distance);
					//FlashLED4(1);

					debugArray[i] = Distance;
				status = VL53L1X_ClearInterrupt(dev); /* clear interrupt has to be called to enable next interrupt*/
					//sprintf(printf_buffer,"%u, %u, %u, %u, %u\r\n", RangeStatus, Distance, SignalRate, AmbientRate,SpadNum);
					sprintf(printf_buffer,"%u\r\n", Distance);	// only prints distance data
					UART_printf(printf_buffer);
				
				motorStart();	//Rotates motor 8 steps
				aglCount += 8;
				if(aglCount % 64 == 0){		//Blinks onboard LED every 45 degrees
					FlashLED1(1);
				}
			}
		}
	}
VL53L1X_StopRanging(dev);

}

//TOF Code
#define I2C_MCS_ACK             0x00000008  // Data Acknowledge Enable
#define I2C_MCS_DATACK          0x00000008  // Acknowledge Data
#define I2C_MCS_ADRACK          0x00000004  // Acknowledge Address
#define I2C_MCS_STOP            0x00000004  // Generate STOP
#define I2C_MCS_START           0x00000002  // Generate START
#define I2C_MCS_ERROR           0x00000002  // Error
#define I2C_MCS_RUN             0x00000001  // I2C Master Enable
#define I2C_MCS_BUSY            0x00000001  // I2C Busy
#define I2C_MCR_MFE             0x00000010  // I2C Master Function Enable

#define MAXRETRIES              5           // number of receive attempts before giving up
void I2C_Init(void){
  SYSCTL_RCGCI2C_R |= SYSCTL_RCGCI2C_R0;           // activate I2C0
  SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R1;          // activate port B
  while((SYSCTL_PRGPIO_R&0x0002) == 0){};// ready?

    GPIO_PORTB_AFSEL_R |= 0x0C;           // 3) enable alt funct on PB2,3       0b00001100
    GPIO_PORTB_ODR_R |= 0x08;             // 4) enable open drain on PB3 only

    GPIO_PORTB_DEN_R |= 0x0C;             // 5) enable digital I/O on PB2,3
//    GPIO_PORTB_AMSEL_R &= ~0x0C;          // 7) disable analog functionality on PB2,3

                                                                                // 6) configure PB2,3 as I2C
//  GPIO_PORTB_PCTL_R = (GPIO_PORTB_PCTL_R&0xFFFF00FF)+0x00003300;
  GPIO_PORTB_PCTL_R = (GPIO_PORTB_PCTL_R&0xFFFF00FF)+0x00002200;    //TED
    I2C0_MCR_R = I2C_MCR_MFE;                      // 9) master function enable
    I2C0_MTPR_R = 0b0000000000000101000000000111011;                                        // 8) configure for 100 kbps clock (added 8 clocks of glitch suppression ~50ns)
    
}

//The VL53L1X needs to be reset using XSHUT.  We will use PG0
void PortG_Init(void){
    //Use PortG0
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R6;                // activate clock for Port N
    while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R6) == 0){};    // allow time for clock to stabilize
    GPIO_PORTG_DIR_R &= 0x00;                                        // make PG0 in (HiZ)
  GPIO_PORTG_AFSEL_R &= ~0x01;                                     // disable alt funct on PG0
  GPIO_PORTG_DEN_R |= 0x01;                                        // enable digital I/O on PG0
                                                                                                    // configure PG0 as GPIO
  //GPIO_PORTN_PCTL_R = (GPIO_PORTN_PCTL_R&0xFFFFFF00)+0x00000000;
  GPIO_PORTG_AMSEL_R &= ~0x01;                                     // disable analog functionality on PN0

    return;
}

//XSHUT     This pin is an active-low shutdown input; the board pulls it up to VDD to enable the sensor by default. Driving this pin low puts the sensor into hardware standby. This input is not level-shifted.
void VL53L1X_XSHUT(void){
    GPIO_PORTG_DIR_R |= 0x01;                                        // make PG0 out
    GPIO_PORTG_DATA_R &= 0b11111110;                                 //PG0 = 0
    FlashAllLEDs();
    SysTick_Wait10ms(10);
    GPIO_PORTG_DIR_R &= ~0x01;                                            // make PG0 input (HiZ)
    
}

//Mid Project Code
#define NVIC_ST_CTRL_R          (*((volatile uint32_t *)0xE000E010))
#define NVIC_ST_RELOAD_R        (*((volatile uint32_t *)0xE000E014))
#define NVIC_ST_CURRENT_R       (*((volatile uint32_t *)0xE000E018))
#define NVIC_ST_CTRL_COUNT      0x00010000  // Count flag
#define NVIC_ST_CTRL_CLK_SRC    0x00000004  // Clock Source
#define NVIC_ST_CTRL_INTEN      0x00000002  // Interrupt enable
#define NVIC_ST_CTRL_ENABLE     0x00000001  // Counter mode
#define NVIC_ST_RELOAD_M        0x00FFFFFF  // Counter load value


void PortN_Init(void){
	//Use PortN onboard LED	
	SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R12;				// activate clock for Port N
	while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R12) == 0){};	// allow time for clock to stabilize
	GPIO_PORTN_DIR_R |= 0b01;        								// make PN0 out (PN0 built-in LED1)
  GPIO_PORTN_AFSEL_R &= ~0b01;     								// disable alt funct on PN0
  GPIO_PORTN_DEN_R |= 0b01;        								// enable digital I/O on PN0
																									// configure PN1 as GPIO
  //GPIO_PORTN_PCTL_R = (GPIO_PORTN_PCTL_R&0xFFFFFF0F)+0x00000000;
								// disable analog functionality on PN0		
	return;
}


void PortL_Init(void){
	SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R10;	
	while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R10) == 0){};	
	GPIO_PORTL_DIR_R |= 0b10;        								
  GPIO_PORTL_AFSEL_R &= ~0b10;     								
  GPIO_PORTL_DEN_R |= 0b10;        								    									
	

	return;
}


void PortM_Init(void){
	SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R11;				
	while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R11) == 0){};	
	GPIO_PORTM_DIR_R |= 0x3C; 
  GPIO_PORTM_AFSEL_R &= ~0x3D;     							
  GPIO_PORTM_DEN_R |= 0x3D;        							
	return;
}

void motorStart(void){
	for(int i = 1; i <= 8; i++){		//512 is one full rotation
		int delay = 192000;

		GPIO_PORTM_DATA_R &= 0b00000000;
		GPIO_PORTM_DATA_R |= 0b00100100;
		SysTick_Wait(delay);
		GPIO_PORTM_DATA_R &= 0b00000000;
		GPIO_PORTM_DATA_R |= 0b00110000;
		SysTick_Wait(delay);
		GPIO_PORTM_DATA_R &= 0b00000000;
		GPIO_PORTM_DATA_R |= 0b00011000;
		SysTick_Wait(delay);
		GPIO_PORTM_DATA_R &= 0b00000000;
		GPIO_PORTM_DATA_R |= 0b00001100;
		SysTick_Wait(delay);

	}
}

