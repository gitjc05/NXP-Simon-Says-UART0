/*********************************************************************/
/* <Your program description here>                                   */
/* Name:  <Your name here>                                           */
/* Date:  <Date completed>                                           */
/* Class:  CMPE 250                                                  */
/* Section:  <Your section here>                                     */
/*-------------------------------------------------------------------*/
/* Template:  R. W. Melton                                           */
/*            November 11, 2024                                      */
/*********************************************************************/
#include "MKL05Z4.h"
#include "Exercise11_C.h"

#define FALSE      (0)
#define TRUE       (1)

#define MAX_STRING (79)

#define PWM_FREQ        (50u)
/* #define TPM_SOURCE_FREQ (48000000u) */
#define TPM_SOURCE_FREQ (47972352u)

/* ADC0 */
#define ADC_MAX (0x3FFu)
#define ADC_VALUES (0x400u)
/* ------------------------------------------------------------------*/
/* ADC0_CFG1:  ADC0 configuration register 1                         */
/*  0-->31-8:(reserved):read-only:0                                  */
/*  1-->   7:ADLPC=ADC low-power configuration                       */
/* 10--> 6-5:ADIV=ADC clock divide select                            */
/*           Internal ADC clock = input clock / 2^ADIV               */
/*  1-->   4:ADLSMP=ADC long sample time configuration               */
/* 10--> 3-2:MODE=conversion mode selection                          */
/*           00=single-ended 8-bit conversion                        */
/*           01=single-ended 12-bit conversion                       */
/*           10=single-ended 10-bit conversion                       */
/*           11=(reserved) do not use this value                     */
/* 01--> 1-0:ADICLK=ADC input clock select                           */
/*           00=bus clock                                            */
/*           01=bus clock / 2                                        */
/*           10=alternate clock (ALTCLK)                             */
/*           11=asynchronous clock (ADACK)                           */
/* BUSCLK = CORECLK / 2 = FLLCLK / 2                                 */
/* FLLCLK is ~48 MHz                                                 */
/* FLLCLK is 47972352 Hz                                             */
/* BUSCLK is ~24 MHz                                                 */
/* BUSCLK is 23986176 Hz                                             */
/* ADCinCLK is BUSCLK / 2 = ~12 MHz                                  */
/* BUSCLK is 11993088 Hz                                             */
/* ADCCLK is ADCinCLK / 4 = ~3 MHz                                   */
/* ADCCLK is 2998272 Hz                                              */
#define ADC0_CFG1_ADIV_BY2 (2u)
#define ADC0_CFG1_MODE_SGL10 (2u)
#define ADC0_CFG1_ADICLK_BUSCLK_DIV2 (1u)
#define ADC0_CFG1_LP_LONG_SGL10_3MHZ (ADC_CFG1_ADLPC_MASK | \
              (ADC0_CFG1_ADIV_BY2 << ADC_CFG1_ADIV_SHIFT) | \
                                     ADC_CFG1_ADLSMP_MASK | \
            (ADC0_CFG1_MODE_SGL10 << ADC_CFG1_MODE_SHIFT) | \
                               ADC0_CFG1_ADICLK_BUSCLK_DIV2)
/*-------------------------------------------------------------*/
/* ADC0_CFG2:  ADC0 configuration register 2                   */
/*  0-->31-8:(reserved):read-only:0                            */
/*  0--> 7-5:(reserved):read-only:0                            */
/*  0-->   4:MUXSEL=ADC mux select:  A channel                 */
/*  0-->   3:ADACKEN=ADC asynchronous clock output enable      */
/*  0-->   2:ADHSC=ADC high-speed configuration:  normal       */
/* 00--> 1-0:ADLSTS=ADC long sample time select (ADK cycles)   */
/*           default longest sample time:  24 total ADK cycles */
#define ADC0_CFG2_CHAN_A_NORMAL_LONG (0x00u)
/*---------------------------------------------------------   */
/* ADC0_SC1:  ADC0 channel status and control register 1      */
/*     0-->31-8:(reserved):read-only:0                        */
/*     0-->   7:COCO=conversion complete flag (read-only)     */
/*     0-->   6:AIEN=ADC interrupt enabled                    */
/*     0-->   5:DIFF=differential mode enable                 */
/* 00101--> 4-0:ADCH=ADC input channel select (5 = DAC0_OUT) */
#define ADC0_SC1_ADCH_AD5 (5u)
#define ADC0_SC1_SGL_DAC0 (ADC0_SC1_ADCH_AD5)
/*-----------------------------------------------------------*/
/* ADC0_SC2:  ADC0 status and control register 2             */
/*  0-->31-8:(reserved):read-only:0                          */
/*  0-->   7:ADACT=ADC conversion active                     */
/*  0-->   6:ADTRG=ADC conversion trigger select:  software  */
/*  0-->   5:ACFE=ADC compare function enable                */
/*  X-->   4:ACFGT=ADC compare function greater than enable  */
/*  0-->   3:ACREN=ADC compare function range enable         */
/*            0=disabled; only ADC0_CV1 compared             */
/*            1=enabled; both ADC0_CV1 and ADC0_CV2 compared */
/*  0-->   2:DMAEN=DMA enable                                */
/* 01--> 1-0:REFSEL=voltage reference selection:  VDDA       */
#define ADC0_SC2_REFSEL_VDDA (1u)
#define ADC0_SC2_SWTRIG_VDDA (ADC0_SC2_REFSEL_VDDA)
/*-------------------------------------------------------------*/
/* ADC0_SC3:  ADC0 status and control register 3               */
/* 31-8:(reserved):read-only:0                                 */
/*  0-->   7:CAL=calibration                                   */
/*          write:0=(no effect)                                */
/*                1=start calibration sequence                 */
/*          read:0=calibration sequence complete               */
/*               1=calibration sequence in progress            */
/*  0-->   6:CALF=calibration failed flag                      */
/* 00--> 5-4:(reserved):read-only:0                            */
/*  0-->   3:ADCO=ADC continuous conversion enable             */
/*           (if ADC0_SC3.AVGE = 1)                            */
/*  0-->   2:AVGE=hardware average enable                      */
/* XX--> 1-0:AVGS=hardware average select:  2^(2+AVGS) samples */
#define ADC0_SC3_SINGLE (0x00u)
#define ADC0_SC3_CAL (ADC_SC3_CAL_MASK | ADC_SC3_AVGE_MASK | \
                                           ADC_SC3_AVGS_MASK)
/*-------------------------------------------------------------*/
/* DAC */
#define DAC_DATH_MIN   0x00u
#define DAC_DATL_MIN   0x00u
/*---------------------------------------------------------------------*/
/* DAC_C0:  DAC control register 0                                   */
/* 1-->7:DACEN=DAC enabled                                             */
/* 1-->6:DACRFS=DAC reference select VDDA                              */
/* 0-->5:DACTRGSEL=DAC trigger select (X)                              */
/* 0-->4:DACSWTRG=DAC software trigger (X)                             */
/* 0-->3:LPEN=DAC low power control:high power                         */
/* 0-->2:(reserved):read-only:0                                        */
/* 0-->1:DACBTIEN=DAC buffer read pointer top flag interrupt enable    */
/* 0-->0:DACBBIEN=DAC buffer read pointer bottom flag interrupt enable */
#define DAC_C0_ENABLE  (DAC_C0_DACEN_MASK | DAC_C0_DACRFS_MASK)
/*----------------------------------------*/
/* DAC_C1:  DAC control register 1      */
/* 0-->  7:DMAEN=DMA disabled             */
/* 0-->6-3:(reserved)                     */
/* 0-->  2:DACBFMD=DAC buffer mode normal */
/* 0-->  1:(reserved)                     */
/* 0-->  0:DACBFEN=DAC buffer disabled    */
#define DAC_C1_BUFFER_DISABLED  (0x00u)
/*-------------------------------------------------------------*/
/* PORTx_PCRn                                                  */
/*     -->31-25:(reserved):read-only:0                         */
/*    1-->   24:ISF=interrupt status flag (write 1 to clear)   */
/*     -->23-20:(reserved):read-only:0                         */
/* 0000-->19-16:IRQC=interrupt configuration (IRQ/DMA diabled) */
/*     -->15-11:(reserved):read-only:0                         */
/*  ???-->10- 8:MUX=pin mux control                            */
/*     -->    7:(reserved):read-only:0                         */
/*    ?-->    6:DSE=drive strengh enable                       */
/*     -->    5:(reserved):read-only:0                         */
/*    0-->    4:PFE=passive filter enable                      */
/*     -->    3:(reserved):read-only:0                         */
/*    0-->    2:SRE=slew rate enable                           */
/*    0-->    1:PE=pull enable                                 */
/*    x-->    0:PS=pull select                                 */
/* Port B */
/*   DAC0_OUT connects through PTB1 to FRDM-KL05Z J8 pin 2 (D1)*/
/*   This connection is not compatible with UART0 USB COM port */
#define PTB1_MUX_DAC0_OUT (0u << PORT_PCR_MUX_SHIFT)
#define SET_PTB1_DAC0_OUT (PORT_PCR_ISF_MASK | PTB1_MUX_DAC0_OUT)
/*   TPM0_CH2_OUT connects through PTB7                        */
/*     to FRDM-KL05Z J8 pin 8 (D7)                             */
#define PTB7_MUX_TPM0_CH2_OUT (2u << PORT_PCR_MUX_SHIFT)
#define SET_PTB7_TPM0_CH2_OUT (PORT_PCR_ISF_MASK | \
                               PTB7_MUX_TPM0_CH2_OUT)
/*-------------------------------------------------------*/
/* SIM_SOPT2                                             */
/*   -->31-28:(reserved):read-only:0                     */
/*   -->27-26:UART0SRC=UART0 clock source select         */
/* 01-->25-24:TPMSRC=TPM clock source select (MCGFLLCLK) */
/*   -->23-19:(reserved):read-only:0                     */
/*   -->   18:USBSRC=USB clock source select             */
/*   -->   17:(reserved):read-only:0                     */
/*  1-->   16:PLLFLLSEL=PLL/FLL clock select (MCGFLLCLK) */
/*   -->15- 8:(reserved):read-only:0                     */
/*   --> 7- 5:CLKOUTSEL=CLKOUT select                    */
/*   -->    4:RTCCLKOUTSEL=RTC clock out select          */
/*   --> 3- 0:(reserved):read-only:0                     */
#define SIM_SOPT2_TPMSRC_MCGFLLCLK (1u << SIM_SOPT2_TPMSRC_SHIFT)
/*------------------------------------------------------------------*/
/* TPMx_CONF:  Configuration (recommended to use default values     */
/*     -->31-28:(reserved):read-only:0                              */
/* 0000-->27-24:TRGSEL=trigger select (external pin EXTRG_IN)       */
/*     -->23-19:(reserved):read-only:0                              */
/*    0-->   18:CROT=counter reload on trigger                      */
/*    0-->   17:CSOO=counter stop on overflow                       */
/*    0-->   16:CSOT=counter stop on trigger                        */
/*     -->15-10:(reserved):read-only:0                              */
/*    0-->    9:GTBEEN=global time base enable                      */
/*     -->    8:(reserved):read-only:0                              */
/*   00-->  7-6:DBGMODE=debug mode (paused in debug)                */
/*    0-->    5:DOZEEN=doze enable                                  */
/*     -->  4-0:(reserved):read-only:0                              */
/* 15- 0:COUNT=counter value (writing any value will clear counter) */
#define TPM_CONF_DEFAULT (0u)
/*------------------------------------------------------------------*/
/* TPMx_CNT:  Counter                                               */
/* 31-16:(reserved):read-only:0                                     */
/* 15- 0:COUNT=counter value (writing any value will clear counter) */
#define TPM_CNT_INIT (0u)
/*------------------------------------------------------------------------*/
/* TPMx_MOD:  Modulo                                                      */
/* 31-16:(reserved):read-only:0                                           */
/* 15- 0:MOD=modulo value (recommended to clear TPMx_CNT before changing) */
/* Period = 3 MHz / 50 Hz */
/* #define TPM_PWM_PERIOD_20ms ((TPM_SOURCE_FREQ / (1 << TPM_SC_PS_DIV16)) / PWM_FREQ) */
/* #define TPM_MOD_PWM_PERIOD_20ms (TPM_PWM_PERIOD - 1)                        */
#define TPM_PWM_PERIOD (60000u)
#define TPM_MOD_PWM_PERIOD_20ms (TPM_PWM_PERIOD - 1)       
/*------------------------------------------------------------------*/
/* TPMx_CnSC:  Channel n Status and Control                         */
/* 0-->31-8:(reserved):read-only:0                                  */
/* 0-->   7:CHF=channel flag                                        */
/*              set on channel event                                */
/*              write 1 to clear                                    */
/* 0-->   6:CHIE=channel interrupt enable                           */
/* 1-->   5:MSB=channel mode select B (see selection table below)   */
/* 0-->   4:MSA=channel mode select A (see selection table below)   */
/* 1-->   3:ELSB=edge or level select B (see selection table below) */
/* 0-->   2:ELSA=edge or level select A (see selection table below) */
/* 0-->   1:(reserved):read-only:0                                  */
/* 0-->   0:DMA=DMA enable                                          */
#define TPM_CnSC_PWMH (TPM_CnSC_MSB_MASK | TPM_CnSC_ELSB_MASK)
/*--------------------------------------------------------*/
/* TPMx_CnV:  Channel n Value                             */
/* 31-16:(reserved):read-only:0                           */
/* 15- 0:MOD (all bytes must be written at the same time) */
/*--------------------------------------------------------------*/
/* TPMx_SC:  Status and Control                                 */
/*    -->31-9:(reserved):read-only:0                            */
/*   0-->   8:DMA=DMA enable                                    */
/*    -->   7:TOF=timer overflow flag                           */
/*   0-->   6:TOIE=timer overflow interrupt enable              */
/*   0-->   5:CPWMS=center-aligned PWM select (edge align)      */
/*  01--> 4-3:CMOD=clock mode selection (count each TPMx clock) */
/* 100--> 2-0:PS=prescale factor selection                      */
/*    -->        can be written only when counter is disabled   */
#define TPM_SC_CMOD_CLK (1u)
#define TPM_SC_PS_DIV16 (0x4u)
#define TPM_SC_CLK_DIV16 ((TPM_SC_CMOD_CLK << TPM_SC_CMOD_SHIFT) | \
                          TPM_SC_PS_DIV16)
/*- -----*/
/* Servo */
#define SERVO_POSITIONS  (5)
// LEDS
#define POS_RED (8)
#define POS_GREEN (9)
#define POS_BLUE (10)

#define PORTB_LED_RED_MASK (1 << POS_RED)
#define PORTB_LED_GREEN_MASK (1 << POS_GREEN)
#define PORTB_LED_BLUE_MASK (1 << POS_BLUE)
#define PORTB_LEDS_MASK (PORTB_LED_RED_MASK | \
													PORTB_LED_GREEN_MASK | \
													PORTB_LED_BLUE_MASK)
/* Port Pin GPIO for LED */
#define PORT_PCR_MUX_SELECT_1_MASK \
													(1 << PORT_PCR_MUX_SHIFT)
#define PORT_PCR_SET_GPIO \
										(PORT_PCR_ISF_MASK | PORT_PCR_MUX_SELECT_1_MASK)





int Init_LED() {
	// initializes the board for use of the RGB LED.
	
	//set clock on port B
	SIM->SCGC5 |= SIM_SCGC5_PORTB_MASK;
	/* Select PORT B Pin 8 for GPIO to red LED */
	PORTB->PCR[POS_RED] = PORT_PCR_SET_GPIO;
	/* Select PORT B Pin 9 for GPIO to green LED */
	PORTB->PCR[POS_GREEN] = PORT_PCR_SET_GPIO;
	/* Select PORT B Pin 10 for GPIO to blue LED */
	PORTB->PCR[POS_BLUE] = PORT_PCR_SET_GPIO;
	
	FPTB->PDDR = PORTB_LEDS_MASK;
	/* Turn off red LED */
	FPTB->PSOR = PORTB_LED_RED_MASK;
	/* Turn off green LED */
	FPTB->PSOR = PORTB_LED_GREEN_MASK;
	/* Turn off blue LED */
	FPTB->PSOR = PORTB_LED_BLUE_MASK;
	return 0;
}

int setLED(int r, int g, int b) {
	// takes in r, g, b => red, green, and blue LED values
	// sets or clears the values based on given input
	// an example of red: setLED(1, 0, 0)... (r, g, b).
	if (r == 0) {
		FPTB->PSOR = PORTB_LED_RED_MASK;
	} else {
		//turn on red
		FPTB->PCOR = PORTB_LED_RED_MASK;
	}
	if (g == 0) {
		FPTB->PSOR = PORTB_LED_GREEN_MASK;
	} else {
		//turn on green
		FPTB->PCOR = PORTB_LED_GREEN_MASK;
	}
	if (b == 0) {
		FPTB->PSOR = PORTB_LED_BLUE_MASK;
	} else {
		//turn on blue
		FPTB->PCOR = PORTB_LED_BLUE_MASK;
	}
	return 0;
}


int randint(int top) {
	int seed = MainClock;
	seed += 13*MainClock;
	top = seed % top;
	top ++;
	return top;
}

int calcScore(int timeR, int roundN) {
	// score takes in the time it took for the user to respond
	// score also takes in the current round number
	// scoring rewards speed and ability to make it to later rounds
	// returns the calculated score
	int remaining = 1100 - timeR;
	int multiple = roundN;
	multiple *= multiple;
	multiple ++;
	return remaining * multiple;
}


int charToLED(char r) {
	// converts user input upper or lower case
	// to a value from 1 - 3 for r, g, or b input
	// if user input does not match returns 4 
	// a return of 4 is interpreted as an error by the main program.
	char caps[3] = {'R', 'G', 'B'};
	char lowers[3] = {'r', 'g', 'b'};
	for (int i = 0; i < sizeof(caps); i++) {
		if (r == caps[i] | r == lowers[i]) {
			return i+1;
		}
	}
	return 4;
}

int setLedCol(int ledCol) {
	// takes in a number from 1-3 for r, g, or b light
	// sets the LED light color to the given value and prints the color
	if (ledCol == 1) {
		setLED(1, 0, 0);
		return ledCol;
	}
	if (ledCol == 2) {
		setLED(0, 1, 0);
		return ledCol;
	}
	if (ledCol == 3) {
		setLED(0, 0, 1);
		return ledCol;
	}
	return -1;
}

int preGameTimer(int duration) {
	// timer countdown for start of round
	// takes in a duration length, for how long the count down will be
	// blinks white LED each second to prepare user
	setLED(0, 0, 0);
	setLED(1, 1, 1);
	PutNumUB(duration);
	PutStringSB("\r", 8);
	int initCount = Count;
	int SL = duration;
	int prevSL = SL;
	while (SL <= duration) {
		setLED(0, 0, 0);
		if (SL != prevSL) {
			int displaySL = duration - SL;
			PutNumU(displaySL);
			setLED(1, 1, 1);
			prevSL = SL;
		}
		SL = (Count-initCount) /100;

		PutStringSB("\r", 7);
	}
	PutStringSB("\r \r", 7);
	return 1;
}

int main (void) {

  __asm("CPSID   I");  /* mask interrupts */
  /* Perform all device initialization here */



  /* Before unmasking interrupts            */
  Init_UART0_IRQ ();
	Init_PIT_IRQ();
	Init_LED();
	setLED(1, 1, 1);

	
  __asm("CPSIE   I");  /* unmask interrupts */


  for (;;) { /* do forever */
  /* Put main program loop code here */
		
		setLED(1, 1, 1);
		//prompt and wait for user to begin game
		PutStringSB("Press SPACE to Begin", 79);
		RunStopWatch = 1;
		char begin = GetChar();
		while (begin != ' ') {
			begin = GetChar();
		}

		int score = 0;
		int maxRounds = 10;
		int nextRound = 1; 
		int roundNum = 1;
		PutChar('\r');
		while (roundNum <= maxRounds & nextRound == 1) {
			PutStringSB("round ", 78);
			PutNumUB(roundNum);
			int roundTime = 11-roundNum;
			PutStringSB(" (", 4);
			PutNumU(roundTime);
			PutStringSB(" s) beginning in: \n\r", 78);
			//pregame count down
			// gives user 2 seconds to prepare
			preGameTimer(2);
			
			// game started
			//get random LED
			int ledCOl = randint(3);
			setLedCol(ledCOl);
			
			//start round:
			
			ClearRxQueu(); //clear anything that might be in the recieve queue before the round starts
			UInt8 hasTyped = Typed;
			
			roundTime *= 100;
			Count = 0;
			int curCount = Count;
			char r;
			
			while (hasTyped == 0 & curCount <= roundTime) {
				hasTyped = Typed;
				curCount = Count;
				if (hasTyped == 1) {
					r = GetChar();
					if (charToLED(r) != ledCOl) {
						hasTyped = 0;
						PutChar(r);
						PutStringSB(" Incorrect. try again\n\r", 70);
					}
				}
				
			}
			if (hasTyped == 0) {
				PutStringSB("Out of time!\n\r", 78);
				nextRound = 1;
			} else {
				hasTyped = Typed;
				while (hasTyped == 1) {
					GetChar();
					hasTyped = Typed;
				}
				PutChar(r);
				PutStringSB(": ", 6);
				int response = charToLED(r);
				if (response == 4) {
					PutStringSB("invalid response. Scored: 0\n\r", 79);
					nextRound = 1;
				} else {
					if (response == ledCOl) {
						// entered correct color
						PutStringSB("Correct! Scored: ", 79);
						int curScore = calcScore(curCount, roundNum);
						PutNumU(curScore);
						PutStringSB("\n\r", 7);
						score += curScore;
						if (roundNum ==10) {
								PutStringSB("Congratulations, you have completed all rounds!\n\r", 90);
						}
						
						nextRound = 1;
						roundNum ++;
					}	else {
						PutStringSB("Incorrect. Scored: 0\n\r", 79);
						nextRound = 1;
					}
				}
			}
			
			
		}
		PutStringSB("Final score: ", 70);
		PutNumU(score);
		PutStringSB("\n\r", 8);
			




  } /* do forever */

  return (0);
} /* main */
