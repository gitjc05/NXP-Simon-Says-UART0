




            TTL ex11
;****************************************************************
;lab 12 project
;LED light game
;
;Name:  <Joaquin Cabra Romero>
;Date:  <11/19/2024>
;Class:  CMPE-250
;Section:  <#5, Thursday, 2:00pm>
;---------------------------------------------------------------
;Keil Template for KL05
;R. W. Melton
;September 13, 2020
;****************************************************************
;Assembler directives
            THUMB
            GBLL  MIXED_ASM_C
MIXED_ASM_C SETL  {TRUE}
            OPT    64  ;Turn on listing macro expansions
;****************************************************************
;Include files
			GET  MKL05Z4.s	;Included by start.s
            OPT  1   ;Turn on listing
;****************************************************************
;EQUates
;****************************************************************
;EQUates
;---------------------------------------------------------------
;Characters
BS          EQU  0x08
CR          EQU  0x0D
DEL         EQU  0x7F
ESC         EQU  0x1B
LF          EQU  0x0A
NULL        EQU  0x00
;---------------------------------------------------------------
;DAC0
DAC0_BITS   EQU   12
DAC0_STEPS  EQU   4096
DAC0_0V     EQU   0x00
;---------------------------------------------------------------
;Servo
SERVO_POSITIONS  EQU  5
;---------------------------------------------------------------
PWM_FREQ          EQU  50
;TPM_SOURCE_FREQ  EQU  48000000
TPM_SOURCE_FREQ   EQU  47972352
TPM_SC_PS_VAL     EQU  4
;PWM_PERIOD       EQU  ((TPM_SOURCE_FREQ / (1 << TPM_SC_PS_VAL)) / \;
;                       PWM_FREQ)
;PWM_DUTY_5       EQU  (PWM_PERIOD / 20)  ;  5% duty cycle
;PWM_DUTY_10      EQU  (PWM_PERIOD / 10)  ; 10% duty cycle
PWM_PERIOD        EQU  60000
PWM_DUTY_10       EQU  6000
PWM_DUTY_5        EQU  2000
;---------------------------------------------------------------
;Number output characteristics
MAX_WORD_DECIMAL_DIGITS  EQU  10
;---------------------------------------------------------------
; Queue management record field offsets
IN_PTR      EQU   0
OUT_PTR     EQU   4
BUF_STRT    EQU   8
BUF_PAST    EQU   12
BUF_SIZE    EQU   16
NUM_ENQD    EQU   17
; Queue structure sizes
XQ_BUF_SZ   EQU   80  ;Xmit queue contents
Q_REC_SZ    EQU   18  ;Queue management record
;---------------------------------------------------------------
;NVIC_ICER
;31-00:CLRENA=masks for HW IRQ sources;
;             read:   0 = unmasked;   1 = masked
;             write:  0 = no effect;  1 = mask
;22:PIT IRQ mask
;12:UART0 IRQ mask
NVIC_ICER_PIT_MASK    EQU  PIT_IRQ_MASK
NVIC_ICER_UART0_MASK  EQU  UART0_IRQ_MASK
;---------------------------------------------------------------
;NVIC_ICPR
;31-00:CLRPEND=pending status for HW IRQ sources;
;             read:   0 = not pending;  1 = pending
;             write:  0 = no effect;
;                     1 = change status to not pending
;22:PIT IRQ pending status
;12:UART0 IRQ pending status
NVIC_ICPR_PIT_MASK    EQU  PIT_IRQ_MASK
NVIC_ICPR_UART0_MASK  EQU  UART0_IRQ_MASK
;---------------------------------------------------------------
;NVIC_IPR0-NVIC_IPR7
;2-bit priority:  00 = highest; 11 = lowest
;--PIT--------------------
PIT_IRQ_PRIORITY    EQU  0
NVIC_IPR_PIT_MASK   EQU  (3 << PIT_PRI_POS)
NVIC_IPR_PIT_PRI_0  EQU  (PIT_IRQ_PRIORITY << PIT_PRI_POS)
;--UART0--------------------
UART0_IRQ_PRIORITY    EQU  3
NVIC_IPR_UART0_MASK   EQU (3 << UART0_PRI_POS)
NVIC_IPR_UART0_PRI_3  EQU (UART0_IRQ_PRIORITY << UART0_PRI_POS)
;---------------------------------------------------------------
;NVIC_ISER
;31-00:SETENA=masks for HW IRQ sources;
;             read:   0 = masked;     1 = unmasked
;             write:  0 = no effect;  1 = unmask
;22:PIT IRQ mask
;12:UART0 IRQ mask
NVIC_ISER_PIT_MASK    EQU  PIT_IRQ_MASK
NVIC_ISER_UART0_MASK  EQU  UART0_IRQ_MASK
;---------------------------------------------------------------
;PIT_LDVALn:  PIT load value register n
;31-00:TSV=timer start value (period in clock cycles - 1)
;Clock ticks for 0.01 s at ~24 MHz count rate
;0.01 s * ~24,000,000 Hz = ~240,000
;TSV = ~240,000 - 1
;Clock ticks for 0.01 s at 23,986,176 Hz count rate
;0.01 s * 23,986,176 Hz = 239,862
;TSV = 239,862 - 1
PIT_LDVAL_10ms  EQU  239861
;---------------------------------------------------------------
;PIT_MCR:  PIT module control register
;1-->    0:FRZ=freeze (continue'/stop in debug mode)
;0-->    1:MDIS=module disable (PIT section)
;               RTI timer not affected
;               must be enabled before any other PIT setup
PIT_MCR_EN_FRZ  EQU  PIT_MCR_FRZ_MASK
;---------------------------------------------------------------
;PIT_TCTRL:  timer control register
;0-->   2:CHN=chain mode (enable)
;1-->   1:TIE=timer interrupt enable
;1-->   0:TEN=timer enable
PIT_TCTRL_CH_IE  EQU  (PIT_TCTRL_TEN_MASK :OR: PIT_TCTRL_TIE_MASK)
;---------------------------------------------------------------
;PORTx_PCRn (Port x pin control register n [for pin n])
;___->10-08:Pin mux control (select 0 to 8)
;Use provided PORT_PCR_MUX_SELECT_2_MASK
;---------------------------------------------------------------
;Port B
PORT_PCR_SET_PTB2_UART0_RX  EQU  (PORT_PCR_ISF_MASK :OR: \
                                  PORT_PCR_MUX_SELECT_2_MASK)
PORT_PCR_SET_PTB1_UART0_TX  EQU  (PORT_PCR_ISF_MASK :OR: \
                                  PORT_PCR_MUX_SELECT_2_MASK)
;---------------------------------------------------------------
;SIM_SCGC4
;1->10:UART0 clock gate control (enabled)
;Use provided SIM_SCGC4_UART0_MASK
;---------------------------------------------------------------
;SIM_SCGC5
;1->09:Port B clock gate control (enabled)
;Use provided SIM_SCGC5_PORTB_MASK
;---------------------------------------------------------------
;SIM_SCGC6
;1->23:PIT clock gate control (enabled)
;Use provided SIM_SCGC6_PIT_MASK
;---------------------------------------------------------------
;SIM_SOPT2
;01=27-26:UART0SRC=UART0 clock source select (MCGFLLCLK)
;---------------------------------------------------------------
SIM_SOPT2_UART0SRC_MCGFLLCLK  EQU  \
                                 (1 << SIM_SOPT2_UART0SRC_SHIFT)
;---------------------------------------------------------------
;SIM_SOPT5
; 0->   16:UART0 open drain enable (disabled)
; 0->   02:UART0 receive data select (UART0_RX)
;00->01-00:UART0 transmit data select source (UART0_TX)
SIM_SOPT5_UART0_EXTERN_MASK_CLEAR  EQU  \
                               (SIM_SOPT5_UART0ODE_MASK :OR: \
                                SIM_SOPT5_UART0RXSRC_MASK :OR: \
                                SIM_SOPT5_UART0TXSRC_MASK)
;---------------------------------------------------------------
;UART0_BDH
;    0->  7:LIN break detect IE (disabled)
;    0->  6:RxD input active edge IE (disabled)
;    0->  5:Stop bit number select (1)
;00001->4-0:SBR[12:0] (UART0CLK / [9600 * (OSR + 1)]) 
;UART0CLK is MCGPLLCLK/2
;MCGPLLCLK is 96 MHz
;MCGPLLCLK/2 is 48 MHz
;SBR = 48 MHz / (9600 * 16) = 312.5 --> 312 = 0x138
UART0_BDH_9600  EQU  0x01
;---------------------------------------------------------------
;UART0_BDL
;26->7-0:SBR[7:0] (UART0CLK / [9600 * (OSR + 1)])
;UART0CLK is MCGPLLCLK/2
;MCGPLLCLK is 96 MHz
;MCGPLLCLK/2 is 48 MHz
;SBR = 48 MHz / (9600 * 16) = 312.5 --> 312 = 0x138
UART0_BDL_9600  EQU  0x38
;---------------------------------------------------------------
;UART0_C1
;0-->7:LOOPS=loops select (normal)
;0-->6:DOZEEN=doze enable (disabled)
;0-->5:RSRC=receiver source select (internal--no effect LOOPS=0)
;0-->4:M=9- or 8-bit mode select 
;        (1 start, 8 data [lsb first], 1 stop)
;0-->3:WAKE=receiver wakeup method select (idle)
;0-->2:IDLE=idle line type select (idle begins after start bit)
;0-->1:PE=parity enable (disabled)
;0-->0:PT=parity type (even parity--no effect PE=0)
UART0_C1_8N1  EQU  0x00
;---------------------------------------------------------------
;UART0_C2
;0-->7:TIE=transmit IE for TDRE (disabled)
;0-->6:TCIE=transmission complete IE for TC (disabled)
;0-->5:RIE=receiver IE for RDRF (disabled)
;0-->4:ILIE=idle line IE for IDLE (disabled)
;1-->3:TE=transmitter enable (enabled)
;1-->2:RE=receiver enable (enabled)
;0-->1:RWU=receiver wakeup control (normal)
;0-->0:SBK=send break (disabled, normal)
UART0_C2_T_R    EQU  (UART0_C2_TE_MASK :OR: UART0_C2_RE_MASK)
UART0_C2_T_RI   EQU  (UART0_C2_RIE_MASK :OR: UART0_C2_T_R)
UART0_C2_TI_RI  EQU  (UART0_C2_TIE_MASK :OR: UART0_C2_T_RI)
;---------------------------------------------------------------
;UART0_C3
;0-->7:R8T9=9th data bit for receiver (not used M=0)
;           10th data bit for transmitter (not used M10=0)
;0-->6:R9T8=9th data bit for transmitter (not used M=0)
;           10th data bit for receiver (not used M10=0)
;0-->5:TXDIR=UART_TX pin direction in single-wire mode
;            (no effect LOOPS=0)
;0-->4:TXINV=transmit data inversion (not inverted)
;0-->3:ORIE=overrun IE for OR (disabled)
;0-->2:NEIE=noise error IE for NF (disabled)
;0-->1:FEIE=framing error IE for FE (disabled)
;0-->0:PEIE=parity error IE for PF (disabled)
UART0_C3_NO_TXINV  EQU  0x00
;---------------------------------------------------------------
;UART0_C4
;    0-->  7:MAEN1=match address mode enable 1 (disabled)
;    0-->  6:MAEN2=match address mode enable 2 (disabled)
;    0-->  5:M10=10-bit mode select (not selected)
;01111-->4-0:OSR=over sampling ratio (16)
;               = 1 + OSR for 3 <= OSR <= 31
;               = 16 for 0 <= OSR <= 2 (invalid values)
UART0_C4_OSR_16           EQU  0x0F
UART0_C4_NO_MATCH_OSR_16  EQU  UART0_C4_OSR_16
;---------------------------------------------------------------
;UART0_C5
;  0-->  7:TDMAE=transmitter DMA enable (disabled)
;  0-->  6:Reserved; read-only; always 0
;  0-->  5:RDMAE=receiver full DMA enable (disabled)
;000-->4-2:Reserved; read-only; always 0
;  0-->  1:BOTHEDGE=both edge sampling (rising edge only)
;  0-->  0:RESYNCDIS=resynchronization disable (enabled)
UART0_C5_NO_DMA_SSR_SYNC  EQU  0x00
;---------------------------------------------------------------
;UART0_S1
;0-->7:TDRE=transmit data register empty flag; read-only
;0-->6:TC=transmission complete flag; read-only
;0-->5:RDRF=receive data register full flag; read-only
;1-->4:IDLE=idle line flag; write 1 to clear (clear)
;1-->3:OR=receiver overrun flag; write 1 to clear (clear)
;1-->2:NF=noise flag; write 1 to clear (clear)
;1-->1:FE=framing error flag; write 1 to clear (clear)
;1-->0:PF=parity error flag; write 1 to clear (clear)
UART0_S1_CLEAR_FLAGS  EQU  (UART0_S1_IDLE_MASK :OR: \
                            UART0_S1_OR_MASK :OR: \
                            UART0_S1_NF_MASK :OR: \
                            UART0_S1_FE_MASK :OR: \
                            UART0_S1_PF_MASK)
;---------------------------------------------------------------
;UART0_S2
;1-->7:LBKDIF=LIN break detect interrupt flag (clear)
;             write 1 to clear
;1-->6:RXEDGIF=RxD pin active edge interrupt flag (clear)
;              write 1 to clear
;0-->5:(reserved); read-only; always 0
;0-->4:RXINV=receive data inversion (disabled)
;0-->3:RWUID=receive wake-up idle detect
;0-->2:BRK13=break character generation length (10)
;0-->1:LBKDE=LIN break detect enable (disabled)
;0-->0:RAF=receiver active flag; read-only
UART0_S2_NO_RXINV_BRK10_NO_LBKDETECT_CLEAR_FLAGS  EQU  \
        (UART0_S2_LBKDIF_MASK :OR: UART0_S2_RXEDGIF_MASK)
;---------------------------------------------------------------
;---------------------------------------------------------------
UpA			EQU		65
UpZ			EQU		90
lowa		EQU		97
lowz		EQU		122
caseDif		EQU		32
MAX_LEN		EQU		79
	; Queue management record field offsets
; Queue structure sizes
Q_BUF_SZ    EQU   80 ;Queue buffer contents
Q_BUF_SZ2	EQU	  4
; Queue delimiters for printed output
Q_BEGIN_CH  EQU   '>'
Q_END_CH    EQU   '<'
	
;****************************************************************
;Program
;Linker requires Reset_Handler
            AREA    MyCode,CODE,READONLY
            ENTRY
			EXPORT		GetChar
			EXPORT 		PutChar
			EXPORT		GetStringSB
			EXPORT		Init_UART0_IRQ
			EXPORT		Init_PIT_IRQ
			EXPORT		PIT_IRQHandler
			EXPORT		UART0_IRQHandler
			EXPORT		PutNumHex
			EXPORT		PutNumUB
			EXPORT		PutNumU
			EXPORT		PutStringSB
			EXPORT		ClearRxQueu


			

;>>>>> begin subroutine code <<<<<
;;;;INITIALIZATION SUBROUTINE;;;;

;fix moving masks to applying masks

Init_UART0_IRQ      PROC	{R0-R14}
;Pre condition: unintialized registers and UART0
;Post condition: registers initialized, UART0 ready

        PUSH    {R0-R3,LR}
		
		LDR		R0,=TxQBuffer
		LDR		R1,=TxQRecord
		MOVS	R2,#Q_BUF_SZ
		BL		InitQueue
		
		LDR		R0,=RxQBuffer
		LDR		R1,=RxQRecord
		BL		InitQueue
		
;Select MCGFLLCLK as UART0 clock source
        LDR     R0,=SIM_SOPT2
        LDR     R1,=SIM_SOPT2_UART0SRC_MASK
        LDR     R2,[R0,#0]
        BICS    R2,R2,R1
        LDR     R1,=SIM_SOPT2_UART0SRC_MCGFLLCLK
        ORRS    R2,R2,R1
        STR     R2,[R0,#0]
;Set UART0 for external connection
        LDR     R0,=SIM_SOPT5
        LDR     R1,=SIM_SOPT5_UART0_EXTERN_MASK_CLEAR
        LDR     R2,[R0,#0]
        BICS    R2,R2,R1
        STR     R2,[R0,#0]
;Enable UART0 module clock
        LDR     R0,=SIM_SCGC4
        LDR     R1,=SIM_SCGC4_UART0_MASK
        LDR     R2,[R0,#0]
        ORRS    R2,R2,R1
        STR     R2,[R0,#0]
;Enable PORT B module clock
        LDR     R0,=SIM_SCGC5
        LDR     R1,=SIM_SCGC5_PORTB_MASK
        LDR     R2,[R0,#0]
        ORRS    R2,R2,R1
        STR     R2,[R0,#0]
;Select PORT B Pin 2 (D0) for UART0 RX (J8 Pin 01)
        LDR     R0,=PORTB_PCR2
        LDR     R1,=PORT_PCR_SET_PTB2_UART0_RX
        STR     R1,[R0,#0]
; Select PORT B Pin 1 (D1) for UART0 TX (J8 Pin 02)
        LDR     R0,=PORTB_PCR1
        LDR     R1,=PORT_PCR_SET_PTB1_UART0_TX
        STR     R1,[R0,#0]
;Disable UART0 receiver and transmitter
        LDR     R0,=UART0_BASE
        MOVS    R1,#UART0_C2_T_R
        LDRB    R2,[R0,#UART0_C2_OFFSET]
        BICS    R2,R2,R1
        STRB    R2,[R0,#UART0_C2_OFFSET]

;NVIC init
		LDR 	R0,=UART0_IPR
		;LDR R1,=NVIC_IPR_UART0_MASK
		LDR 	R2,=NVIC_IPR_UART0_PRI_3
		LDR 	R3,[R0,#0]
		;BICS R3,R3,R1
		ORRS 	R3,R3,R2
		STR 	R3,[R0,#0]
;Clear any pending UART0 interrupts
		LDR 	R0,=NVIC_ICPR
		LDR 	R1,=NVIC_ICPR_UART0_MASK
		STR 	R1,[R0,#0]
;Unmask UART0 interrupts
		LDR 	R0,=NVIC_ISER
		LDR 	R1,=NVIC_ISER_UART0_MASK
		STR 	R1,[R0,#0]
		
;Set UART0 for 9600 baud, 8N1 protocol
        LDR     R0,=UART0_BASE
        MOVS    R1,#UART0_BDH_9600
        STRB    R1,[R0,#UART0_BDH_OFFSET]
        MOVS    R1,#UART0_BDL_9600
        STRB    R1,[R0,#UART0_BDL_OFFSET]
        MOVS    R1,#UART0_C1_8N1
        STRB    R1,[R0,#UART0_C1_OFFSET]
        MOVS    R1,#UART0_C3_NO_TXINV
        STRB    R1,[R0,#UART0_C3_OFFSET]
        MOVS    R1,#UART0_C4_NO_MATCH_OSR_16
        STRB    R1,[R0,#UART0_C4_OFFSET]
        MOVS    R1,#UART0_C5_NO_DMA_SSR_SYNC
        STRB    R1,[R0,#UART0_C5_OFFSET]
        MOVS    R1,#UART0_S1_CLEAR_FLAGS
        STRB    R1,[R0,#UART0_S1_OFFSET]
        MOVS    R1, \
                #UART0_S2_NO_RXINV_BRK10_NO_LBKDETECT_CLEAR_FLAGS
        STRB    R1,[R0,#UART0_S2_OFFSET]
		
;Enable UART0 receiver and transmitter
		LDR		R0,=UART0_BASE
        MOVS    R1,#UART0_C2_T_RI
		LDRB	R2,[R0,#UART0_C2_OFFSET]
		ORRS	R1,R1,R2
        STRB    R1,[R0,#UART0_C2_OFFSET]

		
        POP     {R0-R3,PC}
        ENDP
			
Init_PIT_IRQ	PROC	{R0-R14}
        PUSH    {R0-R3}

		LDR     R0,=SIM_SCGC6
        LDR     R1,=SIM_SCGC6_PIT_MASK
        LDR     R2,[R0,#0]
        ORRS    R2,R2,R1
        STR     R2,[R0,#0]
        ;Disable PIT timer 0
        LDR     R0,=PIT_CH0_BASE
        LDR     R1,=PIT_TCTRL_TEN_MASK
        LDR     R2,[R0,#PIT_TCTRL_OFFSET]
        BICS    R2,R2,R1
        STR     R2,[R0,#PIT_TCTRL_OFFSET]

        ;Set PIT interrupt priority
        LDR 	R0,=PIT_IPR
        LDR 	R1,=NVIC_IPR_PIT_MASK
        ;LDR R2,=NVIC_IPR_PIT_PRI_0
        LDR 	R3,[R0,#0]
        BICS 	R3,R3,R1
        ;ORRS Rl,Rl,R2
        STR 	R3,[R0,#0]
        ;Clear any pending PIT interrupts
        LDR 	R0,=NVIC_ICPR
        LDR 	R1,=NVIC_ICPR_PIT_MASK
        STR 	R1,[R0,#0]
        ;Unmask PIT interrupts
        LDR 	R0,=NVIC_ISER
        LDR 	R1,=NVIC_ISER_PIT_MASK
        STR 	R1,[R0,#0]
        ;Enable PIT module
        LDR 	R0,=PIT_BASE
        LDR 	R1,=PIT_MCR_EN_FRZ
        STR 	R1,[R0,#PIT_MCR_OFFSET]
        ;Set PIT timer 0 period for 0.01 s
        LDR 	R0,=PIT_CH0_BASE
        LDR 	R1,=PIT_LDVAL_10ms
        STR 	R1,[R0,#PIT_LDVAL_OFFSET]
        ;Enable PIT timer 0 interrupt
        LDR 	R1,=PIT_TCTRL_CH_IE
        STR 	R1,[R0,#PIT_TCTRL_OFFSET]
		POP		{R0-R3}
		BX		LR
		ENDP

;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;;;;;;PUT CHAR;;;;;;;;;;

PutChar PROC	{R0-R14}
;
		PUSH	{R0-R2,LR}
loopPC
		CPSID	I
		LDR		R1,=TxQRecord
		BL		Enqueue
		CPSIE	I
		BCS		loopPC
		
		MOVS    R1,#UART0_C2_TI_RI
		LDR		R0,=UART0_BASE
		STRB    R1,[R0,#UART0_C2_OFFSET]
		
		POP 	{R0-R2,PC}
		
		ENDP


;;;;;;;;GET CHAR;;;;;;;;;;

GetChar PROC	{R1-R14}
;      
		PUSH	{R1,LR}
loopGC
		CPSID	I
		LDR		R1,=RxQRecord
		BL		Dequeue
		CPSIE	I
		BCS		loopGC
		LDRB	R1,[R1,#NUM_ENQD]
		CMP		R1,#0
		BNE		endGetChar
		LDR		R1,=Typed
		PUSH	{R2}
		MOVS	R2,#0
		STRB	R2,[R1]
		POP		{R2}
endGetChar
		POP		{R1,PC}
		ENDP

ClearRxQueu	PROC	{R0-R14}
	; no precondition
	; post condition: emptied RxQueue
	; clears recvie queue and sets typed to 0
	; used for reseting RxQueue before the begining of a new round for game
	PUSH 	{R1-R2, LR}
	LDR		R1,=RxQRecord
loopcrq
	BL		Dequeue
	BCS		donecrq
	B		loopcrq
donecrq
	LDR		R1,=Typed
	MOVS	R2,#0
	STR		R2,[R1]
	POP		{R1-R2, PC}
	ENDP
	
	
	
PIT_IRQHandler	 PROC    {R0-R14}

		CPSID   I
        ;load runstopwatch
        PUSH    {R0-R2,LR}
        
        LDR     R0,=RunStopWatch
        LDRB    R1,[R0]
        ; load mask
        CMP     R1,#0
        BEQ     clrPIT
        MOVS    R0,R1
        LDR     R2,=Count
        LDR     R1,[R2]
        ADDS    R1,R1,#1
        STR     R1,[R2]
		LDR     R2,=MainClock
        LDR     R1,[R2]
        ADDS    R1,R1,#1
        STR     R1,[R2]

clrPIT
        LDR     R0,=PIT_TFLG0
        LDR     R1,=PIT_TFLG_TIF_MASK
        STR     R1,[R0]
        CPSIE   I
        POP     {R0-R2,PC}
		ENDP


UART0_IRQHandler	PROC	{R0-R14}
		CPSID	I
		PUSH	{R0-R3,LR}
		LDR		R0,=UART0_BASE
		LDRB	R1,[R0,#UART0_C2_OFFSET]
		MOVS	R2,#UART0_C2_TIE_MASK
		ANDS	R1,R1,R2
		BEQ		checkRead	;if disabled
		LDRB	R1,[R0,#UART0_S1_OFFSET]
		MOVS	R2,#UART0_S1_TDRE_MASK
		ANDS	R1,R1,R2
		BEQ		checkRead	;if disabled
		;deque and print
		LDR		R1,=TxQRecord
		BL		Dequeue
		BCS		disableInteruptISR
		LDR		R3,=UART0_BASE
		STRB	R0,[R3,#UART0_D_OFFSET]
		B 		returnISR
	
disableInteruptISR
		MOVS	R0,#UART0_C2_T_RI
		LDR		R1,=UART0_BASE
		STRB	R0,[R1,#UART0_C2_OFFSET]
		B 		returnISR

checkRead
		LDRB	R1,[R0,#UART0_S1_OFFSET]
		MOVS	R2,#UART0_S1_RDRF_MASK
		ANDS	R1,R1,R2
		BEQ		returnISR	;if disabled		
		LDRB	R3,[R0,#UART0_D_OFFSET]
		LDR		R1,=RxQRecord
		MOVS	R0,R3
		BL		Enqueue
		LDR		R1,=Typed
		MOVS	R2,#1
		STRB	R2,[R1]
		B  		returnISR
returnISR
		
		CPSIE	I
		POP		{R0-R3,PC}
		BX		LR
		ENDP
		


;
;;DIVU
DIVU      PROC {R2-R14}
; Divider
;   Pre condition:  R0 contains the denominator,        R1 contains the numerator
;   Post condition: R0 contains the result (quotient),  R1 contains the remainder c flag set if attempted division by 0, clear otherwise
; uses subtraction loop till no more divisors can be taken from what was initialy dividend
; allows for easy remainder
; c flag set or cleared if a/0 or a/b
; Answer=>r0, Remainder=>R1

            PUSH    {R2}                ; psuh r2
            MOVS    R2,#0               ; intialize answer reg (quotient)
            CMP     R0,#0               ; check for zero div
            BEQ     divZero             ; branch to zero div

loopdu       
            CMP     R1,R0               ; compare dividend, divisor
            BLO     endLoopdu           ; if dividend<Divisor go to endloop stage
            ADDS    R2,R2,#1            ; increment quotienbt (answer)
            SUBS    R1,R1,R0            ; dividend - divisor
            B       loopdu              ; loop

endLoopdu    
            MOVS    R0,R2               ; remainders in r1, answer in r2, so move answer (quotient) to r0
            POP     {R2}                ; reset r2
            PUSH    {R0,R1}             ; push r0 and r1 to stack (about to use to set C flag)
            MRS     R0,APSR             ; move specials to r0 --C Flag
            MOVS    R1,#0x2      		; make carry mask (gonna be inverted through BICS)
			LSLS	R1,R1,#28
            BICS    R0,R0,R1            ; apply inverted carry mask to make new specials val making c=0
            MSR     APSR,R0             ; set specials val to what i made in r0
            POP     {R0,R1}             ; put answer and quotient back in r0 and r1

exit        BX      LR                  ; returns right after loop if a/b, else divZero calls if a/0

divZero    
            PUSH    {R0,R1}             ; push r0 and r1, gonna use for c flag set
            MRS     R0,APSR             ; move specials to r0 --C Flag
            MOVS    R1,#0x2		        ; make mask for carry
			LSLS	R1,#28
            ORRS    R0,R0,R1            ; use mask to make specials val with C=1
            MSR     APSR,R0             ; move val to specials
            POP     {R0,R1,R2}          ; put inputs back in r0 and r1, and reset r2
            B       exit                ; return


                    ENDP
;PutStringSB
PutStringSB     PROC    {R0-R14}

;Input: R0: begining memory address for string to print
        ;R1 String length (including null at the end)
;Output: None
;Modifies: APSR

        PUSH    {R2-R4,LR}
        
        MOVS    R3,R0
        MOVS    R2,#0
        LDRB    R0,[R3, R2]
        ADDS    R2,R2,#1
loopp
        CMP     R0,#0
        BEQ     exitlp 
		CMP		R2,#MAX_LEN
		BHI		exitlp
        BL      PutChar
        LDRB    R0,[R3,R2]
        ADDS    R2,R2,#1
		B		loopp
exitlp
        POP     {R2-R4,PC}
        ENDP



GetStringSB     PROC    {R0-R14}
;Input: R0: address of string buffer in memory for inptu from user (unsigned word)
;
;Out: R0: memory address for the stored string
;Modifies: APSR

        PUSH    {R1-R5,LR}
        MOVS    R2,#0

        MOVS    R3,R0


        BL      GetChar
loopg
        CMP     R0,#13
        BEQ     exitlg
		CMP		R2,R1
		BEQ		exitlmax
        BL      PutChar
        STRB    R0,[R3,R2]
        ADDS    R2,R2,#1

exitlmax 
        BL      GetChar
		B       loopg
exitlg
        MOVS    R4,#0
	
        STRB    R4,[R3,R2]
        BL      PutChar
        MOVS    R0,#10
        BL      PutChar
        MOVS    R0,R3

        POP     {R1-R5,PC}
        ENDP

;
;
;PutNumU
PutNumU         PROC    {R0-R14} ;;*Written to NOT print leading zeroes*;;
;Input: R0: Unisgned word value to print
;Output: None
;Modifies: APSR

; get num
; have addr in mem
; loop divU R0/10 till quotient=0
; remainders stored in mem each time, offset = size
; remainders = digits backwards, offset ->0
; stores number val in memory @startP
        PUSH    {R1-R5,LR}

        LDR     R3,=startP
        MOVS    R2,#0

        MOVS    R5,R0           ;move r5 back into r0
        MOVS    R0,#10
        MOVS    R1,R5
		CMP		R1,#0
		BEQ		exitlooplnd

        BL      DIVU
looplnd
        CMP     R0,#0
        BEQ     exitlooplnd
        STRB    R1,[R3,R2]
        ADDS    R2,R2,#1
        MOVS    R1,R0
        MOVS    R0,#10
        BL      DIVU
        B       looplnd

exitlooplnd
        STRB    R1,[R3,R2]

looplnp

        LDRB    R0,[R3,R2]
		ADDS	R0,R0,#0x30
        BL      PutChar
        CMP     R2,#0
        BEQ     exitlooplnp
		SUBS    R2,R2,#1
		B		looplnp

exitlooplnp    
        
        MOVS    R0,R5
        POP     {R1-R5,PC}
        ENDP



InitQueue PROC {R0,R2-R14}
;intilizes queue
;R1:record, R0: buf, R2: size
;no pre condition 
;post condition: queue is initilized and ready for use
        PUSH {R0-R2}

        STR     R0,[R1,#IN_PTR]
        STR     R0,[R1,#OUT_PTR]
        STR     R0,[R1,#BUF_STRT]
        ADDS    R0,R0,R2
        STRB    R2,[R1,#BUF_SIZE]
        STR     R0,[R1,#BUF_PAST]
        MOVS    R0,#0
        STRB    R0,[R1,#NUM_ENQD]


        POP     {R0-R2}
        BX      LR
        ENDP
			
		

Dequeue PROC {R1-R14}
;dequeue operation
;takes in record pointer in R1
;Deques out pointer value to R0
;sets C flag if operation failure, otherwise clears it
    PUSH    {R2-R5}

    LDRB    R2,[R1,#NUM_ENQD]
    CMP     R2,#0
    BEQ     Dequeueemptydq
    ;deque into r0, clear c, and return
	
    LDR     R3,[R1,#OUT_PTR]

    LDRB    R0,[R3]
    SUBS    R2,R2,#1
	MOVS	R5,R2
    STRB    R2,[R1,#NUM_ENQD]
    ADDS    R3,R3,#1
	LDR		R4,[R1,#BUF_PAST]
    CMP     R3,R4
    BLO     Dequeueoutready
    LDR     R3,[R1,#BUF_STRT]
	CMP		R5,#0
	BNE		Dequeueoutready
	LDR		R3,[R1,#BUF_STRT]
	STR		R3,[R1,#IN_PTR]
	
Dequeueoutready
    STR     R3,[R1,#OUT_PTR]

	MRS     R4,APSR             ; move specials to r0 --C Flag
    MOVS    R3,#0x2      		; make carry mask (gonna be inverted through BICS)
	LSLS	R3,R3,#28
    BICS    R4,R4,R3            ; apply inverted carry mask to make new specials val making c=0
    MSR     APSR,R4             ; set specials val to what i made in r0
    POP     {R2-R5}
    BX      LR

Dequeueemptydq

	LDR		R0,[R1,#BUF_STRT]
	STR		R0,[R1,#IN_PTR]
	STR		R0,[R1,#OUT_PTR]
    ;set c flag and return
	
    MRS     R4,APSR             ; move specials to r0 --C Flag
    MOVS    R3,#0x2		        ; make mask for carry
	LSLS	R3,#28
    ORRS    R4,R4,R3            ; use mask to make specials val with C=1
    MSR     APSR,R4             ; move val to specials
    POP     {R2-R5}
    BX      LR
    ENDP


Enqueue PROC {R0-R14}
;takes in character ASCII value in R0, and record pointer at R1
;attempts to queue value
;returns C flag set if failed attempt, else C flag cleared
    PUSH    {R2-R5}

    LDRB    R2,[R1,#NUM_ENQD] 
    CMP     R2,#Q_BUF_SZ
    BEQ     Enqueuefulleq 

    ADDS    R2,R2,#1
    STRB    R2,[R1,#NUM_ENQD]
	MOVS	R5,R2
    LDR     R2,[R1,#IN_PTR]
    LDR     R3,=QBuffer
    STRB    R0,[R2]

    ADDS    R2,R2,#1
    LDR		R4,[R1,#BUF_PAST]

    CMP     R2,R4
    BLO     Enqueueinready
    LDR     R2,[R1,#BUF_STRT]
Enqueueinready
    STR     R2,[R1,#IN_PTR]

	MRS     R4,APSR             ; move specials to r0 --C Flag
    MOVS    R3,#0x2      		; make carry mask (gonna be inverted through BICS)
	LSLS	R3,R3,#28
    BICS    R4,R4,R3            ; apply inverted carry mask to make new specials val making c=0
    MSR     APSR,R4             ; set specials val to what i made in r0
    POP     {R2-R5}
    BX      LR

Enqueuefulleq
    ;set c flag and return
    MRS     R4,APSR             ; move specials to r0 --C Flag
    MOVS    R3,#0x2		        ; make mask for carry
	LSLS	R3,#28
    ORRS    R4,R4,R3            ; use mask to make specials val with C=1
    MSR     APSR,R4             ; move val to specials
    POP     {R2-R5}
    BX      LR
    ENDP



PutNumHex PROC {R0-R14}
;takes in value in R0
;prints hex value
;no registers changed
    PUSH    {R1-R5,LR}
    LDR     R3,=0xF0000000  ;initial mask biggest bit (will shift Right)
    MOVS    R2,#28          ;intiial shift value (from mask loc -> lowest btis to be read b yprinter)

PutNumHexloophx          ;loop the remasking & printing
    MOVS    R4,R0       ;make copy of og num
    ANDS    R4,R4,R3    ;apply cur mask
    LSRS    R4,R4,R2    ;shift right to be @ lowest bit spot
    
PutNumHexhexPrinter
	MOVS    R5,R4   ;copy for da operations
    CMP     R5,#9   ;check if dig
    BLS     PutNumHexdigconv    ;convert to dig ASCII if <=
    ADDS    R5,R5,#55   ;convert to char ASCII val else
    B       PutNumHexprinthx
PutNumHexdigconv
    ADDS    R5,#0x30    ;convert to digit ACII

PutNumHexprinthx
	MOVS	R4,R0
	MOVS	R0,R5
    BL      PutChar
	MOVS	R0,R4

    LSRS    R3,R3,#4    ;shift mask to next hex spot
    SUBS    R2,R2,#4    ;decrement the shifter amount to reflect ^
    CMP     R2,#0
    BGE     PutNumHexloophx      ;if shifter val is still valid continue
    

    POP     {R1-R5,PC}   ;shifter val is no longer valid (negative). return

	ENDP
		

PutNumUB PROC {R0-R14}
;takes in value to print in R0
;prints value, no registers changed
    PUSH    {R1-R2,LR}        
    MOVS    R1,R0              
    LDR     R0,=0x000000FF        ;mask the least bits that i want
    ANDS    R0,R1,R0                ;apply mask
    BL      PutNumU                 ;pass in the num (in r0) to printer
    MOVS    R0,R1             
    POP     {R1-R2,PC}        
    ENDP


;>>>>>   end subroutine code <<<<<
            ALIGN
;****************************************************************
;Vector Table Mapped to Address 0 at Reset
;Linker requires __Vectors to be exported
;
;****************************************************************
;Constants
            AREA    MyConst,DATA,READONLY

	
;>>>>> begin constants here <<<<<



;>>>>>   end constants here <<<<<
            ALIGN
;****************************************************************
;Variables
            AREA    MyData,DATA,READWRITE
			EXPORT	Count
			EXPORT	RunStopWatch
			EXPORT	Typed
			EXPORT	MainClock
;>>>>> begin variables here <<<<<

startP      SPACE       10
	        ALIGN
opString	SPACE		79
	        ALIGN
RxQBuffer   SPACE       Q_BUF_SZ
	        ALIGN
TxQBuffer   SPACE       Q_BUF_SZ
	        ALIGN
RxQRecord   SPACE       Q_REC_SZ
		    ALIGN
TxQRecord   SPACE       Q_REC_SZ
			ALIGN
QRecord   	SPACE       Q_REC_SZ
			ALIGN
QBuffer   	SPACE       Q_BUF_SZ2
                        ALIGN
Count           SPACE           8
                        ALIGN
MainClock		SPACE			16
						ALIGN
RunStopWatch    SPACE           1
						ALIGN
Typed		SPACE				1
;>>>>>   end variables here <<<<<
            ALIGN
            END



