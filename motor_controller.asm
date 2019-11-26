;**************************************************************************
;*           Fredrik Åhman   High-Q KIT  K8048/VM111 Motor speed controller    *
;*           CPU: 16F648A,  Capacity 4K instructions, 256 internal flash  mem      *
;*                                                                                                            *
;**************************************************************************
;*          Hardw. Rev: P8048'1          Softw. Rev:  1.00                                    *
;*          OSC.......: XT 4MHz Max.     POWER.....:  5V DC                                *
;*												  *
;**************************************************************************

;----- Temporary registers------------------------------------------------------

W                            EQU     H'0000'
F                            EQU     H'0001'

;----- Internal registers------------------------------------------------------

INDF                         EQU     H'0000'
TMR0                         EQU     H'0001'
PCL                          EQU     H'0002'
STATUS                       EQU     H'0003'
INT_VEC                      EQU     H'0004'
PORTA                        EQU     H'0005'
PORTB                        EQU     H'0006'
T1CON						 EQU	 H'0010'
T2CON						 EQU	 H'0012'
INTCON                       EQU     H'000B'
PIR1						 EQU	 H'000C'
PIE1						 EQU	 H'008C'
TMR1L						 EQU	 H'000E'
TMR1H					     EQU	 H'000F'
TMR2						 EQU	 H'0011'
OPTION_REG                   EQU     H'0081'
TRISA                        EQU     H'0085'
TRISB                        EQU     H'0086'
CMCON                        EQU     H'001F'
CCPR1L						 EQU	 H'0015'
CCPR1H						 EQU	 H'0016'
CCP1CON						 EQU	 H'0017'

PR2							 EQU	 H'0092'

;----- STATUS Bits --------------------------------------------------------
IRP                          EQU     H'0007'
RP1                          EQU     H'0006'
RP0                          EQU     H'0005'
NOT_TO                       EQU     H'0004'
NOT_PD                       EQU     H'0003'
Z                            EQU     H'0002'
DC                           EQU     H'0001'
C                            EQU     H'0000'


;==========================================================================
;
;       RAM Definition
;
;==========================================================================

    __MAXRAM H'01FF'
    __BADRAM H'07'-H'09', H'0D', H'13'-H'14', H'1B'-H'1E'
    __BADRAM H'87'-H'89', H'8D', H'8F'-H'91', H'93'-H'97', H'9E'
    __BADRAM H'105', H'107'-H'109', H'10C'-H'11F', H'150'-H'16F'
    __BADRAM H'185', H'187'-H'189', H'18C'-H'1EF'

;==========================================================================
;
;       Configuration Bits
;
;==========================================================================

_BODEN_ON                    EQU     H'3FFF'
_BODEN_OFF                   EQU     H'3FBF'
_CP_ALL                      EQU     H'03FF'
_CP_75                       EQU     H'17FF'
_CP_50                       EQU     H'2BFF'
_CP_OFF                      EQU     H'3FFF'
_DATA_CP_ON                  EQU     H'3EFF'
_DATA_CP_OFF                 EQU     H'3FFF'
_PWRTE_OFF                   EQU     H'3FFF'
_PWRTE_ON                    EQU     H'3FF7'
_WDT_ON                      EQU     H'3FFF'
_WDT_OFF                     EQU     H'3FFB'
_LVP_ON                      EQU     H'3FFF'
_LVP_OFF                     EQU     H'3F7F'
_MCLRE_ON                    EQU     H'3FFF'
_MCLRE_OFF                   EQU     H'3FDF'
_ER_OSC_CLKOUT               EQU     H'3FFF'
_ER_OSC_NOCLKOUT             EQU     H'3FFE'
_INTRC_OSC_CLKOUT            EQU     H'3FFD'
_INTRC_OSC_NOCLKOUT          EQU     H'3FFC'
_EXTCLK_OSC                  EQU     H'3FEF'
_LP_OSC                      EQU     H'3FEC'
_XT_OSC                      EQU     H'3FED'
_HS_OSC                      EQU     H'3FEE'

	
__CONFIG	_BODEN_ON & _CP_OFF & _DATA_CP_OFF & _PWRTE_ON & _WDT_OFF & _LVP_OFF & _MCLRE_ON & _INTRC_OSC_NOCLKOUT

;==========================================================================
;       Variable Definitions
;==========================================================================

TIMER1		EQU	H'20'		;Used in delay routine
TIMER2		EQU	H'21'		; "	"	"	
PATERN		EQU	H'22'		;Pattern data for effect's
PORTA_WORD_TO_SERIAL	EQU	H'23'
PORTA_BITCOUNT	EQU H'24'
INPUT_REGISTER	EQU	H'25'
KEY_INPUT		EQU	H'26'
DIGIT_COUNT		EQU	H'27'

FIRST_DIGIT		EQU	H'28'
SECOND_DIGIT	EQU	H'29'
THIRD_DIGIT		EQU H'30'

P_VALUE			EQU	H'31'
I_RES_VALUE		EQU H'3A'
PROD_VALUE		EQU H'3E'
	

MUL_OP_1		EQU	H'32'
MUL_OP_2		EQU H'33'
MUL_PROD		EQU H'34'

DIV_NUM			EQU	H'2A'
DIV_DENOM		EQU	H'2B'
DIV_RESULT		EQU	H'2C'
DIV_REMINDER	EQU H'2D'

MUL_BIT_COUNT	EQU H'35'
MUL_BIT_NUM		EQU	H'36'

MUL_TEMP_P		EQU H'37'
INV_CH_SEL		EQU	H'38'
TP_COUNT		EQU	H'39'

UNLIM_VAL_IN	EQU	H'3B'
LIM_VAL_OUT		EQU	H'3C'
VAL_RANGE		EQU H'3D'

HUN_NUM			EQU	H'42'
DEC_NUM			EQU	H'43'
ONE_NUM			EQU	H'44'

INPUT_HEX_NUM	EQU H'45'
TEMP_HEX_VALUE  EQU H'46'

NUM_TO_ASCII	EQU	H'47'

UP_DEC_CH_1ST	EQU	H'48'
LO_DEC_CH_1ST	EQU H'49'

UP_DEC_CH_2ND	EQU H'50'
LO_DEC_CH_2ND	EQU H'51'

UP_ONE_CH_1ST	EQU	H'52'
LO_ONE_CH_1ST	EQU	H'53'

UP_ONE_CH_2ND	EQU	H'54'
LO_ONE_CH_2ND	EQU	H'55'


TIMER2_TICKS	EQU	H'56'
SET_MOTOR_RPM	EQU H'57'
EPSILON			EQU H'58'
STEER_SIGNAL	EQU H'59'
MOTOR_SIGNAL	EQU H'4A'
STORED_STATUS	EQU H'4B'
I_TICK_100_MS	EQU	H'4C'  	
LIM_STATUS		EQU	H'4D'

;=======================================================================================================================
; Program startadress and GOTO-instruction
;=======================================================================================================================
ORG		0x0000	;Start vector address
GOTO	MAIN

;=======================================================================================================================
; External and peripheral interrupts
;=======================================================================================================================
ORG     0x0004               ; Interrupt vector address
				MOVLW	B'00000000' ; Disable external interrupt, RB0
				MOVWF	INTCON
				BSF		PORTB,5
				BTFSC   PIR1,1 ; Check if timer interrupt was triggered
				CALL 	PROCESS_TIMER2_INTERRUPT
				BTFSC   PORTB,0; Check if external interrupt (knob or key pressed) was triggered
				CALL 	PROCESS_EXT_INTERRUPT
				;MOVF	STEER_SIGNAL,W
			    ;MOVWF	TIMER2
				;CALL	MOTOR_DELAY
				BCF		PORTB,5
				MOVLW	B'11010000' ; Re-enable external and peripheral interrupts 
				MOVWF	INTCON
				RETFIE

;==========================================================================================================================
; Process timer2 interrupt
;==========================================================================================================================
PROCESS_TIMER2_INTERRUPT:
				MOVLW	D'01'
				MOVWF	W
				ADDWF	TIMER2_TICKS,1 ; Timer ticks +=1 
				MOVF    TIMER2_TICKS,W
				SUBLW	D'12'          
				BTFSC	STATUS,Z ; Timer ticks == 12 ?
				CALL	DISPLAY_ROTOR_RPM ; [Yes: Read rotor count, No: Continue]
				BCF     PIR1,1
				RETURN
	

;=========================================================================================================================
; Process external interrupts from keyboards, rotary sensors etc
;=========================================================================================================================	
PROCESS_EXT_INTERRUPT:
                BTFSS   PORTA,7
				CALL    INC_MOTOR_SPEED
				BTFSC   PORTA,7
				CALL    DEC_MOTOR_SPEED		
				CALL	PRINT_SET_RPM
				;CALL	MEDIUM_DELAY
				CALL 	TMR2_RESET
				RETURN

INC_MOTOR_SPEED:
                MOVF    SET_MOTOR_RPM,W
				SUBLW	D'120'          
				BTFSS	STATUS,Z ; Motor rpm == 120 ?  
				INCF	SET_MOTOR_RPM,1
				RETURN
				
DEC_MOTOR_SPEED:
				MOVF    SET_MOTOR_RPM,W
				SUBLW	D'1'          
				BTFSS	STATUS,Z ; Motor rpm == 1 ?  
				DECF	SET_MOTOR_RPM,1
				RETURN

PRINT_SET_RPM:				
				CALL	LCD_CUR_SEC_ROW
				MOVF	SET_MOTOR_RPM,W ; Convert set rpm HEX value to DECIMAL readable value
				MOVWF   INPUT_HEX_NUM
				CALL 	CONVERT_HEX_TO_DEC
				CALL    PRINT_SET
				;CALL	PRINT_SPACE
                ;CALL    PRINT_RPM
				CALL	PRINT_KOLON
				CALL	PRINT_DEC_VAL
				CALL 	PRINT_00
				RETURN


LIMIT_INPUT_VAL:	
				BTFSC   UNLIM_VAL_IN,7
				GOTO	ADJ_NEG_INP
	ADJ_POS_INP:
				MOVF	UNLIM_VAL_IN,W
				SUBWF	VAL_RANGE,0
				BTFSC   STATUS,C
				GOTO	USE_ORIG_INP
				MOVF	VAL_RANGE,W
				MOVWF	LIM_VAL_OUT
				GOTO    END_LIMIT_VAL
   USE_ORIG_INP:
				MOVF	UNLIM_VAL_IN,W
				MOVWF   LIM_VAL_OUT
				GOTO    END_LIMIT_VAL
	ADJ_NEG_INP:
				MOVF	UNLIM_VAL_IN,W
				ADDWF	VAL_RANGE,0
				BTFSC   STATUS,C
				GOTO    USE_ORIG_INP
				MOVF	VAL_RANGE,W
				SUBLW   D'0'
				MOVWF	LIM_VAL_OUT
	END_LIMIT_VAL:
                RETURN


SUB_PROD_VAL:
				MOVF	PROD_VALUE,W
				SUBLW	D'120'	
				RETURN
			
CALC_I_RES_VAL:
                CLRF	I_TICK_100_MS
				MOVF	EPSILON,W
				MOVWF	UNLIM_VAL_IN
				MOVLW	D'5'
				MOVWF	VAL_RANGE
				CALL	LIMIT_INPUT_VAL
				MOVF	LIM_VAL_OUT,W
				ADDWF 	I_RES_VALUE,1
				BTFSS   LIM_VAL_OUT,7
				CALL	SUB_PROD_VAL
				BTFSC	LIM_VAL_OUT,7
				MOVLW	D'120'
				MOVWF	VAL_RANGE
				MOVF	I_RES_VALUE,W
				MOVWF	UNLIM_VAL_IN
				CALL	LIMIT_INPUT_VAL
				
				BTFSC   LIM_VAL_OUT,7
				BCF		STORED_STATUS,C
				BTFSS   LIM_VAL_OUT,7
				BSF		STORED_STATUS,C
				
				MOVF	LIM_VAL_OUT,W
				MOVWF	I_RES_VALUE
				RETURN
				
				
CALC_EPSILON:
				MOVF	TMR1L,W
				SUBWF	SET_MOTOR_RPM,0
				MOVWF	EPSILON
				MOVF	STATUS,W
				MOVWF	STORED_STATUS
				RETURN
	
CALC_STEER_SIGNAL:  
                INCF	I_TICK_100_MS
                MOVF	EPSILON,W
				MOVWF	UNLIM_VAL_IN
				MOVLW	D'40'
				MOVWF	VAL_RANGE
				CALL	LIMIT_INPUT_VAL
			    MOVF	P_VALUE,W
				MOVWF	MUL_OP_1
				MOVF	LIM_VAL_OUT,W
				MOVWF	MUL_OP_2
				CALL	MULTIPLY	
				MOVF	MUL_PROD,W
				MOVWF	UNLIM_VAL_IN
				MOVLW	D'120'
				MOVWF	VAL_RANGE
				CALL	LIMIT_INPUT_VAL
				MOVF	LIM_VAL_OUT,W
				MOVWF	PROD_VALUE
				MOVF	PROD_VALUE,W
				ADDWF   I_RES_VALUE,0
				MOVWF	STEER_SIGNAL
				MOVF	STATUS,W
				MOVWF	STORED_STATUS
				
				BTFSC	STEER_SIGNAL,7
				BCF		STORED_STATUS,C
				BTFSS	STEER_SIGNAL,7
				BSF		STORED_STATUS,C
				
				MOVLW	D'5'
				SUBWF	I_TICK_100_MS,0
				BTFSC	STATUS,Z
				CALL	CALC_I_RES_VAL
				BTFSC	STEER_SIGNAL,7
				CLRF	STEER_SIGNAL
				RETURN
			
	
				
				
					
;========================================================================================================================
; Present motor speed on display
; 
; Input : TMR1L - Motor speed in hex format using factor (x 100 RPM)
; Output: None
;
;========================================================================================================================				
DISPLAY_ROTOR_RPM:
				CALL 	LCD_CUR_1ST_ROW
				INCF	TMR1L,1
				MOVF	TMR1L,W ; Convert spin counter HEX value to DECIMAL readable value
				MOVWF   INPUT_HEX_NUM
				CALL 	CONVERT_HEX_TO_DEC
			
			    CALL 	PRINT_ACT
				;CALL	PRINT_SPACE
                ;CALL    PRINT_RPM
				CALL	PRINT_KOLON
			    CALL	PRINT_DEC_VAL
			
				CALL 	PRINT_00
				
				CALL	PRINT_SPACE
				CALL	CALC_EPSILON
				CALL	CALC_STEER_SIGNAL
				MOVF	STEER_SIGNAL,W
				MOVWF	INPUT_HEX_NUM
				MOVWF	CCPR1L
				RLF		CCPR1L,1
				CALL	PRINT_SIGNED_VAL
				
				CLRF	TMR1L
				CLRF	TMR1H
				CLRF	TIMER2_TICKS
				
				RETLW 0
				

			
;=======================================================================================================================
;
; Convert HEX to DEC-routine
;
; 	Input: 		INPUT_HEX_NUM (Number in HEX to be converted)
;
; 	Output : 		HUN_NUM (Hundred decimal)	
;       	      			DEC_NUM (Tenth decimal)
;              			ONE_NUM (Single decimal)	
;=======================================================================================================================	
CONVERT_HEX_TO_DEC:
				CLRF	HUN_NUM
				CLRF	DEC_NUM
				CLRF	ONE_NUM
				MOVF	INPUT_HEX_NUM,W
				MOVWF	TEMP_HEX_VALUE
		CNT_HUN:
				MOVLW	D'100'
				MOVWF	W
				SUBWF	TEMP_HEX_VALUE,1
				BTFSC	STATUS,C
				GOTO	ADD_HUN
				MOVLW	D'100'
				MOVWF	W
				ADDWF	TEMP_HEX_VALUE,1
				GOTO	CNT_DEC
		ADD_HUN:
				MOVLW	D'1'
				MOVWF	W
				ADDWF	HUN_NUM,1
				GOTO	CNT_HUN
		CNT_DEC:
				MOVLW	D'10'
				MOVWF	W
				SUBWF	TEMP_HEX_VALUE,1
				BTFSC	STATUS,C
				GOTO	ADD_DEC
				MOVLW	D'10'
				MOVWF	W
				ADDWF	TEMP_HEX_VALUE,1
				GOTO	CNT_ONE
		ADD_DEC:
				MOVLW	D'1'
				MOVWF	W
				ADDWF	DEC_NUM,1
				GOTO	CNT_DEC

		CNT_ONE:
				MOVLW	D'1'
				MOVWF	W
				SUBWF	TEMP_HEX_VALUE,1
				BTFSC	STATUS,C
				GOTO	ADD_ONE
				MOVLW	D'1'
				MOVWF	W
				ADDWF	TEMP_HEX_VALUE,1
				GOTO	FINISH_HEX_DEC
		ADD_ONE:
				MOVLW	D'1'
				MOVWF	W
				ADDWF	ONE_NUM,1
				GOTO	CNT_ONE
		FINISH_HEX_DEC:
				RETLW	0
				
				
PRINT_DEC_VAL:
				MOVF	HUN_NUM,W
				MOVWF	NUM_TO_ASCII
				CALL	NUM_RESULT_TO_ASCII ; Display hundred count as ASCII-value
				
				MOVF	DEC_NUM,W
				MOVWF	NUM_TO_ASCII
				CALL	NUM_RESULT_TO_ASCII ; Display decimal count as ASCII-value
				
				MOVF	ONE_NUM,W
				MOVWF	NUM_TO_ASCII
			    CALL	NUM_RESULT_TO_ASCII ; Display input key as ASCII-value
				RETLW	0
				
				
PRINT_SIGNED_VAL:
                CALL	PRINT_BYTE_SIGN
				BTFSS	INPUT_HEX_NUM,7
				GOTO	SIGN_VAL_END
				COMF	INPUT_HEX_NUM,1
				MOVLW	D'1'
				MOVWF	W
				ADDWF	INPUT_HEX_NUM
	SIGN_VAL_END:
				CALL	CONVERT_HEX_TO_DEC
				CALL	PRINT_DEC_VAL
				RETLW	0
				
	

PRINT_BYTE_SIGN:
				BTFSS	INPUT_HEX_NUM,7
				GOTO	PLUS_SIGN
				MOVLW	A'-'	; Write the letter '-'
				MOVWF	PORTA_WORD_TO_SERIAL 
				BCF		PORTA,3 ; Set address bit shift register one or two
				
				CALL 	PAR_TO_SERIAL_PORTA
				CALL	LCD_CHR
				GOTO	SIGN_END
	PLUS_SIGN:	
				MOVLW	A'+'	; Write the letter '-'
				MOVWF	PORTA_WORD_TO_SERIAL 
				BCF		PORTA,3 ; Set address bit shift register one or two
				CALL 	PAR_TO_SERIAL_PORTA
				CALL	LCD_CHR
	SIGN_END:
				RETURN
				

;==================================================================================================================
; Multiply numbers
; 
;==================================================================================================================				
MULTIPLY:
				CLRF	MUL_BIT_NUM
				CLRF	MUL_PROD
				MOVLW	D'07' ;Load number of bits to shift
				MOVWF	MUL_BIT_COUNT
		MUL_NEXT_BIT:
				BTFSS 	MUL_OP_1,0
				GOTO    ROTATE_MUL
				GOTO	ADD_TO_PROD
		ROTATE_MUL:				
				MOVLW	D'1'
				MOVWF	W
				ADDWF	MUL_BIT_NUM,1
				RRF		MUL_OP_1,1	
				DECFSZ	MUL_BIT_COUNT,F
				GOTO	MUL_NEXT_BIT
				RETLW	0

		ADD_TO_PROD:
				MOVF	MUL_OP_2,W
				MOVWF	MUL_TEMP_P
				MOVF	MUL_BIT_NUM,W
				MOVWF	TP_COUNT
		ROTATE_T_P:
				SUBLW	D'00'
				BTFSS	STATUS, C
				RLF		MUL_TEMP_P,1
				DECFSZ	TP_COUNT,F
				GOTO	ROTATE_T_P
				MOVF	MUL_TEMP_P,W
				ADDWF	MUL_PROD,1
				GOTO	ROTATE_MUL				
		
		
;==================================================================================================================
; Process key input, store entered digit in variable depending on enter digit order
;
;==================================================================================================================		
PROCESS_DIGIT:
				MOVLW	D'01'
				MOVWF	W
				ADDWF	DIGIT_COUNT,1
				
				MOVF	DIGIT_COUNT,W
				SUBLW	D'01'
				BTFSC	STATUS,Z
				GOTO	SET_FIRST_DIGIT
				
				MOVF	DIGIT_COUNT,W
				SUBLW	D'02'
				BTFSC	STATUS,Z
				GOTO	SET_SECOND_DIGIT
				
				MOVF	DIGIT_COUNT,W
				SUBLW	D'03'
				BTFSC	STATUS,Z
				GOTO	SET_THIRD_DIGIT
				
				GOTO	FINISH_DIGIT
	SET_FIRST_DIGIT:
				MOVF	KEY_INPUT,W
				MOVWF	FIRST_DIGIT
				GOTO 	FINISH_DIGIT
	SET_SECOND_DIGIT:
				MOVF	KEY_INPUT,W
				MOVWF	SECOND_DIGIT
				GOTO 	FINISH_DIGIT
	SET_THIRD_DIGIT:
				MOVF	KEY_INPUT,W
				MOVWF	THIRD_DIGIT
				GOTO 	FINISH_DIGIT
	FINISH_DIGIT:
				RETLW	0

				
				
				
				
;===========================================================================================================
; Get the Key input
;
;===========================================================================================================				
				
GET_KEY_INPUT:	
				MOVF	INPUT_REGISTER,W
				ANDLW	B'00001111'
				SUBLW	D'3'
				BTFSS	STATUS,C
				GOTO	CHK_SECOND_ROW
				GOTO	PROCESS_1ST_ROW
				
	CHK_SECOND_ROW:			
				MOVF	INPUT_REGISTER,W
				ANDLW	B'00001111'
				SUBLW	D'7'
				BTFSS	STATUS,C
				GOTO	CHK_THIRD_ROW
				GOTO	PROCESS_2ND_ROW
				GOTO	FINISH_COMP
	CHK_THIRD_ROW:				
				MOVF	INPUT_REGISTER,W
				ANDLW	B'00001111'
				SUBLW	D'10'
				BTFSS	STATUS,C
				GOTO	CHK_FOURTH_ROW
				GOTO	PROCESS_3RD_ROW
				GOTO	FINISH_COMP

	CHK_FOURTH_ROW:
				MOVF	INPUT_REGISTER,W
				ANDLW	B'00001111'
				SUBLW	D'13'
				BTFSS	STATUS,Z
				GOTO	PROCESS_NAN
				GOTO	PROCESS_ZERO
				GOTO	FINISH_COMP
	PROCESS_1ST_ROW:
				MOVF	INPUT_REGISTER,W
				ANDLW	B'00001111'
				ADDLW	D'01'
				MOVWF	KEY_INPUT
				GOTO	FINISH_COMP

	PROCESS_2ND_ROW:
				MOVF	INPUT_REGISTER,W
				ANDLW	B'00001111'
				MOVWF	KEY_INPUT
				GOTO	FINISH_COMP		

	PROCESS_3RD_ROW:
				MOVF	INPUT_REGISTER,W
				ANDLW	B'00001111'
				MOVWF	KEY_INPUT
				MOVLW	D'01'
				MOVWF	W
				SUBWF	KEY_INPUT,1
				GOTO	FINISH_COMP			
	PROCESS_ZERO:
				CLRF	KEY_INPUT
				GOTO	FINISH_COMP
	PROCESS_NAN:
				MOVF	INPUT_REGISTER,W
				ANDLW	B'00001111'
				MOVWF	KEY_INPUT
				GOTO	FINISH_COMP		
				
	FINISH_COMP:
				RETLW	0


				
				
				
;=====================================================================================================================
; Present a single hexnumber as a ASCII-decimal number
;
; Input :	NUM_TO_ASCII (hex number to be presented as decimal) 
; Output: 	None
;
;=====================================================================================================================
NUM_RESULT_TO_ASCII:
				MOVF	NUM_TO_ASCII,W
				ADDLW	B'00110000'
				MOVWF	PORTA_WORD_TO_SERIAL
				BCF		PORTA,3
				CALL 	PAR_TO_SERIAL_PORTA
				CALL	LCD_CHR
				RETLW	0
				
				
;======================================================================================================================
; Print routines for static text strings
;======================================================================================================================

; Print space
PRINT_SPACE:
				MOVLW	' '
				MOVWF	PORTA_WORD_TO_SERIAL ;Testa kall till seriell dataöverföring
				BCF		PORTA,3 ; Set address bit shift register one or two
				CALL 	PAR_TO_SERIAL_PORTA
				CALL	LCD_CHR
				RETLW	0

; Print colon				
PRINT_KOLON:
				MOVLW	A':'	; Write the letter ':'
				MOVWF	PORTA_WORD_TO_SERIAL ;Testa kall till seriell dataöverföring
				BCF		PORTA,3 ; Set address bit shift register one or two
				CALL 	PAR_TO_SERIAL_PORTA
				CALL	LCD_CHR
				RETLW	0
				
;Print 00
PRINT_00:
				MOVLW	A'0'	; Write the letter '0'
				MOVWF	PORTA_WORD_TO_SERIAL ;Testa kall till seriell dataöverföring
				BCF		PORTA,3 ; Set address bit shift register one or two
				CALL 	PAR_TO_SERIAL_PORTA
				CALL	LCD_CHR
				MOVLW	A'0'	; Write the letter '0'
				MOVWF	PORTA_WORD_TO_SERIAL ;Testa kall till seriell dataöverföring
				BCF		PORTA,3 ; Set address bit shift register one or two
				CALL 	PAR_TO_SERIAL_PORTA
				CALL	LCD_CHR
				RETLW	0
; Print RPM
PRINT_RPM:
				MOVLW	A'R'	; Write the letter 'R'
				MOVWF	PORTA_WORD_TO_SERIAL 
				BCF		PORTA,3 ; Set address bit shift register one or two
				CALL 	PAR_TO_SERIAL_PORTA
				CALL	LCD_CHR
				MOVLW	A'P'	; Write the letter 'P'
				MOVWF	PORTA_WORD_TO_SERIAL 
				BCF		PORTA,3 ; Set address bit shift register one or two
				CALL 	PAR_TO_SERIAL_PORTA
				CALL	LCD_CHR
				MOVLW	A'M'	; Write the letter 'M'
				MOVWF	PORTA_WORD_TO_SERIAL 
				BCF		PORTA,3 ; Set address bit shift register one or two
				CALL 	PAR_TO_SERIAL_PORTA
				CALL	LCD_CHR
				RETLW	0
				
PRINT_SET:
				MOVLW	A'S'	; Write the letter 'S'
				MOVWF	PORTA_WORD_TO_SERIAL 
				BCF		PORTA,3 ; Set address bit shift register one or two
				CALL 	PAR_TO_SERIAL_PORTA
				CALL	LCD_CHR
				MOVLW	A'E'	; Write the letter 'E'
				MOVWF	PORTA_WORD_TO_SERIAL 
				BCF		PORTA,3 ; Set address bit shift register one or two
				CALL 	PAR_TO_SERIAL_PORTA
				CALL	LCD_CHR
				MOVLW	A'T'	; Write the letter 'T'
				MOVWF	PORTA_WORD_TO_SERIAL 
				BCF		PORTA,3 ; Set address bit shift register one or two
				CALL 	PAR_TO_SERIAL_PORTA
				CALL	LCD_CHR
				RETLW	0
PRINT_ACT:
				MOVLW	A'A'	; Write the letter 'A'
				MOVWF	PORTA_WORD_TO_SERIAL 
				BCF		PORTA,3 ; Set address bit shift register one or two
				CALL 	PAR_TO_SERIAL_PORTA
				CALL	LCD_CHR
				MOVLW	A'C'	; Write the letter 'C'
				MOVWF	PORTA_WORD_TO_SERIAL 
				BCF		PORTA,3 ; Set address bit shift register one or two
				CALL 	PAR_TO_SERIAL_PORTA
				CALL	LCD_CHR
				MOVLW	A'T'	; Write the letter 'T'
				MOVWF	PORTA_WORD_TO_SERIAL 
				BCF		PORTA,3 ; Set address bit shift register one or two
				CALL 	PAR_TO_SERIAL_PORTA
				CALL	LCD_CHR
				RETLW	0
				
PRINT_EPS:
				MOVLW	A'E'	; Write the letter 'E'
				MOVWF	PORTA_WORD_TO_SERIAL 
				BCF		PORTA,3 ; Set address bit shift register one or two
				CALL 	PAR_TO_SERIAL_PORTA
				CALL	LCD_CHR
				MOVLW	A'P'	; Write the letter 'P'
				MOVWF	PORTA_WORD_TO_SERIAL 
				BCF		PORTA,3 ; Set address bit shift register one or two
				CALL 	PAR_TO_SERIAL_PORTA
				CALL	LCD_CHR
				MOVLW	A'S'	; Write the letter 'S'
				MOVWF	PORTA_WORD_TO_SERIAL 
				BCF		PORTA,3 ; Set address bit shift register one or two
				CALL 	PAR_TO_SERIAL_PORTA
				CALL	LCD_CHR
				RETLW	0
				
				
				
;======================================================================================================================
; DELAY - Delay routine for long delays, delay by encapsuled iterations
;======================================================================================================================
DELAY_ROUTINE:
				MOVLW   D'255'         ;54 Generate approx 80 ms delay at 4Mhz CLK
                MOVWF   TIMER2
DEL_LOOP1_D     MOVLW   D'255'	       ;60	
                MOVWF   TIMER1
DEL_LOOP2_D     DECFSZ  TIMER1,F
                GOTO    DEL_LOOP2_D
                DECFSZ  TIMER2,F
                GOTO    DEL_LOOP1_D
				RETLW   0
				
;======================================================================================================================
; MEDIUM DELAY - Delay routine for medium delays, delay by encapsuled iterations
;======================================================================================================================		
MEDIUM_DELAY:
				MOVLW   D'100'         ;Generate approx 30 ms delay at 4Mhz CLK
                MOVWF   TIMER2
DEL_LOOP1_M     MOVLW   D'255'	       
                MOVWF   TIMER1
DEL_LOOP2_M     DECFSZ  TIMER1,F
                GOTO    DEL_LOOP2_M
                DECFSZ  TIMER2,F
                GOTO    DEL_LOOP1_M
				RETLW   0

;======================================================================================================================
; MEDIUM DELAY - Delay routine for medium delays, delay by encapsuled iterations
;======================================================================================================================		
MOTOR_DELAY:
LOOP1_MTR     	MOVLW   D'80'	       
                MOVWF   TIMER1
LOOP2_MTR     	DECFSZ  TIMER1,F
                GOTO    LOOP2_MTR
                DECFSZ  TIMER2,F
                GOTO    LOOP1_MTR
				RETLW   0
				
		
;=========================================================================================================================
; Convert an eight bit word to serial representation, 
; clock each bit during conversion
;=========================================================================================================================
PAR_TO_SERIAL_PORTA
				MOVLW	D'08' ;Load number of bits to shift
				MOVWF	PORTA_BITCOUNT
				BCF		PORTA,2 ; Clear shift register
				NOP
				NOP
				NOP
				NOP
				BSF		PORTA,2
BIT_SHIFT	    
				BTFSS 	PORTA_WORD_TO_SERIAL,0
				GOTO	CLR_BIT0_PORTA
				BSF		PORTA,1 ; Set serial bit
				GOTO	BYPASS_CLR_PORTA
CLR_BIT0_PORTA
				BCF		PORTA,1 ; Clear serial bit 
BYPASS_CLR_PORTA
				BSF		PORTA,0 ; Send clock pulse
				BCF		PORTA,0 
				RRF		PORTA_WORD_TO_SERIAL,1
				DECFSZ	PORTA_BITCOUNT,F
				GOTO	BIT_SHIFT
				RETLW	0

				
				
;==============================================================================================================================
; Perform a simple letter test, writing HI to the display
;==============================================================================================================================				
LETTER_TEST:
				MOVLW	B'01001000'	; Write the letter 'H'
				MOVWF	PORTA_WORD_TO_SERIAL ;Testa kall till seriell dataöverföring
				BCF		PORTA,3 ; Set address bit shift register one or two
				CALL 	PAR_TO_SERIAL_PORTA
				CALL	LCD_CHR
				MOVLW	B'01001001'	; Write the letter 'I'
				MOVWF	PORTA_WORD_TO_SERIAL ;Testa kall till seriell dataöverföring
				BCF		PORTA,3 ; Set address bit shift register one or two
				CALL 	PAR_TO_SERIAL_PORTA
				CALL	LCD_CHR
				RETURN
		
; **********************************
; **  RESET :  main boot routine  **
; **********************************		
RESET:	
				MOVLW	B'00000111'	;Disable Comparator module's
				MOVWF	CMCON
				
				MOVLW	D'1'
				MOVWF	SET_MOTOR_RPM
				
				MOVLW	D'2'
				MOVWF	P_VALUE
				
				MOVLW	D'3'
				MOVWF	PROD_VALUE
				
				CLRF	I_RES_VALUE   ; Reset  Integrator values
				CLRF	I_TICK_100_MS
				
				BSF		STATUS,RP0	;Switch to register bank 1
					;Disable pull-ups
					;INT on rising edge
					
				MOVLW	B'11010111'	;Set PIC options (See datasheet).
				MOVWF	OPTION_REG	;Write the OPTION register.
				
				MOVLW	B'00000110'
				MOVWF	T1CON ; Write to T1CON
				
				MOVLW	B'11010000' ; Enable external interrupt, RB0
				MOVWF	INTCON
				
				MOVLW	B'11000001'
				MOVWF	TRISB		;all RB ports are outputs except RB7, RB6, RB0
					
				MOVLW	B'11000000'	;all RA ports are outputs except RA7, RA6
				MOVWF	TRISA
				
				BCF		STATUS,RP0	;Switch Back to reg. Bank 0
				RETURN

;====================================================================================================
; Initialize LCD-module
;====================================================================================================			
LCD_RESET:	
				BCF		PORTB,1
				BSF		PORTB,2
				BSF		PORTB,4
				BSF     PORTA,2
				
				MOVLW	B'00111000'	; Use 8-bit operation, two rows and 5x8 character font
				MOVWF	PORTA_WORD_TO_SERIAL 
				BCF		PORTA,3 ; Set address bit shift register one or two
				CALL 	PAR_TO_SERIAL_PORTA
				
				CALL	LCD_FUNC
				
				MOVLW	B'00001110'	; Turn on display, use cursor
				MOVWF	PORTA_WORD_TO_SERIAL 
				BCF		PORTA,3 ; Set address bit shift register one or two
				CALL 	PAR_TO_SERIAL_PORTA
				
				CALL 	LCD_FUNC
				
				MOVLW	B'00000110' ; Entry mode
				MOVWF	PORTA_WORD_TO_SERIAL 
				BCF		PORTA,3 ; Set address bit shift register one or two
				CALL 	PAR_TO_SERIAL_PORTA
				
				CALL 	LCD_FUNC
				RETURN
				

;=================================================================================================
; Initial configuration Timer 1
;=================================================================================================
				
TMR1_RESET:
				CLRF	TMR1L
				CLRF	TMR1H
				BCF		T1CON,3
				BSF		T1CON,2
				BSF		T1CON,1
				BSF		T1CON,0
				RETURN
;=================================================================================================				
; Initial configuration Timer 2				
;=================================================================================================
TMR2_RESET:
				CLRF	TMR2
				CLRF	T2CON
				BSF		STATUS,RP0	;Switch to register bank 1
				CLRF	TIMER2_TICKS ; Reset ticks
				CLRF	PIE1
				BSF		PIE1,1 ; Enable peripheral interrupts
				BCF		STATUS,RP0	;Switch Back to reg. Bank 0
				BCF     PIR1,1 ; Reset match bit for T2 PR2 Match
				BSF		T2CON,6
				BCF		T2CON,5
				BCF		T2CON,4
				BSF		T2CON,3
				BCF		T2CON,1
				BSF		T2CON,0
				BSF		STATUS,RP0	;Switch Back to reg. Bank 0
				MOVLW   B'11111010' ; Set counter value to PR2
				MOVWF	PR2
				BCF		STATUS,RP0
				BSF		T2CON,2 ; Turn on timer 2
				RETURN
				

PWM_RESET:
				BSF CCP1CON,3
				BSF CCP1CON,2
				RETURN
				

;**********************
; LCD Functional mode
;*********************
LCD_FUNC:	
				BCF		PORTB,4
				CALL 	LCD_CHR
				MOVLW   D'25'         ;54 Generate approx 20mS delay at 4Mhz CLK
				MOVWF   TIMER2
DEL_LOOP1:   
				MOVLW   D'25'	       	
				MOVWF   TIMER1
DEL_LOOP2:  
				DECFSZ  TIMER1,F
				GOTO    DEL_LOOP2
				DECFSZ  TIMER2,F
				GOTO    DEL_LOOP1
				BSF		PORTB,4
				RETURN
		
LCD_CHR:		
				BCF		PORTB,2
				NOP
				BSF		PORTB,1
				NOP
				NOP
				NOP
				BCF		PORTB,1
				NOP
				BSF		PORTB,2
				MOVLW   D'25'         ;3 Generate approx 96 micro seconds delay at 4Mhz CLK
				MOVWF   TIMER2
LOOP_96:
				DECFSZ  TIMER2,F
				GOTO    LOOP_96
				RETURN

LCD_CLEAR:
				MOVLW	B'00000001'	; Clear
				MOVWF	PORTA_WORD_TO_SERIAL ;Testa kall till seriell dataöverföring
				BCF		PORTA,3 ; Set address bit shift register one or two
				CALL 	PAR_TO_SERIAL_PORTA
				CALL 	LCD_FUNC
				RETURN
LCD_CUR_SEC_ROW:
				MOVLW	B'11000000' ; Position cursor at second row
				MOVWF	PORTA_WORD_TO_SERIAL ;Testa kall till seriell dataöverföring
				BCF		PORTA,3 ; Set address bit shift register one or two
				CALL 	PAR_TO_SERIAL_PORTA
				CALL 	LCD_FUNC
				RETURN
LCD_CUR_1ST_ROW:
				MOVLW	B'00000010' ; Position cursor at first row
				MOVWF	PORTA_WORD_TO_SERIAL ;Testa kall till seriell dataöverföring
				BCF		PORTA,3 ; Set address bit shift register one or two
				CALL 	PAR_TO_SERIAL_PORTA
				CALL 	LCD_FUNC
				RETURN
				

MAIN
		CALL	RESET ; Initialize configurations
		CALL	LCD_RESET
		CALL	LCD_CLEAR
		CALL	TMR1_RESET
		CALL 	PWM_RESET
		CALL	TMR2_RESET
WAIT_INT:
		CALL	DELAY_ROUTINE
		GOTO	WAIT_INT
		END
		