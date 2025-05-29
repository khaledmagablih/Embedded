
_Read_ADC:

;Code_embedded.c,25 :: 		unsigned int Read_ADC(unsigned char channel) {
;Code_embedded.c,26 :: 		ADCON0 = 0b01000001 | (channel << 3);
	MOVF       FARG_Read_ADC_channel+0, 0
	MOVWF      R0+0
	RLF        R0+0, 1
	BCF        R0+0, 0
	RLF        R0+0, 1
	BCF        R0+0, 0
	RLF        R0+0, 1
	BCF        R0+0, 0
	MOVLW      65
	IORWF      R0+0, 0
	MOVWF      ADCON0+0
;Code_embedded.c,27 :: 		Delay_ms(2);
	MOVLW      6
	MOVWF      R12+0
	MOVLW      48
	MOVWF      R13+0
L_Read_ADC0:
	DECFSZ     R13+0, 1
	GOTO       L_Read_ADC0
	DECFSZ     R12+0, 1
	GOTO       L_Read_ADC0
	NOP
;Code_embedded.c,28 :: 		ADCON0 = ADCON0 | 0x02;
	BSF        ADCON0+0, 1
;Code_embedded.c,29 :: 		while (ADCON0 & GO_DONE_MASK);
L_Read_ADC1:
	MOVF       _GO_DONE_MASK+0, 0
	ANDWF      ADCON0+0, 0
	MOVWF      R0+0
	BTFSC      STATUS+0, 2
	GOTO       L_Read_ADC2
	GOTO       L_Read_ADC1
L_Read_ADC2:
;Code_embedded.c,30 :: 		return ((ADRESH << 8) + ADRESL);
	MOVF       ADRESH+0, 0
	MOVWF      R0+1
	CLRF       R0+0
	MOVF       ADRESL+0, 0
	ADDWF      R0+0, 1
	BTFSC      STATUS+0, 0
	INCF       R0+1, 1
;Code_embedded.c,31 :: 		}
L_end_Read_ADC:
	RETURN
; end of _Read_ADC

_Delay_us_New:

;Code_embedded.c,33 :: 		void Delay_us_New(unsigned int us) {
;Code_embedded.c,35 :: 		while(us--) {
L_Delay_us_New3:
	MOVF       FARG_Delay_us_New_us+0, 0
	MOVWF      R0+0
	MOVF       FARG_Delay_us_New_us+1, 0
	MOVWF      R0+1
	MOVLW      1
	SUBWF      FARG_Delay_us_New_us+0, 1
	BTFSS      STATUS+0, 0
	DECF       FARG_Delay_us_New_us+1, 1
	MOVF       R0+0, 0
	IORWF      R0+1, 0
	BTFSC      STATUS+0, 2
	GOTO       L_Delay_us_New4
;Code_embedded.c,37 :: 		for (i = 0; i < 2; i++) {
	CLRF       R2+0
	CLRF       R2+1
L_Delay_us_New5:
	MOVLW      0
	SUBWF      R2+1, 0
	BTFSS      STATUS+0, 2
	GOTO       L__Delay_us_New36
	MOVLW      2
	SUBWF      R2+0, 0
L__Delay_us_New36:
	BTFSC      STATUS+0, 0
	GOTO       L_Delay_us_New6
	INCF       R2+0, 1
	BTFSC      STATUS+0, 2
	INCF       R2+1, 1
;Code_embedded.c,39 :: 		}
	GOTO       L_Delay_us_New5
L_Delay_us_New6:
;Code_embedded.c,40 :: 		}
	GOTO       L_Delay_us_New3
L_Delay_us_New4:
;Code_embedded.c,41 :: 		}
L_end_Delay_us_New:
	RETURN
; end of _Delay_us_New

_Delay_ms_New:

;Code_embedded.c,43 :: 		void Delay_ms_New(unsigned int ms) {
;Code_embedded.c,44 :: 		while (ms--) {
L_Delay_ms_New8:
	MOVF       FARG_Delay_ms_New_ms+0, 0
	MOVWF      R0+0
	MOVF       FARG_Delay_ms_New_ms+1, 0
	MOVWF      R0+1
	MOVLW      1
	SUBWF      FARG_Delay_ms_New_ms+0, 1
	BTFSS      STATUS+0, 0
	DECF       FARG_Delay_ms_New_ms+1, 1
	MOVF       R0+0, 0
	IORWF      R0+1, 0
	BTFSC      STATUS+0, 2
	GOTO       L_Delay_ms_New9
;Code_embedded.c,45 :: 		Delay_us_New(1000);
	MOVLW      232
	MOVWF      FARG_Delay_us_New_us+0
	MOVLW      3
	MOVWF      FARG_Delay_us_New_us+1
	CALL       _Delay_us_New+0
;Code_embedded.c,46 :: 		}
	GOTO       L_Delay_ms_New8
L_Delay_ms_New9:
;Code_embedded.c,47 :: 		}
L_end_Delay_ms_New:
	RETURN
; end of _Delay_ms_New

_PWM_Software:

;Code_embedded.c,50 :: 		void PWM_Software(unsigned char duty_percent) {
;Code_embedded.c,51 :: 		PORTD = PORTD | MOTOR_ENABLE_MASK;                           // set RD0
	MOVF       _MOTOR_ENABLE_MASK+0, 0
	IORWF      PORTD+0, 1
;Code_embedded.c,52 :: 		Delay_us_New(duty_percent * 10);                 // ON time
	MOVF       FARG_PWM_Software_duty_percent+0, 0
	MOVWF      R0+0
	MOVLW      10
	MOVWF      R4+0
	CALL       _Mul_8X8_U+0
	MOVF       R0+0, 0
	MOVWF      FARG_Delay_us_New_us+0
	MOVF       R0+1, 0
	MOVWF      FARG_Delay_us_New_us+1
	CALL       _Delay_us_New+0
;Code_embedded.c,53 :: 		PORTD = PORTD & ~MOTOR_ENABLE_MASK;                          // clear RD0
	COMF       _MOTOR_ENABLE_MASK+0, 0
	MOVWF      R0+0
	MOVF       R0+0, 0
	ANDWF      PORTD+0, 1
;Code_embedded.c,54 :: 		Delay_us_New((100 - duty_percent) * 10);         // OFF time
	MOVF       FARG_PWM_Software_duty_percent+0, 0
	SUBLW      100
	MOVWF      R0+0
	CLRF       R0+1
	BTFSS      STATUS+0, 0
	DECF       R0+1, 1
	MOVLW      10
	MOVWF      R4+0
	MOVLW      0
	MOVWF      R4+1
	CALL       _Mul_16X16_U+0
	MOVF       R0+0, 0
	MOVWF      FARG_Delay_us_New_us+0
	MOVF       R0+1, 0
	MOVWF      FARG_Delay_us_New_us+1
	CALL       _Delay_us_New+0
;Code_embedded.c,55 :: 		}
L_end_PWM_Software:
	RETURN
; end of _PWM_Software

_CCPPWM_init:

;Code_embedded.c,58 :: 		void CCPPWM_init(void) {
;Code_embedded.c,59 :: 		T2CON   = 0b00000111;  // Timer2 on, Fosc/4, 1:16 prescaler
	MOVLW      7
	MOVWF      T2CON+0
;Code_embedded.c,60 :: 		CCP1CON = 0b00001100;  // PWM mode on CCP1 (RC2)
	MOVLW      12
	MOVWF      CCP1CON+0
;Code_embedded.c,61 :: 		PR2     = 250;         // ~20ms period at 8MHz
	MOVLW      250
	MOVWF      PR2+0
;Code_embedded.c,62 :: 		TRISC  = TRISC & ~SERVO_MASK; // RC2 output
	COMF       _SERVO_MASK+0, 0
	MOVWF      R0+0
	MOVF       R0+0, 0
	ANDWF      TRISC+0, 1
;Code_embedded.c,63 :: 		CCPR1L  = 62;          // 1ms pulse width default
	MOVLW      62
	MOVWF      CCPR1L+0
;Code_embedded.c,64 :: 		}
L_end_CCPPWM_init:
	RETURN
; end of _CCPPWM_init

_servo_control:

;Code_embedded.c,67 :: 		void servo_control(unsigned char position) {
;Code_embedded.c,68 :: 		CCPR1L = position;     // 62–125 range
	MOVF       FARG_servo_control_position+0, 0
	MOVWF      CCPR1L+0
;Code_embedded.c,69 :: 		}
L_end_servo_control:
	RETURN
; end of _servo_control

_interrupt:
	MOVWF      R15+0
	SWAPF      STATUS+0, 0
	CLRF       STATUS+0
	MOVWF      ___saveSTATUS+0
	MOVF       PCLATH+0, 0
	MOVWF      ___savePCLATH+0
	CLRF       PCLATH+0

;Code_embedded.c,74 :: 		void interrupt() {
;Code_embedded.c,75 :: 		if (INTCON & 0b00100000) {
	BTFSS      INTCON+0, 5
	GOTO       L_interrupt10
;Code_embedded.c,76 :: 		INTCON = INTCON & 0b11011111;    // clear overflow flag
	MOVLW      223
	ANDWF      INTCON+0, 1
;Code_embedded.c,77 :: 		TMR0        = 6;    // reload for next ~64ms
	MOVLW      6
	MOVWF      TMR0+0
;Code_embedded.c,80 :: 		if (PORTD & FLAME_SENSOR_MASK) {
	MOVF       _FLAME_SENSOR_MASK+0, 0
	ANDWF      PORTD+0, 0
	MOVWF      R0+0
	BTFSC      STATUS+0, 2
	GOTO       L_interrupt11
;Code_embedded.c,81 :: 		flameDetected = 1;
	MOVLW      1
	MOVWF      _flameDetected+0
;Code_embedded.c,82 :: 		}
L_interrupt11:
;Code_embedded.c,83 :: 		}
L_interrupt10:
;Code_embedded.c,84 :: 		}
L_end_interrupt:
L__interrupt42:
	MOVF       ___savePCLATH+0, 0
	MOVWF      PCLATH+0
	SWAPF      ___saveSTATUS+0, 0
	MOVWF      STATUS+0
	SWAPF      R15+0, 1
	SWAPF      R15+0, 0
	RETFIE
; end of _interrupt

_main:

;Code_embedded.c,89 :: 		void main() {
;Code_embedded.c,90 :: 		unsigned char duty = 70;    // PWM duty percent
	MOVLW      70
	MOVWF      main_duty_L0+0
;Code_embedded.c,97 :: 		CMCON  = 0b00000111;
	MOVLW      7
	MOVWF      CMCON+0
;Code_embedded.c,98 :: 		ADCON1 = 0b10000000;
	MOVLW      128
	MOVWF      ADCON1+0
;Code_embedded.c,101 :: 		TRISA = TRISA | 0b00000001;                                   // RA0 input (LM35)
	BSF        TRISA+0, 0
;Code_embedded.c,102 :: 		TRISD = TRISD & 0b11111000;                                  // RD0–RD2 outputs
	MOVLW      248
	ANDWF      TRISD+0, 1
;Code_embedded.c,103 :: 		TRISD = (TRISD | FLAME_SENSOR_MASK) & ~(LED1_MASK|LED2_MASK|LED3_MASK);
	MOVF       _FLAME_SENSOR_MASK+0, 0
	IORWF      TRISD+0, 0
	MOVWF      R1+0
	MOVF       _LED2_MASK+0, 0
	IORWF      _LED1_MASK+0, 0
	MOVWF      R0+0
	MOVF       _LED3_MASK+0, 0
	IORWF      R0+0, 1
	COMF       R0+0, 1
	MOVF       R0+0, 0
	ANDWF      R1+0, 0
	MOVWF      TRISD+0
;Code_embedded.c,104 :: 		TRISB = (TRISB & ~TRIG_MASK) | ECHO_MASK;              // TRIG out, ECHO in
	COMF       _TRIG_MASK+0, 0
	MOVWF      R5+0
	MOVF       R5+0, 0
	ANDWF      TRISB+0, 0
	MOVWF      R0+0
	MOVF       _ECHO_MASK+0, 0
	IORWF      R0+0, 0
	MOVWF      TRISB+0
;Code_embedded.c,105 :: 		TRISE = TRISE & ~BUZZER_MASK;                                 // RE0 output (buzzer)
	COMF       _BUZZER_MASK+0, 0
	MOVWF      R4+0
	MOVF       R4+0, 0
	ANDWF      TRISE+0, 1
;Code_embedded.c,106 :: 		TRISC = TRISC & ~FLAME_LED_MASK;                              // RC0 output (flame LED)
	COMF       _FLAME_LED_MASK+0, 0
	MOVWF      R3+0
	MOVF       R3+0, 0
	ANDWF      TRISC+0, 1
;Code_embedded.c,107 :: 		TRISB = TRISB | IR_SENSOR_MASK;                               // RB0 input (IR)
	MOVF       _IR_SENSOR_MASK+0, 0
	IORWF      TRISB+0, 1
;Code_embedded.c,108 :: 		TRISC = TRISC & ~SERVO_MASK;                                  // RC2 output (servo PWM)
	COMF       _SERVO_MASK+0, 0
	MOVWF      R0+0
	MOVF       R0+0, 0
	ANDWF      TRISC+0, 1
;Code_embedded.c,109 :: 		TRISE = TRISE & ~(IR_LED1_MASK | IR_LED2_MASK);  // RE1, RE2 outputs for IR-indicator LEDs
	MOVF       _IR_LED2_MASK+0, 0
	IORWF      _IR_LED1_MASK+0, 0
	MOVWF      R0+0
	COMF       R0+0, 0
	MOVWF      R2+0
	MOVF       R2+0, 0
	ANDWF      TRISE+0, 1
;Code_embedded.c,113 :: 		PORTD = (PORTD | IN1_MASK) & ~IN2_MASK;                // motor IN1=1, IN2=0
	MOVF       _IN1_MASK+0, 0
	IORWF      PORTD+0, 0
	MOVWF      R1+0
	COMF       _IN2_MASK+0, 0
	MOVWF      R0+0
	MOVF       R0+0, 0
	ANDWF      R1+0, 0
	MOVWF      PORTD+0
;Code_embedded.c,114 :: 		PORTB = PORTB & ~TRIG_MASK;
	MOVF       R5+0, 0
	ANDWF      PORTB+0, 1
;Code_embedded.c,115 :: 		PORTE = PORTE & ~BUZZER_MASK;
	MOVF       R4+0, 0
	ANDWF      PORTE+0, 1
;Code_embedded.c,116 :: 		PORTC = PORTC & ~FLAME_LED_MASK;
	MOVF       R3+0, 0
	ANDWF      PORTC+0, 1
;Code_embedded.c,117 :: 		PORTE = PORTE & ~(IR_LED1_MASK | IR_LED2_MASK);  // make sure IR LEDs start off
	MOVF       R2+0, 0
	ANDWF      PORTE+0, 1
;Code_embedded.c,121 :: 		Delay_ms_New(50);
	MOVLW      50
	MOVWF      FARG_Delay_ms_New_ms+0
	MOVLW      0
	MOVWF      FARG_Delay_ms_New_ms+1
	CALL       _Delay_ms_New+0
;Code_embedded.c,122 :: 		CCPPWM_init();
	CALL       _CCPPWM_init+0
;Code_embedded.c,125 :: 		flameDetected = 0;                                     // clear flag
	CLRF       _flameDetected+0
;Code_embedded.c,126 :: 		OPTION_REG = 0b00000111;  // prescaler 1:256 on TMR0
	MOVLW      7
	MOVWF      OPTION_REG+0
;Code_embedded.c,127 :: 		TMR0       = 6;           // preload
	MOVLW      6
	MOVWF      TMR0+0
;Code_embedded.c,128 :: 		INTCON = INTCON | 0b00010000;          // enable TMR0 interrupt
	BSF        INTCON+0, 4
;Code_embedded.c,129 :: 		INTCON = INTCON | 0b10000000;         // global interrupts
	BSF        INTCON+0, 7
;Code_embedded.c,132 :: 		while (1) {
L_main12:
;Code_embedded.c,134 :: 		if (flameDetected) {
	MOVF       _flameDetected+0, 0
	BTFSC      STATUS+0, 2
	GOTO       L_main14
;Code_embedded.c,135 :: 		PORTE = PORTE | BUZZER_MASK;      // buzzer on
	MOVF       _BUZZER_MASK+0, 0
	IORWF      PORTE+0, 1
;Code_embedded.c,136 :: 		PORTC = PORTC | FLAME_LED_MASK;   // flame LED on
	MOVF       _FLAME_LED_MASK+0, 0
	IORWF      PORTC+0, 1
;Code_embedded.c,138 :: 		while (PORTD & FLAME_SENSOR_MASK);
L_main15:
	MOVF       _FLAME_SENSOR_MASK+0, 0
	ANDWF      PORTD+0, 0
	MOVWF      R0+0
	BTFSC      STATUS+0, 2
	GOTO       L_main16
	GOTO       L_main15
L_main16:
;Code_embedded.c,140 :: 		PORTE = PORTE & ~BUZZER_MASK;
	COMF       _BUZZER_MASK+0, 0
	MOVWF      R0+0
	MOVF       R0+0, 0
	ANDWF      PORTE+0, 1
;Code_embedded.c,141 :: 		PORTC = PORTC & ~FLAME_LED_MASK;
	COMF       _FLAME_LED_MASK+0, 0
	MOVWF      R0+0
	MOVF       R0+0, 0
	ANDWF      PORTC+0, 1
;Code_embedded.c,142 :: 		flameDetected = 0;
	CLRF       _flameDetected+0
;Code_embedded.c,143 :: 		continue;                  // skip normal routine this pass
	GOTO       L_main12
;Code_embedded.c,144 :: 		}
L_main14:
;Code_embedded.c,147 :: 		PORTB = PORTB & ~TRIG_MASK; Delay_us_New(2);
	COMF       _TRIG_MASK+0, 0
	MOVWF      R0+0
	MOVF       R0+0, 0
	ANDWF      PORTB+0, 1
	MOVLW      2
	MOVWF      FARG_Delay_us_New_us+0
	MOVLW      0
	MOVWF      FARG_Delay_us_New_us+1
	CALL       _Delay_us_New+0
;Code_embedded.c,148 :: 		PORTB = PORTB | TRIG_MASK; Delay_us_New(10);
	MOVF       _TRIG_MASK+0, 0
	IORWF      PORTB+0, 1
	MOVLW      10
	MOVWF      FARG_Delay_us_New_us+0
	MOVLW      0
	MOVWF      FARG_Delay_us_New_us+1
	CALL       _Delay_us_New+0
;Code_embedded.c,149 :: 		PORTB = PORTB & ~TRIG_MASK;
	COMF       _TRIG_MASK+0, 0
	MOVWF      R0+0
	MOVF       R0+0, 0
	ANDWF      PORTB+0, 1
;Code_embedded.c,151 :: 		while (!(PORTB & ECHO_MASK));
L_main17:
	MOVF       _ECHO_MASK+0, 0
	ANDWF      PORTB+0, 0
	MOVWF      R0+0
	BTFSS      STATUS+0, 2
	GOTO       L_main18
	GOTO       L_main17
L_main18:
;Code_embedded.c,152 :: 		TMR1H = 0; TMR1L = 0;
	CLRF       TMR1H+0
	CLRF       TMR1L+0
;Code_embedded.c,153 :: 		T1CON = 0b00000001;           // start Timer1
	MOVLW      1
	MOVWF      T1CON+0
;Code_embedded.c,154 :: 		while (PORTB & ECHO_MASK);
L_main19:
	MOVF       _ECHO_MASK+0, 0
	ANDWF      PORTB+0, 0
	MOVWF      R0+0
	BTFSC      STATUS+0, 2
	GOTO       L_main20
	GOTO       L_main19
L_main20:
;Code_embedded.c,155 :: 		T1CON = T1CON & 0b11111110;                 // stop Timer1
	MOVLW      254
	ANDWF      T1CON+0, 1
;Code_embedded.c,156 :: 		tcount = (TMR1H << 8) | TMR1L;
	MOVF       TMR1H+0, 0
	MOVWF      R0+1
	CLRF       R0+0
	MOVF       TMR1L+0, 0
	IORWF      R0+0, 0
	MOVWF      R2+0
	MOVF       R0+1, 0
	MOVWF      R2+1
	MOVLW      0
	IORWF      R2+1, 1
	MOVF       R2+0, 0
	MOVWF      main_tcount_L0+0
	MOVF       R2+1, 0
	MOVWF      main_tcount_L0+1
;Code_embedded.c,158 :: 		if (tcount < TH_5CM) {
	MOVF       _TH_5CM+1, 0
	SUBWF      R2+1, 0
	BTFSS      STATUS+0, 2
	GOTO       L__main44
	MOVF       _TH_5CM+0, 0
	SUBWF      R2+0, 0
L__main44:
	BTFSC      STATUS+0, 0
	GOTO       L_main21
;Code_embedded.c,159 :: 		PORTD = PORTD  | (LED1_MASK | LED2_MASK | LED3_MASK);
	MOVF       _LED2_MASK+0, 0
	IORWF      _LED1_MASK+0, 0
	MOVWF      R0+0
	MOVF       _LED3_MASK+0, 0
	IORWF      R0+0, 1
	MOVF       R0+0, 0
	IORWF      PORTD+0, 1
;Code_embedded.c,160 :: 		}
	GOTO       L_main22
L_main21:
;Code_embedded.c,161 :: 		else if (tcount < TH_10CM) {
	MOVF       _TH_10CM+1, 0
	SUBWF      main_tcount_L0+1, 0
	BTFSS      STATUS+0, 2
	GOTO       L__main45
	MOVF       _TH_10CM+0, 0
	SUBWF      main_tcount_L0+0, 0
L__main45:
	BTFSC      STATUS+0, 0
	GOTO       L_main23
;Code_embedded.c,162 :: 		PORTD = (PORTD & ~LED3_MASK) | (LED1_MASK | LED2_MASK);
	COMF       _LED3_MASK+0, 0
	MOVWF      R0+0
	MOVF       R0+0, 0
	ANDWF      PORTD+0, 0
	MOVWF      R1+0
	MOVF       _LED2_MASK+0, 0
	IORWF      _LED1_MASK+0, 0
	MOVWF      R0+0
	MOVF       R0+0, 0
	IORWF      R1+0, 0
	MOVWF      PORTD+0
;Code_embedded.c,163 :: 		}
	GOTO       L_main24
L_main23:
;Code_embedded.c,164 :: 		else if (tcount < TH_15CM) {
	MOVF       _TH_15CM+1, 0
	SUBWF      main_tcount_L0+1, 0
	BTFSS      STATUS+0, 2
	GOTO       L__main46
	MOVF       _TH_15CM+0, 0
	SUBWF      main_tcount_L0+0, 0
L__main46:
	BTFSC      STATUS+0, 0
	GOTO       L_main25
;Code_embedded.c,165 :: 		PORTD = (PORTD & ~(LED2_MASK|LED3_MASK)) | LED1_MASK;
	MOVF       _LED3_MASK+0, 0
	IORWF      _LED2_MASK+0, 0
	MOVWF      R0+0
	COMF       R0+0, 1
	MOVF       PORTD+0, 0
	ANDWF      R0+0, 1
	MOVF       _LED1_MASK+0, 0
	IORWF      R0+0, 0
	MOVWF      PORTD+0
;Code_embedded.c,166 :: 		}
	GOTO       L_main26
L_main25:
;Code_embedded.c,168 :: 		PORTD &= ~(LED1_MASK|LED2_MASK|LED3_MASK);
	MOVF       _LED2_MASK+0, 0
	IORWF      _LED1_MASK+0, 0
	MOVWF      R0+0
	MOVF       _LED3_MASK+0, 0
	IORWF      R0+0, 1
	COMF       R0+0, 1
	MOVF       R0+0, 0
	ANDWF      PORTD+0, 1
;Code_embedded.c,169 :: 		}
L_main26:
L_main24:
L_main22:
;Code_embedded.c,172 :: 		adc_val = Read_ADC(0);
	CLRF       FARG_Read_ADC_channel+0
	CALL       _Read_ADC+0
;Code_embedded.c,173 :: 		temp    = adc_val * 0.488;
	CALL       _word2double+0
	MOVLW      35
	MOVWF      R4+0
	MOVLW      219
	MOVWF      R4+1
	MOVLW      121
	MOVWF      R4+2
	MOVLW      125
	MOVWF      R4+3
	CALL       _Mul_32x32_FP+0
;Code_embedded.c,174 :: 		if (temp > 30.0) {
	MOVF       R0+0, 0
	MOVWF      R4+0
	MOVF       R0+1, 0
	MOVWF      R4+1
	MOVF       R0+2, 0
	MOVWF      R4+2
	MOVF       R0+3, 0
	MOVWF      R4+3
	MOVLW      0
	MOVWF      R0+0
	MOVLW      0
	MOVWF      R0+1
	MOVLW      112
	MOVWF      R0+2
	MOVLW      131
	MOVWF      R0+3
	CALL       _Compare_Double+0
	MOVLW      1
	BTFSC      STATUS+0, 0
	MOVLW      0
	MOVWF      R0+0
	MOVF       R0+0, 0
	BTFSC      STATUS+0, 2
	GOTO       L_main27
;Code_embedded.c,175 :: 		for (i = 0; i < 50; i++) {
	CLRF       main_i_L0+0
L_main28:
	MOVLW      50
	SUBWF      main_i_L0+0, 0
	BTFSC      STATUS+0, 0
	GOTO       L_main29
;Code_embedded.c,176 :: 		PWM_Software(duty);
	MOVF       main_duty_L0+0, 0
	MOVWF      FARG_PWM_Software_duty_percent+0
	CALL       _PWM_Software+0
;Code_embedded.c,175 :: 		for (i = 0; i < 50; i++) {
	INCF       main_i_L0+0, 1
;Code_embedded.c,177 :: 		}
	GOTO       L_main28
L_main29:
;Code_embedded.c,178 :: 		} else {
	GOTO       L_main31
L_main27:
;Code_embedded.c,179 :: 		PORTD = PORTD & ~MOTOR_ENABLE_MASK;
	COMF       _MOTOR_ENABLE_MASK+0, 0
	MOVWF      R0+0
	MOVF       R0+0, 0
	ANDWF      PORTD+0, 1
;Code_embedded.c,180 :: 		}
L_main31:
;Code_embedded.c,183 :: 		if (PORTB & IR_SENSOR_MASK) {
	MOVF       _IR_SENSOR_MASK+0, 0
	ANDWF      PORTB+0, 0
	MOVWF      R0+0
	BTFSC      STATUS+0, 2
	GOTO       L_main32
;Code_embedded.c,185 :: 		PORTE = PORTE & ~(IR_LED1_MASK | IR_LED2_MASK);
	MOVF       _IR_LED2_MASK+0, 0
	IORWF      _IR_LED1_MASK+0, 0
	MOVWF      R0+0
	COMF       R0+0, 1
	MOVF       R0+0, 0
	ANDWF      PORTE+0, 1
;Code_embedded.c,186 :: 		servo_position = 62;
	MOVLW      62
	MOVWF      main_servo_position_L0+0
;Code_embedded.c,187 :: 		} else {
	GOTO       L_main33
L_main32:
;Code_embedded.c,189 :: 		PORTE = PORTE | (IR_LED1_MASK | IR_LED2_MASK);
	MOVF       _IR_LED2_MASK+0, 0
	IORWF      _IR_LED1_MASK+0, 0
	MOVWF      R0+0
	MOVF       R0+0, 0
	IORWF      PORTE+0, 1
;Code_embedded.c,190 :: 		servo_position = 150;
	MOVLW      150
	MOVWF      main_servo_position_L0+0
;Code_embedded.c,191 :: 		}
L_main33:
;Code_embedded.c,192 :: 		servo_control(servo_position);
	MOVF       main_servo_position_L0+0, 0
	MOVWF      FARG_servo_control_position+0
	CALL       _servo_control+0
;Code_embedded.c,194 :: 		Delay_ms_New(100);
	MOVLW      100
	MOVWF      FARG_Delay_ms_New_ms+0
	MOVLW      0
	MOVWF      FARG_Delay_ms_New_ms+1
	CALL       _Delay_ms_New+0
;Code_embedded.c,195 :: 		}
	GOTO       L_main12
;Code_embedded.c,196 :: 		}
L_end_main:
	GOTO       $+0
; end of _main
