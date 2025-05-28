
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

_delay_us_variable:

;Code_embedded.c,34 :: 		void delay_us_variable(unsigned int t) {
;Code_embedded.c,35 :: 		while (t--) {
L_delay_us_variable3:
	MOVF       FARG_delay_us_variable_t+0, 0
	MOVWF      R0+0
	MOVF       FARG_delay_us_variable_t+1, 0
	MOVWF      R0+1
	MOVLW      1
	SUBWF      FARG_delay_us_variable_t+0, 1
	BTFSS      STATUS+0, 0
	DECF       FARG_delay_us_variable_t+1, 1
	MOVF       R0+0, 0
	IORWF      R0+1, 0
	BTFSC      STATUS+0, 2
	GOTO       L_delay_us_variable4
;Code_embedded.c,36 :: 		Delay_us(1);
	NOP
	NOP
;Code_embedded.c,37 :: 		}
	GOTO       L_delay_us_variable3
L_delay_us_variable4:
;Code_embedded.c,38 :: 		}
L_end_delay_us_variable:
	RETURN
; end of _delay_us_variable

_PWM_Software:

;Code_embedded.c,41 :: 		void PWM_Software(unsigned char duty_percent) {
;Code_embedded.c,42 :: 		PORTD = PORTD | MOTOR_ENABLE_MASK;                           // set RD0
	MOVF       _MOTOR_ENABLE_MASK+0, 0
	IORWF      PORTD+0, 1
;Code_embedded.c,43 :: 		delay_us_variable(duty_percent * 10);                 // ON time
	MOVF       FARG_PWM_Software_duty_percent+0, 0
	MOVWF      R0+0
	MOVLW      10
	MOVWF      R4+0
	CALL       _Mul_8X8_U+0
	MOVF       R0+0, 0
	MOVWF      FARG_delay_us_variable_t+0
	MOVF       R0+1, 0
	MOVWF      FARG_delay_us_variable_t+1
	CALL       _delay_us_variable+0
;Code_embedded.c,44 :: 		PORTD = PORTD & ~MOTOR_ENABLE_MASK;                          // clear RD0
	COMF       _MOTOR_ENABLE_MASK+0, 0
	MOVWF      R0+0
	MOVF       R0+0, 0
	ANDWF      PORTD+0, 1
;Code_embedded.c,45 :: 		delay_us_variable((100 - duty_percent) * 10);         // OFF time
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
	MOVWF      FARG_delay_us_variable_t+0
	MOVF       R0+1, 0
	MOVWF      FARG_delay_us_variable_t+1
	CALL       _delay_us_variable+0
;Code_embedded.c,46 :: 		}
L_end_PWM_Software:
	RETURN
; end of _PWM_Software

_CCPPWM_init:

;Code_embedded.c,49 :: 		void CCPPWM_init(void) {
;Code_embedded.c,50 :: 		T2CON   = 0b00000111;  // Timer2 on, Fosc/4, 1:16 prescaler
	MOVLW      7
	MOVWF      T2CON+0
;Code_embedded.c,51 :: 		CCP1CON = 0b00001100;  // PWM mode on CCP1 (RC2)
	MOVLW      12
	MOVWF      CCP1CON+0
;Code_embedded.c,52 :: 		PR2     = 250;         // ~20ms period at 8MHz
	MOVLW      250
	MOVWF      PR2+0
;Code_embedded.c,53 :: 		TRISC  = TRISC & ~SERVO_MASK; // RC2 output
	COMF       _SERVO_MASK+0, 0
	MOVWF      R0+0
	MOVF       R0+0, 0
	ANDWF      TRISC+0, 1
;Code_embedded.c,54 :: 		CCPR1L  = 62;          // 1ms pulse width default
	MOVLW      62
	MOVWF      CCPR1L+0
;Code_embedded.c,55 :: 		}
L_end_CCPPWM_init:
	RETURN
; end of _CCPPWM_init

_servo_control:

;Code_embedded.c,58 :: 		void servo_control(unsigned char position) {
;Code_embedded.c,59 :: 		CCPR1L = position;     // 62–125 range
	MOVF       FARG_servo_control_position+0, 0
	MOVWF      CCPR1L+0
;Code_embedded.c,60 :: 		}
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

;Code_embedded.c,65 :: 		void interrupt() {
;Code_embedded.c,66 :: 		if (INTCON.T0IF) {
	BTFSS      INTCON+0, 2
	GOTO       L_interrupt5
;Code_embedded.c,67 :: 		INTCON.T0IF = 0;    // clear overflow flag
	BCF        INTCON+0, 2
;Code_embedded.c,68 :: 		TMR0        = 6;    // reload for next ~64ms
	MOVLW      6
	MOVWF      TMR0+0
;Code_embedded.c,71 :: 		if (PORTD & FLAME_SENSOR_MASK) {
	MOVF       _FLAME_SENSOR_MASK+0, 0
	ANDWF      PORTD+0, 0
	MOVWF      R0+0
	BTFSC      STATUS+0, 2
	GOTO       L_interrupt6
;Code_embedded.c,72 :: 		flameDetected = 1;
	MOVLW      1
	MOVWF      _flameDetected+0
;Code_embedded.c,73 :: 		}
L_interrupt6:
;Code_embedded.c,74 :: 		}
L_interrupt5:
;Code_embedded.c,75 :: 		}
L_end_interrupt:
L__interrupt38:
	MOVF       ___savePCLATH+0, 0
	MOVWF      PCLATH+0
	SWAPF      ___saveSTATUS+0, 0
	MOVWF      STATUS+0
	SWAPF      R15+0, 1
	SWAPF      R15+0, 0
	RETFIE
; end of _interrupt

_main:

;Code_embedded.c,80 :: 		void main() {
;Code_embedded.c,81 :: 		unsigned char duty = 70;    // PWM duty percent
	MOVLW      70
	MOVWF      main_duty_L0+0
;Code_embedded.c,88 :: 		CMCON  = 0b00000111;
	MOVLW      7
	MOVWF      CMCON+0
;Code_embedded.c,89 :: 		ADCON1 = 0b10000000;
	MOVLW      128
	MOVWF      ADCON1+0
;Code_embedded.c,92 :: 		TRISA = TRISA | 0b00000001;                                   // RA0 input (LM35)
	BSF        TRISA+0, 0
;Code_embedded.c,93 :: 		TRISD = TRISD & 0b11111000;                                  // RD0–RD2 outputs
	MOVLW      248
	ANDWF      TRISD+0, 1
;Code_embedded.c,94 :: 		TRISD = (TRISD | FLAME_SENSOR_MASK) & ~(LED1_MASK|LED2_MASK|LED3_MASK);
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
;Code_embedded.c,95 :: 		TRISB = (TRISB & ~TRIG_MASK) | ECHO_MASK;              // TRIG out, ECHO in
	COMF       _TRIG_MASK+0, 0
	MOVWF      R5+0
	MOVF       R5+0, 0
	ANDWF      TRISB+0, 0
	MOVWF      R0+0
	MOVF       _ECHO_MASK+0, 0
	IORWF      R0+0, 0
	MOVWF      TRISB+0
;Code_embedded.c,96 :: 		TRISE = TRISE & ~BUZZER_MASK;                                 // RE0 output (buzzer)
	COMF       _BUZZER_MASK+0, 0
	MOVWF      R4+0
	MOVF       R4+0, 0
	ANDWF      TRISE+0, 1
;Code_embedded.c,97 :: 		TRISC = TRISC & ~FLAME_LED_MASK;                              // RC0 output (flame LED)
	COMF       _FLAME_LED_MASK+0, 0
	MOVWF      R3+0
	MOVF       R3+0, 0
	ANDWF      TRISC+0, 1
;Code_embedded.c,98 :: 		TRISB = TRISB | IR_SENSOR_MASK;                               // RB0 input (IR)
	MOVF       _IR_SENSOR_MASK+0, 0
	IORWF      TRISB+0, 1
;Code_embedded.c,99 :: 		TRISC = TRISC & ~SERVO_MASK;                                  // RC2 output (servo PWM)
	COMF       _SERVO_MASK+0, 0
	MOVWF      R0+0
	MOVF       R0+0, 0
	ANDWF      TRISC+0, 1
;Code_embedded.c,100 :: 		TRISE = TRISE & ~(IR_LED1_MASK | IR_LED2_MASK);  // RE1, RE2 outputs for IR-indicator LEDs
	MOVF       _IR_LED2_MASK+0, 0
	IORWF      _IR_LED1_MASK+0, 0
	MOVWF      R0+0
	COMF       R0+0, 0
	MOVWF      R2+0
	MOVF       R2+0, 0
	ANDWF      TRISE+0, 1
;Code_embedded.c,104 :: 		PORTD = (PORTD | IN1_MASK) & ~IN2_MASK;                // motor IN1=1, IN2=0
	MOVF       _IN1_MASK+0, 0
	IORWF      PORTD+0, 0
	MOVWF      R1+0
	COMF       _IN2_MASK+0, 0
	MOVWF      R0+0
	MOVF       R0+0, 0
	ANDWF      R1+0, 0
	MOVWF      PORTD+0
;Code_embedded.c,105 :: 		PORTB = PORTB & ~TRIG_MASK;
	MOVF       R5+0, 0
	ANDWF      PORTB+0, 1
;Code_embedded.c,106 :: 		PORTE = PORTE & ~BUZZER_MASK;
	MOVF       R4+0, 0
	ANDWF      PORTE+0, 1
;Code_embedded.c,107 :: 		PORTC = PORTC & ~FLAME_LED_MASK;
	MOVF       R3+0, 0
	ANDWF      PORTC+0, 1
;Code_embedded.c,108 :: 		PORTE = PORTE & ~(IR_LED1_MASK | IR_LED2_MASK);  // make sure IR LEDs start off
	MOVF       R2+0, 0
	ANDWF      PORTE+0, 1
;Code_embedded.c,112 :: 		Delay_ms(50);
	MOVLW      130
	MOVWF      R12+0
	MOVLW      221
	MOVWF      R13+0
L_main7:
	DECFSZ     R13+0, 1
	GOTO       L_main7
	DECFSZ     R12+0, 1
	GOTO       L_main7
	NOP
	NOP
;Code_embedded.c,113 :: 		CCPPWM_init();
	CALL       _CCPPWM_init+0
;Code_embedded.c,116 :: 		flameDetected = 0;                                     // clear flag
	CLRF       _flameDetected+0
;Code_embedded.c,117 :: 		OPTION_REG = 0b00000111;  // prescaler 1:256 on TMR0
	MOVLW      7
	MOVWF      OPTION_REG+0
;Code_embedded.c,118 :: 		TMR0       = 6;           // preload
	MOVLW      6
	MOVWF      TMR0+0
;Code_embedded.c,119 :: 		INTCON.T0IE = 1;          // enable TMR0 interrupt
	BSF        INTCON+0, 5
;Code_embedded.c,120 :: 		INTCON.GIE  = 1;          // global interrupts
	BSF        INTCON+0, 7
;Code_embedded.c,123 :: 		while (1) {
L_main8:
;Code_embedded.c,125 :: 		if (flameDetected) {
	MOVF       _flameDetected+0, 0
	BTFSC      STATUS+0, 2
	GOTO       L_main10
;Code_embedded.c,126 :: 		PORTE = PORTE | BUZZER_MASK;      // buzzer on
	MOVF       _BUZZER_MASK+0, 0
	IORWF      PORTE+0, 1
;Code_embedded.c,127 :: 		PORTC = PORTC | FLAME_LED_MASK;   // flame LED on
	MOVF       _FLAME_LED_MASK+0, 0
	IORWF      PORTC+0, 1
;Code_embedded.c,129 :: 		while (PORTD & FLAME_SENSOR_MASK);
L_main11:
	MOVF       _FLAME_SENSOR_MASK+0, 0
	ANDWF      PORTD+0, 0
	MOVWF      R0+0
	BTFSC      STATUS+0, 2
	GOTO       L_main12
	GOTO       L_main11
L_main12:
;Code_embedded.c,131 :: 		PORTE = PORTE & ~BUZZER_MASK;
	COMF       _BUZZER_MASK+0, 0
	MOVWF      R0+0
	MOVF       R0+0, 0
	ANDWF      PORTE+0, 1
;Code_embedded.c,132 :: 		PORTC = PORTC & ~FLAME_LED_MASK;
	COMF       _FLAME_LED_MASK+0, 0
	MOVWF      R0+0
	MOVF       R0+0, 0
	ANDWF      PORTC+0, 1
;Code_embedded.c,133 :: 		flameDetected = 0;
	CLRF       _flameDetected+0
;Code_embedded.c,134 :: 		continue;                  // skip normal routine this pass
	GOTO       L_main8
;Code_embedded.c,135 :: 		}
L_main10:
;Code_embedded.c,138 :: 		PORTB = PORTB & ~TRIG_MASK; Delay_us(2);
	COMF       _TRIG_MASK+0, 0
	MOVWF      R0+0
	MOVF       R0+0, 0
	ANDWF      PORTB+0, 1
	NOP
	NOP
	NOP
	NOP
;Code_embedded.c,139 :: 		PORTB = PORTB | TRIG_MASK; Delay_us(10);
	MOVF       _TRIG_MASK+0, 0
	IORWF      PORTB+0, 1
	MOVLW      6
	MOVWF      R13+0
L_main13:
	DECFSZ     R13+0, 1
	GOTO       L_main13
	NOP
;Code_embedded.c,140 :: 		PORTB = PORTB & ~TRIG_MASK;
	COMF       _TRIG_MASK+0, 0
	MOVWF      R0+0
	MOVF       R0+0, 0
	ANDWF      PORTB+0, 1
;Code_embedded.c,142 :: 		while (!(PORTB & ECHO_MASK));
L_main14:
	MOVF       _ECHO_MASK+0, 0
	ANDWF      PORTB+0, 0
	MOVWF      R0+0
	BTFSS      STATUS+0, 2
	GOTO       L_main15
	GOTO       L_main14
L_main15:
;Code_embedded.c,143 :: 		TMR1H = 0; TMR1L = 0;
	CLRF       TMR1H+0
	CLRF       TMR1L+0
;Code_embedded.c,144 :: 		T1CON = 0b00000001;           // start Timer1
	MOVLW      1
	MOVWF      T1CON+0
;Code_embedded.c,145 :: 		while (PORTB & ECHO_MASK);
L_main16:
	MOVF       _ECHO_MASK+0, 0
	ANDWF      PORTB+0, 0
	MOVWF      R0+0
	BTFSC      STATUS+0, 2
	GOTO       L_main17
	GOTO       L_main16
L_main17:
;Code_embedded.c,146 :: 		T1CON.F0 = 0;                 // stop Timer1
	BCF        T1CON+0, 0
;Code_embedded.c,147 :: 		tcount = (TMR1H << 8) | TMR1L;
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
;Code_embedded.c,149 :: 		if (tcount < TH_5CM) {
	MOVF       _TH_5CM+1, 0
	SUBWF      R2+1, 0
	BTFSS      STATUS+0, 2
	GOTO       L__main40
	MOVF       _TH_5CM+0, 0
	SUBWF      R2+0, 0
L__main40:
	BTFSC      STATUS+0, 0
	GOTO       L_main18
;Code_embedded.c,150 :: 		PORTD = PORTD  | (LED1_MASK | LED2_MASK | LED3_MASK);
	MOVF       _LED2_MASK+0, 0
	IORWF      _LED1_MASK+0, 0
	MOVWF      R0+0
	MOVF       _LED3_MASK+0, 0
	IORWF      R0+0, 1
	MOVF       R0+0, 0
	IORWF      PORTD+0, 1
;Code_embedded.c,151 :: 		}
	GOTO       L_main19
L_main18:
;Code_embedded.c,152 :: 		else if (tcount < TH_10CM) {
	MOVF       _TH_10CM+1, 0
	SUBWF      main_tcount_L0+1, 0
	BTFSS      STATUS+0, 2
	GOTO       L__main41
	MOVF       _TH_10CM+0, 0
	SUBWF      main_tcount_L0+0, 0
L__main41:
	BTFSC      STATUS+0, 0
	GOTO       L_main20
;Code_embedded.c,153 :: 		PORTD = (PORTD & ~LED3_MASK) | (LED1_MASK | LED2_MASK);
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
;Code_embedded.c,154 :: 		}
	GOTO       L_main21
L_main20:
;Code_embedded.c,155 :: 		else if (tcount < TH_15CM) {
	MOVF       _TH_15CM+1, 0
	SUBWF      main_tcount_L0+1, 0
	BTFSS      STATUS+0, 2
	GOTO       L__main42
	MOVF       _TH_15CM+0, 0
	SUBWF      main_tcount_L0+0, 0
L__main42:
	BTFSC      STATUS+0, 0
	GOTO       L_main22
;Code_embedded.c,156 :: 		PORTD = (PORTD & ~(LED2_MASK|LED3_MASK)) | LED1_MASK;
	MOVF       _LED3_MASK+0, 0
	IORWF      _LED2_MASK+0, 0
	MOVWF      R0+0
	COMF       R0+0, 1
	MOVF       PORTD+0, 0
	ANDWF      R0+0, 1
	MOVF       _LED1_MASK+0, 0
	IORWF      R0+0, 0
	MOVWF      PORTD+0
;Code_embedded.c,157 :: 		}
	GOTO       L_main23
L_main22:
;Code_embedded.c,159 :: 		PORTD &= ~(LED1_MASK|LED2_MASK|LED3_MASK);
	MOVF       _LED2_MASK+0, 0
	IORWF      _LED1_MASK+0, 0
	MOVWF      R0+0
	MOVF       _LED3_MASK+0, 0
	IORWF      R0+0, 1
	COMF       R0+0, 1
	MOVF       R0+0, 0
	ANDWF      PORTD+0, 1
;Code_embedded.c,160 :: 		}
L_main23:
L_main21:
L_main19:
;Code_embedded.c,163 :: 		adc_val = Read_ADC(0);
	CLRF       FARG_Read_ADC_channel+0
	CALL       _Read_ADC+0
;Code_embedded.c,164 :: 		temp    = adc_val * 0.488;
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
;Code_embedded.c,165 :: 		if (temp < 40.0) {
	MOVLW      0
	MOVWF      R4+0
	MOVLW      0
	MOVWF      R4+1
	MOVLW      32
	MOVWF      R4+2
	MOVLW      132
	MOVWF      R4+3
	CALL       _Compare_Double+0
	MOVLW      1
	BTFSC      STATUS+0, 0
	MOVLW      0
	MOVWF      R0+0
	MOVF       R0+0, 0
	BTFSC      STATUS+0, 2
	GOTO       L_main24
;Code_embedded.c,166 :: 		for (i = 0; i < 50; i++) {
	CLRF       main_i_L0+0
L_main25:
	MOVLW      50
	SUBWF      main_i_L0+0, 0
	BTFSC      STATUS+0, 0
	GOTO       L_main26
;Code_embedded.c,167 :: 		PWM_Software(duty);
	MOVF       main_duty_L0+0, 0
	MOVWF      FARG_PWM_Software_duty_percent+0
	CALL       _PWM_Software+0
;Code_embedded.c,166 :: 		for (i = 0; i < 50; i++) {
	INCF       main_i_L0+0, 1
;Code_embedded.c,168 :: 		}
	GOTO       L_main25
L_main26:
;Code_embedded.c,169 :: 		} else {
	GOTO       L_main28
L_main24:
;Code_embedded.c,170 :: 		PORTD = PORTD & ~MOTOR_ENABLE_MASK;
	COMF       _MOTOR_ENABLE_MASK+0, 0
	MOVWF      R0+0
	MOVF       R0+0, 0
	ANDWF      PORTD+0, 1
;Code_embedded.c,171 :: 		}
L_main28:
;Code_embedded.c,174 :: 		if (PORTB & IR_SENSOR_MASK) {
	MOVF       _IR_SENSOR_MASK+0, 0
	ANDWF      PORTB+0, 0
	MOVWF      R0+0
	BTFSC      STATUS+0, 2
	GOTO       L_main29
;Code_embedded.c,176 :: 		PORTE = PORTE & ~(IR_LED1_MASK | IR_LED2_MASK);
	MOVF       _IR_LED2_MASK+0, 0
	IORWF      _IR_LED1_MASK+0, 0
	MOVWF      R0+0
	COMF       R0+0, 1
	MOVF       R0+0, 0
	ANDWF      PORTE+0, 1
;Code_embedded.c,177 :: 		servo_position = 62;
	MOVLW      62
	MOVWF      main_servo_position_L0+0
;Code_embedded.c,178 :: 		} else {
	GOTO       L_main30
L_main29:
;Code_embedded.c,180 :: 		PORTE = PORTE | (IR_LED1_MASK | IR_LED2_MASK);
	MOVF       _IR_LED2_MASK+0, 0
	IORWF      _IR_LED1_MASK+0, 0
	MOVWF      R0+0
	MOVF       R0+0, 0
	IORWF      PORTE+0, 1
;Code_embedded.c,181 :: 		servo_position = 150;
	MOVLW      150
	MOVWF      main_servo_position_L0+0
;Code_embedded.c,182 :: 		}
L_main30:
;Code_embedded.c,183 :: 		servo_control(servo_position);
	MOVF       main_servo_position_L0+0, 0
	MOVWF      FARG_servo_control_position+0
	CALL       _servo_control+0
;Code_embedded.c,185 :: 		Delay_ms(100);
	MOVLW      2
	MOVWF      R11+0
	MOVLW      4
	MOVWF      R12+0
	MOVLW      186
	MOVWF      R13+0
L_main31:
	DECFSZ     R13+0, 1
	GOTO       L_main31
	DECFSZ     R12+0, 1
	GOTO       L_main31
	DECFSZ     R11+0, 1
	GOTO       L_main31
	NOP
;Code_embedded.c,186 :: 		}
	GOTO       L_main8
;Code_embedded.c,187 :: 		}
L_end_main:
	GOTO       $+0
; end of _main
