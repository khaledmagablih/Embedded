unsigned char flameDetected;

// Thresholds (UltraSonic)
unsigned int TH_15CM = 0b11011011010;   // 1750
unsigned int TH_10CM = 0b10010001111;   // 1167
unsigned int TH_5CM  = 0b1001000111;    // 583

// Bit-masks
unsigned char IR_LED1_MASK      = 0b00000010; // RE1
unsigned char IR_LED2_MASK      = 0b00000100; // RE2
unsigned char TRIG_MASK         = 0b00000010; // RB1
unsigned char ECHO_MASK         = 0b00000100; // RB2
unsigned char LED1_MASK         = 0b00010000; // RD4
unsigned char LED2_MASK         = 0b00100000; // RD5
unsigned char LED3_MASK         = 0b01000000; // RD6
unsigned char FLAME_SENSOR_MASK = 0b00001000; // RD3
unsigned char BUZZER_MASK       = 0b00000001; // RE0
unsigned char FLAME_LED_MASK    = 0b00000001; // RC0
unsigned char IN1_MASK          = 0b00000010; // RD1
unsigned char IN2_MASK          = 0b00000100; // RD2
unsigned char IR_SENSOR_MASK    = 0b00000001; // RB0
unsigned char SERVO_MASK        = 0b00000100; // RC2
unsigned char MOTOR_ENABLE_MASK = 0b00000001; // RD0
unsigned char GO_DONE_MASK = 0b00000010;
unsigned int Read_ADC(unsigned char channel) {
    ADCON0 = 0b01000001 | (channel << 3);
    Delay_ms(2);
    ADCON0 = ADCON0 | 0x02;
    while (ADCON0 & GO_DONE_MASK);
    return ((ADRESH << 8) + ADRESL);
}

// Variable delay
void delay_us_variable(unsigned int t) {
    while (t--) {
        Delay_us(1);
    }
}

// Software PWM on RD0
void PWM_Software(unsigned char duty_percent) {
    PORTD = PORTD | MOTOR_ENABLE_MASK;                           // set RD0
    delay_us_variable(duty_percent * 10);                 // ON time
    PORTD = PORTD & ~MOTOR_ENABLE_MASK;                          // clear RD0
    delay_us_variable((100 - duty_percent) * 10);         // OFF time
}

// CCP PWM Initialization for RC2 (servo)
void CCPPWM_init(void) {
    T2CON   = 0b00000111;  // Timer2 on, Fosc/4, 1:16 prescaler
    CCP1CON = 0b00001100;  // PWM mode on CCP1 (RC2)
    PR2     = 250;         // ~20ms period at 8MHz
    TRISC  = TRISC & ~SERVO_MASK; // RC2 output
    CCPR1L  = 62;          // 1ms pulse width default
}

// Servo control
void servo_control(unsigned char position) {
    CCPR1L = position;     // 62–125 range
}

// —————————————————————————————————————————————————————————————
// Interrupt Service Routine: Timer0 Overflow
// —————————————————————————————————————————————————————————————
void interrupt() {
    if (INTCON.T0IF) {
        INTCON.T0IF = 0;    // clear overflow flag
        TMR0        = 6;    // reload for next ~64ms

        // latch flame detection
        if (PORTD & FLAME_SENSOR_MASK) {
            flameDetected = 1;
        }
    }
}

// =======================
// Main Routine
// =======================
void main() {
    unsigned char duty = 70;    // PWM duty percent
    unsigned char i;
    float temp;
    unsigned int adc_val, tcount;
    unsigned char servo_position;

    // 1) Disable comparators, set AN0 analog only
    CMCON  = 0b00000111;
    ADCON1 = 0b10000000;

    // 2) Configure I/O directions
    TRISA = TRISA | 0b00000001;                                   // RA0 input (LM35)
    TRISD = TRISD & 0b11111000;                                  // RD0–RD2 outputs
    TRISD = (TRISD | FLAME_SENSOR_MASK) & ~(LED1_MASK|LED2_MASK|LED3_MASK);
    TRISB = (TRISB & ~TRIG_MASK) | ECHO_MASK;              // TRIG out, ECHO in
    TRISE = TRISE & ~BUZZER_MASK;                                 // RE0 output (buzzer)
    TRISC = TRISC & ~FLAME_LED_MASK;                              // RC0 output (flame LED)
    TRISB = TRISB | IR_SENSOR_MASK;                               // RB0 input (IR)
    TRISC = TRISC & ~SERVO_MASK;                                  // RC2 output (servo PWM)
    TRISE = TRISE & ~(IR_LED1_MASK | IR_LED2_MASK);  // RE1, RE2 outputs for IR-indicator LEDs


    // 3) Initial output states
    PORTD = (PORTD | IN1_MASK) & ~IN2_MASK;                // motor IN1=1, IN2=0
    PORTB = PORTB & ~TRIG_MASK;
    PORTE = PORTE & ~BUZZER_MASK;
    PORTC = PORTC & ~FLAME_LED_MASK;
    PORTE = PORTE & ~(IR_LED1_MASK | IR_LED2_MASK);  // make sure IR LEDs start off


    // 4) Initialize servo PWM
    Delay_ms(50);
    CCPPWM_init();

    // 5) Initialize flame flag & Timer0 interrupt
    flameDetected = 0;                                     // clear flag
    OPTION_REG = 0b00000111;  // prescaler 1:256 on TMR0
    TMR0       = 6;           // preload
    INTCON.T0IE = 1;          // enable TMR0 interrupt
    INTCON.GIE  = 1;          // global interrupts

    // 6) Main Loop
    while (1) {
        // — If flame detected, halt other tasks —
        if (flameDetected) {
            PORTE = PORTE | BUZZER_MASK;      // buzzer on
            PORTC = PORTC | FLAME_LED_MASK;   // flame LED on
            // wait until flame clears
            while (PORTD & FLAME_SENSOR_MASK);
            // turn off and reset
            PORTE = PORTE & ~BUZZER_MASK;
            PORTC = PORTC & ~FLAME_LED_MASK;
            flameDetected = 0;
            continue;                  // skip normal routine this pass
        }

        // — Normal routine: Ultrasonic —
        PORTB = PORTB & ~TRIG_MASK; Delay_us(2);
        PORTB = PORTB | TRIG_MASK; Delay_us(10);
        PORTB = PORTB & ~TRIG_MASK;

        while (!(PORTB & ECHO_MASK));
        TMR1H = 0; TMR1L = 0;
        T1CON = 0b00000001;           // start Timer1
        while (PORTB & ECHO_MASK);
        T1CON.F0 = 0;                 // stop Timer1
        tcount = (TMR1H << 8) | TMR1L;

        if (tcount < TH_5CM) {
             PORTD = PORTD  | (LED1_MASK | LED2_MASK | LED3_MASK);
        }
        else if (tcount < TH_10CM) {
            PORTD = (PORTD & ~LED3_MASK) | (LED1_MASK | LED2_MASK);
        }
        else if (tcount < TH_15CM) {
            PORTD = (PORTD & ~(LED2_MASK|LED3_MASK)) | LED1_MASK;
        }
        else {
            PORTD &= ~(LED1_MASK|LED2_MASK|LED3_MASK);
        }

        // — Temperature & Motor PWM —
        adc_val = Read_ADC(0);
        temp    = adc_val * 0.488;
        if (temp < 40.0) {
            for (i = 0; i < 50; i++) {
                PWM_Software(duty);
            }
        } else {
            PORTD = PORTD & ~MOTOR_ENABLE_MASK;
        }

        // — IR sensor, LEDs & Servo —
        if (PORTB & IR_SENSOR_MASK) {
            // motion detected ? turn on both IR-indicator LEDs
            PORTE = PORTE & ~(IR_LED1_MASK | IR_LED2_MASK);
            servo_position = 62;
        } else {
            // no motion ? turn LEDs off
            PORTE = PORTE | (IR_LED1_MASK | IR_LED2_MASK);
            servo_position = 150;
        }
        servo_control(servo_position);

        Delay_ms(100);
    }
}