#line 1 "C:/Users/khale/Desktop/Embedded/Code_embedded.c"
unsigned char flameDetected;


unsigned int TH_15CM = 0b11011011010;
unsigned int TH_10CM = 0b10010001111;
unsigned int TH_5CM = 0b1001000111;


unsigned char IR_LED1_MASK = 0b00000010;
unsigned char IR_LED2_MASK = 0b00000100;
unsigned char TRIG_MASK = 0b00000010;
unsigned char ECHO_MASK = 0b00000100;
unsigned char LED1_MASK = 0b00010000;
unsigned char LED2_MASK = 0b00100000;
unsigned char LED3_MASK = 0b01000000;
unsigned char FLAME_SENSOR_MASK = 0b00001000;
unsigned char BUZZER_MASK = 0b00000001;
unsigned char FLAME_LED_MASK = 0b00000001;
unsigned char IN1_MASK = 0b00000010;
unsigned char IN2_MASK = 0b00000100;
unsigned char IR_SENSOR_MASK = 0b00000001;
unsigned char SERVO_MASK = 0b00000100;
unsigned char MOTOR_ENABLE_MASK = 0b00000001;
unsigned char GO_DONE_MASK = 0b00000010;
unsigned int Read_ADC(unsigned char channel) {
 ADCON0 = 0b01000001 | (channel << 3);
 Delay_ms(2);
 ADCON0 = ADCON0 | 0x02;
 while (ADCON0 & GO_DONE_MASK);
 return ((ADRESH << 8) + ADRESL);
}


void delay_us_variable(unsigned int t) {
 while (t--) {
 Delay_us(1);
 }
}


void PWM_Software(unsigned char duty_percent) {
 PORTD = PORTD | MOTOR_ENABLE_MASK;
 delay_us_variable(duty_percent * 10);
 PORTD = PORTD & ~MOTOR_ENABLE_MASK;
 delay_us_variable((100 - duty_percent) * 10);
}


void CCPPWM_init(void) {
 T2CON = 0b00000111;
 CCP1CON = 0b00001100;
 PR2 = 250;
 TRISC = TRISC & ~SERVO_MASK;
 CCPR1L = 62;
}


void servo_control(unsigned char position) {
 CCPR1L = position;
}




void interrupt() {
 if (INTCON.T0IF) {
 INTCON.T0IF = 0;
 TMR0 = 6;


 if (PORTD & FLAME_SENSOR_MASK) {
 flameDetected = 1;
 }
 }
}




void main() {
 unsigned char duty = 70;
 unsigned char i;
 float temp;
 unsigned int adc_val, tcount;
 unsigned char servo_position;


 CMCON = 0b00000111;
 ADCON1 = 0b10000000;


 TRISA = TRISA | 0b00000001;
 TRISD = TRISD & 0b11111000;
 TRISD = (TRISD | FLAME_SENSOR_MASK) & ~(LED1_MASK|LED2_MASK|LED3_MASK);
 TRISB = (TRISB & ~TRIG_MASK) | ECHO_MASK;
 TRISE = TRISE & ~BUZZER_MASK;
 TRISC = TRISC & ~FLAME_LED_MASK;
 TRISB = TRISB | IR_SENSOR_MASK;
 TRISC = TRISC & ~SERVO_MASK;
 TRISE = TRISE & ~(IR_LED1_MASK | IR_LED2_MASK);



 PORTD = (PORTD | IN1_MASK) & ~IN2_MASK;
 PORTB = PORTB & ~TRIG_MASK;
 PORTE = PORTE & ~BUZZER_MASK;
 PORTC = PORTC & ~FLAME_LED_MASK;
 PORTE = PORTE & ~(IR_LED1_MASK | IR_LED2_MASK);



 Delay_ms(50);
 CCPPWM_init();


 flameDetected = 0;
 OPTION_REG = 0b00000111;
 TMR0 = 6;
 INTCON.T0IE = 1;
 INTCON.GIE = 1;


 while (1) {

 if (flameDetected) {
 PORTE = PORTE | BUZZER_MASK;
 PORTC = PORTC | FLAME_LED_MASK;

 while (PORTD & FLAME_SENSOR_MASK);

 PORTE = PORTE & ~BUZZER_MASK;
 PORTC = PORTC & ~FLAME_LED_MASK;
 flameDetected = 0;
 continue;
 }


 PORTB = PORTB & ~TRIG_MASK; Delay_us(2);
 PORTB = PORTB | TRIG_MASK; Delay_us(10);
 PORTB = PORTB & ~TRIG_MASK;

 while (!(PORTB & ECHO_MASK));
 TMR1H = 0; TMR1L = 0;
 T1CON = 0b00000001;
 while (PORTB & ECHO_MASK);
 T1CON.F0 = 0;
 tcount = (TMR1H << 8) | TMR1L;

 if (tcount < TH_5CM) {
 PORTD = PORTD | (LED1_MASK | LED2_MASK | LED3_MASK);
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


 adc_val = Read_ADC(0);
 temp = adc_val * 0.488;
 if (temp < 40.0) {
 for (i = 0; i < 50; i++) {
 PWM_Software(duty);
 }
 } else {
 PORTD = PORTD & ~MOTOR_ENABLE_MASK;
 }


 if (PORTB & IR_SENSOR_MASK) {

 PORTE = PORTE & ~(IR_LED1_MASK | IR_LED2_MASK);
 servo_position = 62;
 } else {

 PORTE = PORTE | (IR_LED1_MASK | IR_LED2_MASK);
 servo_position = 150;
 }
 servo_control(servo_position);

 Delay_ms(100);
 }
}
