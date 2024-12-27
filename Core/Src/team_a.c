//team_a.c
#include "team_a.h"
#include "main.h"
#include <stdint.h>

char keypad_matrix[4][4] = {
	    {'1', '2', '3', 'A'},
	    {'4', '5', '6', 'B'},
	    {'7', '8', '9', 'C'},
	    {'*', '0', '#', 'D'}
	};

// Initialize keypad
void keypad_init() {
    // Configure GPIO pins for keypad
    // Set up input/output modes, pull-ups, etc.

	RCC->AHBENR |= RCC_AHBENR_GPIOCEN;

	// Row pins are PC0 - PC3
	ROW_PORT->MODER &= ~0x000000FF;
	ROW_PORT->MODER |= 0x00000055;
	ROW_PORT->OTYPER &= ~ROW_PINS;
	ROW_PORT->OSPEEDR &= ~0x000000FF;

	// Column pins are PC4 - PC7
	COL_PORT->MODER &= ~0x0000FF00;
	COL_PORT->PUPDR &= ~0x0000FF00;
	COL_PORT->PUPDR |= 0x0000AA00;

}

// Read input from the keypad
char keypad_get_key(void) {
    // Scan the keypad matrix
    // Return the detected keypress as a character
	for (int row = 0; row < 4; row++) {
	        ROW_PORT->ODR = (1 << row);

	        for (int col = 0; col < 4; col++) {
	            if (COL_PORT->IDR & (1 << (col + 4))) {
	                return keypad_matrix[row][col];
	            }
	        }
	    }
	return 'z';
}

// Millisecond delay using busy-wait
void delay_ms(int ms) {
    for (int i = 0; i < ms * 2000; i++) {
        asm("nop");
    }
}

// Microsecond delay using busy-wait
void delay_us(int us) {
    for (int i = 0; i < us * 2; i++) {
        asm("nop");
    }
}

// LCD initialization function
void lcd_init(void) {
    // Enable clocks for GPIOA and GPIOB
    RCC->AHBENR |= RCC_AHBENR_GPIOAEN | RCC_AHBENR_GPIOBEN;

    // Configure PA0 (RS) and PA1 (E) as output
    GPIOA->MODER &= ~((3 << (2 * 4)) | (3 << (2 * 1)));
    GPIOA->MODER |= (1 << (2 * 4)) | (1 << (2 * 1));
    GPIOA->OTYPER &= ~((1 << 4) | (1 << 1));
    GPIOA->OSPEEDR &= ~((3 << (2 * 4)) | (3 << (2 * 1)));

    // Configure PB0 - PB7 as outputs for data
    GPIOB->MODER &= ~0xFFFF;
    GPIOB->MODER |= 0x5555;
    GPIOB->OTYPER &= ~0x00FF;
    GPIOB->OSPEEDR &= ~0xFFFF;

    // LCD initialization sequence
    delay_ms(50);
    lcd_write_instruction(0x38); // Set 8-bit mode, 2-line, 5x8 font
    delay_ms(5);
    lcd_write_instruction(0x0C); // Display ON, cursor OFF
    delay_ms(5);
    lcd_write_instruction(0x06); // Entry mode set
    delay_ms(5);
    lcd_clear();                 // Clear display
}

// Send an instruction to the LCD
void lcd_write_instruction(uint8_t instruction) {
    RS_LOW();                    // Instruction mode
    E_LOW();
    GPIOB->ODR = instruction;    // Send instruction
    E_HIGH();
    delay_us(1);
    E_LOW();
    delay_ms(2);
}

// Send a character to the LCD
void lcd_write_char(uint8_t data) {
    RS_HIGH();                   // Data mode
    E_LOW();
    GPIOB->ODR = data;           // Send character
    E_HIGH();
    delay_us(1);
    E_LOW();
    delay_ms(2);
}

// Clear the LCD display
void lcd_clear(void) {
    lcd_write_instruction(0x01); // Clear command
    delay_ms(2);
}

// Set cursor position on the LCD
void lcd_goto(uint8_t row, uint8_t column) {
    delay_ms(2);
    if (row == 0) {
        lcd_write_instruction(0x80 + column); // Row 0 address
    } else {
        lcd_write_instruction(0xC0 + column); // Row 1 address
    }
}

// Write a string to the LCD
void lcd_write_string(char *s) {
    while (*s != 0) {
        lcd_write_char(*s);      // Send each character
        s++;
    }
}

// Scroll text to the left
void lcd_scroll_left(char *text) {
    lcd_clear();
    lcd_goto(0, 0);
    lcd_write_string(text);

    for (int i = 0; i < 16; i++) {
        delay_ms(300);           // Scroll speed
        lcd_write_instruction(0x18); // Shift display left
    }
}

// Create a custom character in CGRAM
void lcd_create_char(uint8_t location, uint8_t charmap[]) {
    location &= 0x7;             // Limit to 8 locations
    lcd_write_instruction(0x40 | (location << 3)); // CGRAM address

    for (int i = 0; i < 8; i++) {
        lcd_write_char(charmap[i]); // Load custom char data
    }
}

// Display a dollar symbol on the LCD
void lcd_display_dollar(void) {
    uint8_t dollar[8] = {
        0b00100,
        0b01110,
        0b10100,
        0b01110,
        0b00101,
        0b01110,
        0b00100,
        0b00000
    };

    lcd_create_char(0, dollar); // Load custom char to CGRAM
    lcd_goto(0, 0);
    lcd_write_char(0);          // Display custom char
}

void setup_LEDS(){
	//set up green LED to be toggled.
	//LD2 is PA5 the on-board GREEN LED
	RCC ->AHBENR |= RCC_AHBENR_GPIOAEN;
	GPIOA ->MODER |= GPIO_MODER_MODER5_0;
	GPIOA ->MODER &= ~GPIO_MODER_MODER5_1;
	GPIOA ->OTYPER &= ~GPIO_OTYPER_OT_5;
	GPIOA ->OSPEEDR &= ~GPIO_OSPEEDR_OSPEEDR5_0;
	GPIOA->BRR |= GPIO_BRR_BR_5;	//clear it, turn off.

	// pc8(green led)
	RCC ->AHBENR |= RCC_AHBENR_GPIOCEN;
	GPIOC ->MODER |= GPIO_MODER_MODER8_0;
	GPIOC ->MODER &= ~GPIO_MODER_MODER8_1;
	GPIOC ->OTYPER &= ~GPIO_OTYPER_OT_8;
	GPIOC ->OSPEEDR &= ~GPIO_OSPEEDR_OSPEEDR8_0;
	GPIOC->BRR |= GPIO_BRR_BR_8;	//clear it, turn off.

	//pc9(red led)
	GPIOC ->MODER |= GPIO_MODER_MODER9_0;
	GPIOC ->MODER &= ~GPIO_MODER_MODER9_1;
	GPIOC ->OTYPER &= ~GPIO_OTYPER_OT_9;
	GPIOC ->OSPEEDR &= ~GPIO_OSPEEDR_OSPEEDR9_0;
	GPIOC->BRR |= GPIO_BRR_BR_9;	//clear it, turn off.
}

void setup_timer(void){
	//TIMER:
	//enable clock
	RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
	//set prescaler to 1kHz
	TIM3->PSC=8000;
	//set reload value	999 (every sec)
	TIM3->ARR = (1000-1);
	//start
	TIM3->CR1 |= TIM_CR1_CEN; //enabble counter/start

	//set up interrupts
	TIM3->DIER |= TIM_DIER_UIE;
	NVIC_SetPriority(TIM3_IRQn, 2);
	NVIC_EnableIRQ(TIM3_IRQn);
	__enable_irq();
}

void setup_WDT(){
	// Enable write access to IWDG registers
	IWDG->KR = 0x5555;

	// Set prescaler to divide by 32 (slower timeout)
	IWDG->PR = 1;

	// Set reload value to 4095 for maximum timeout period
	IWDG->RLR = 0xFFF;
}

void led_sequence1(void) {
	SERVICE_WDG();

    // Double blink GREEN
	GPIOC->BSRR |= (1UL<<8);  // Turn on LED
	delay_ms(30);
	SERVICE_WDG();
	GPIOC->BSRR |= (1UL<<24); // Turn off LED
	delay_ms(30);
	SERVICE_WDG();
	GPIOC->BSRR |= (1UL<<8);  // Turn on LED
	delay_ms(30);
	SERVICE_WDG();
	GPIOC->BSRR |= (1UL<<24); // Turn off LED
	delay_ms(30);
	SERVICE_WDG();

	// Double blink RED
	GPIOC->BSRR |= (1UL<<9);  // Turn on LED
	delay_ms(30);
	SERVICE_WDG();
	GPIOC->BSRR |= (1UL<<25); // Turn off LED
	delay_ms(30);
	SERVICE_WDG();
	GPIOC->BSRR |= (1UL<<9);  // Turn on LED
	delay_ms(30);
	SERVICE_WDG();
	GPIOC->BSRR |= (1UL<<25); // Turn off LED

	SERVICE_WDG();
}

void led_sequence2(void) {
	SERVICE_WDG();

	// Double blink RED
	GPIOC->BSRR |= (1UL<<9);  // Turn on LED
	delay_ms(30);
	SERVICE_WDG();
	GPIOC->BSRR |= (1UL<<25); // Turn off LED
	delay_ms(30);
	SERVICE_WDG();
	GPIOC->BSRR |= (1UL<<9);  // Turn on LED
	delay_ms(30);
	SERVICE_WDG();
	GPIOC->BSRR |= (1UL<<25); // Turn off LED
	delay_ms(30);
	SERVICE_WDG();
	GPIOC->BSRR |= (1UL<<9);  // Turn on LED
	delay_ms(30);
	SERVICE_WDG();
	GPIOC->BSRR |= (1UL<<25); // Turn off LED

	SERVICE_WDG();
}

int get_distance(){
// Trigger the ultrasonic pulse
        GPIOA->ODR |= TRIG_PIN;    // Set TRIG pin high
        delay_micro(10);              // 10 µs delay for pulse
        GPIOA->ODR &= ~TRIG_PIN;   // Set TRIG pin low

        // Wait for ECHO signal to go high
        while (!(GPIOA->IDR & ECHO_PIN));

        // Start timing when ECHO signal is high
        TIM2->CNT = 0;
        TIM2->CR1 |= TIM_CR1_CEN;  // Start the timer

        // Wait for ECHO signal to go low
        while (GPIOA->IDR & ECHO_PIN);

        // Stop timer and read duration
        double duration_us = TIM2->CNT;
        TIM2->CR1 &= ~TIM_CR1_CEN;  // Stop the timer

        // Calculate distance
        double distance = (duration_us * 0.0343) / 2;

        return distance;
}

void configure_buzzer(void){
    // Configure BUZZER as output (PB9)
	RCC->AHBENR |= RCC_AHBENR_GPIOBEN;
    GPIOB->MODER |= GPIO_MODER_MODER9_0;  // PB9 as output
}

void trigger_buzzer(double distance) {
    if (distance < 5) {
        GPIOB->ODR |= BUZZER_PIN;  // Turn on buzzer
    } else {
        GPIOB->ODR &= ~BUZZER_PIN; // Turn off buzzer
    }
}

void Peri_Init(void) {
    // Enable GPIOA clock
    RCC->AHBENR |= (1 << 17);

    // Configure PA7 as output (TRIG pin)
    GPIOA->MODER |= (1 << (7 * 2));
    GPIOA->MODER &= ~(1 << ((7 * 2) + 1));  // Set PA10 as output

    // Configure PA6 as input (ECHO pin)
    GPIOA->MODER &= ~((1 << (6 * 2)) | (1 << ((6 * 2) + 1)));  // Set PA9 as input

    // Enable Timer 2 clock
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;

    // Configure Timer 2 for 1 µs resolution
    TIM2->PSC = 8 - 1;  // 8 MHz / 8 = 1 MHz (1 µs per tick)
    TIM2->ARR = 0xFFFF; // Max ARR for full timer range
}

void delay_micro(int us) {
    TIM2->CNT = 0;
    TIM2->CR1 |= TIM_CR1_CEN;  // Start Timer
    while (TIM2->CNT < us);
    TIM2->CR1 &= ~TIM_CR1_CEN; // Stop Timer
}

void send_char(char c) {
    while (!(USART2->ISR & USART_ISR_TXE));
    USART2->TDR = (0xFF & c);
}

void send_string(const char *str) {
    while (*str) {
        send_char(*str++);
    }
}

void stringify_distance(double distance, char *buffer) {
    int integer_part = (int)distance;
    int decimal_part = (int)((distance - integer_part + 0.005) * 100);

    int index = 0;
    if (integer_part == 0) {
        buffer[index++] = '0';
    } else {
        char int_buffer[10];
        int int_index = 0;
        while (integer_part > 0) {
            int_buffer[int_index++] = (integer_part % 10) + '0';
            integer_part /= 10;
        }
        for (int i = int_index - 1; i >= 0; i--) {
            buffer[index++] = int_buffer[i];
        }
    }

    buffer[index++] = '.';

    if (decimal_part < 10) {
        buffer[index++] = '0';
    }
    if (decimal_part == 0) {
        buffer[index++] = '0';
    } else {
        char dec_buffer[3];
        int dec_index = 0;
        while (decimal_part > 0) {
            dec_buffer[dec_index++] = (decimal_part % 10) + '0';
            decimal_part /= 10;
        }
        for (int i = dec_index - 1; i >= 0; i--) {
            buffer[index++] = dec_buffer[i];
        }
    }
    buffer[index] = '\0';
}

void configure_UART() {
    // Set up PA2 as UART in TX mode
    RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
    GPIOA->MODER &= ~GPIO_MODER_MODER2_0;
    GPIOA->MODER |= GPIO_MODER_MODER2_1;
    GPIOA->AFR[0] &= ~GPIO_AFRL_AFSEL2_Msk;
    GPIOA->AFR[0] |= (1 << GPIO_AFRL_AFSEL2_Pos);

    // UART configuration for 9600 baud, 8N1
    RCC->APB1ENR |= RCC_APB1ENR_USART2EN;
    USART2->BRR = 0x341;
    USART2->CR1 &= ~USART_CR1_M;
    USART2->CR1 &= ~USART_CR1_PCE;
    USART2->CR2 &= ~USART_CR2_STOP;
    USART2->CR1 |= USART_CR1_TE;
    USART2->CR1 |= USART_CR1_UE;
}


void ADC_Init(void) {
    // Enable ADC1 and GPIOA clocks
    RCC->AHBENR |= RCC_AHBENR_GPIOAEN;       // Enable GPIOA clock
    RCC->APB2ENR |= RCC_APB2ENR_ADCEN;       // Enable ADC clock

    // Configure PA0 in analog mode
    GPIOA->MODER |= GPIO_MODER_MODER0;       // Set PA0 to analog mode (11)

    // Calibrate ADC before enabling
    if ((ADC1->CR & ADC_CR_ADEN) != 0) {     // Ensure ADC is disabled before calibration
        ADC1->CR &= ~ADC_CR_ADEN;
    }
    ADC1->CR |= ADC_CR_ADCAL;                // Start calibration
    while (ADC1->CR & ADC_CR_ADCAL);         // Wait for calibration to complete

    // Set ADC sampling time (e.g., 239.5 cycles for better accuracy)
    ADC1->SMPR |= ADC_SMPR_SMP_2 | ADC_SMPR_SMP_1 | ADC_SMPR_SMP_0; // 239.5 cycles

    // Set single conversion mode
    ADC1->CHSELR = ADC_CHSELR_CHSEL0;        // Select channel 0 (PA0)

    // Enable ADC
    ADC1->CR |= ADC_CR_ADEN;                 // Enable ADC
    while (!(ADC1->ISR & ADC_ISR_ADRDY));    // Wait for ADC to be ready
}

void PWM_Init(void) {
    // Enable TIM2 and GPIOB clocks
    RCC->AHBENR |= RCC_AHBENR_GPIOBEN;       // Enable GPIOB clock
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;      // Enable TIM2 clock

    // Configure PB10 in alternate function mode for TIM2_CH3
    GPIOB->MODER &= ~GPIO_MODER_MODER10_0;   // Clear bit 0 for PB10
    GPIOB->MODER |= GPIO_MODER_MODER10_1;    // Set bit 1 for alternate function
    GPIOB->AFR[1] |= (2 << GPIO_AFRH_AFRH2_Pos);  // AF2 for TIM2_CH3 on PB10

    // Configure Timer2 for PWM
    TIM2->PSC = 8-1;                      // Prescaler for 1 MHz timer clock (8 MHz system clock)
    TIM2->ARR = 999;                     // Auto-reload for 1 kHz PWM frequency

    // Configure PWM mode for Channel 3
    TIM2->CCMR2 &= ~TIM_CCMR2_OC3M;          // Clear output compare mode bits for Channel 3
    TIM2->CCMR2 |= (TIM_CCMR2_OC3M_1 | TIM_CCMR2_OC3M_2);  // PWM mode 1 (active high)
    TIM2->CCMR2 |= TIM_CCMR2_OC3PE;          // Enable preload register for Channel 3

    // Enable output compare for Channel 3
    TIM2->CCER |= TIM_CCER_CC3E;             // Enable channel 3 output

    // Enable counter
    TIM2->CR1 |= TIM_CR1_CEN;                // Enable timer
}

uint16_t Read_ADC(void) {
    ADC1->CR |= ADC_CR_ADSTART;              // Start conversion

    while (!(ADC1->ISR & ADC_ISR_EOC));      // Wait for conversion to complete

    return ADC1->DR;                         // Return conversion result
}

void Set_LED_Brightness(uint16_t adc_value) {
    // Scale ADC value (0-4095) to PWM value (0-999)
    uint32_t pwm_value = (adc_value * 999) / 4095;
    TIM2->CCR3 = pwm_value;                  // Set duty cycle for Channel 3 (PB10)
}
