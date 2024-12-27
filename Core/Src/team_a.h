//team_a.h

#ifndef SRC_TEAM_A_H_
#define SRC_TEAM_A_H_

#include "stm32f0xx.h"

#define ROW_PORT GPIOC
#define COL_PORT GPIOC
#define ROW_PINS 0x0F  // PC0 - PC3
#define COL_PINS 0xF0  // PC4 - PC7

#define RS_PIN          4
#define E_PIN           1
#define DATA_PORT       GPIOB
#define CONTROL_PORT    GPIOA

#define SERVICE_WDG()	IWDG->KR = 0xAAAA

#define TRIG_PIN (1 << 7) // PA7
#define ECHO_PIN (1 << 6)  // PA6
#define BUZZER_PIN (1 << 9)

#define RS_HIGH()       (CONTROL_PORT->BSRR = (1U << RS_PIN))
#define RS_LOW()        (CONTROL_PORT->BSRR = (1U << (RS_PIN + 16)))
#define E_HIGH()        (CONTROL_PORT->BSRR = (1U << E_PIN))
#define E_LOW()         (CONTROL_PORT->BSRR = (1U << (E_PIN + 16)))

void ADC_Init(void);
void PWM_Init(void);
uint16_t Read_ADC(void);
void Set_LED_Brightness(uint16_t adc_value);

void keypad_init(void);
char keypad_get_key(void);
void delay_ms(int ms);
void delay_us(int us);
void lcd_init(void);
void lcd_write_instruction(uint8_t instruction);
void lcd_write_char(uint8_t data);
void lcd_clear(void);
void lcd_goto(uint8_t row, uint8_t column);
void lcd_write_string(char *s);
void lcd_scroll_left(char *text);
void lcd_create_char(uint8_t location, uint8_t charmap[]);
void lcd_display_dollar(void);
void setup_LEDS(void);
void setup_timer(void);
void setup_WDT(void);
void led_sequence1(void);
void led_sequence2(void);
int get_distance(void);
void Peri_Init(void);
void delay_micro(int us);
void stringify_distance(double distance, char *buffer);
void send_char(char c);
void send_string(const char *str);
void configure_UART(void);
void configure_buzzer(void);
void trigger_buzzer(double);

#endif /* SRC_TEAM_A_H_ */
