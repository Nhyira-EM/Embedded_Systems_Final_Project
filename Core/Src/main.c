//main.c
#define scoreboard

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "main.h"

#ifdef scoreboard
#include "team_a.h"
extern void timerHandler(void);

// Definitions for task priorities
#define KEYPAD_TASK_PRIORITY    (tskIDLE_PRIORITY + 3)
#define DISPLAY_TASK_PRIORITY   (tskIDLE_PRIORITY + 1)
#define DISTANCE_TASK_PRIORITY  (tskIDLE_PRIORITY + 2)
#define LED_TASK_PRIORITY       (tskIDLE_PRIORITY + 1)
#define BUZZER_TASK_PRIORITY    (tskIDLE_PRIORITY + 2)

// Task handles
static TaskHandle_t keypadTaskHandle = NULL;
static TaskHandle_t displayTaskHandle = NULL;
static TaskHandle_t distanceTaskHandle = NULL;
static TaskHandle_t ledTaskHandle = NULL;
static TaskHandle_t buzzerTaskHandle = NULL;

// Queue handle for score updates and distance data
static QueueHandle_t scoreQueue;
static QueueHandle_t distanceQueue; // Queue for distance data

// Mutex for LCD access
static SemaphoreHandle_t lcdMutex;

// Mutex for buzzer access
static SemaphoreHandle_t buzzerMutex;

// Task to handle LED brightness control
void ledBrightnessTask(void *pvParameters) {
    while(1) {
        uint16_t adc_val = Read_ADC();
//        send_string("ADC Value: ");
//        char buffer[10];
//        stringify_distance(adc_val, buffer);
//        send_string(buffer);
//        send_string("\r\n");
//        vTaskDelay(pdMS_TO_TICKS(1000));
        Set_LED_Brightness(adc_val);
        vTaskDelay(pdMS_TO_TICKS(50));  // Update every 50ms
    }
}

// Structure to hold score updates
typedef struct {
    char score;
    char action; // '1' for increment, '3' for decrement, '5' for reset
} ScoreUpdate_t;


// Task to handle keypad input
void keypadTask(void *pvParameters) {
    char lastKey = 'z';  // Initialize with an invalid key
    char currentKey;

    while(1) {
        currentKey = keypad_get_key();

        // Only process key if it's different from last key and not 'z'
        if(currentKey != 'z' && currentKey != lastKey) {
            ScoreUpdate_t update;
            update.action = currentKey;

            switch(currentKey) {
                case '1':
                case '3':
                case '5':
                    xQueueSend(scoreQueue, &update, portMAX_DELAY);
                    break;
                default:
                    xSemaphoreTake(lcdMutex, portMAX_DELAY);
                    lcd_clear();
                    lcd_goto(0, 0);
                    lcd_write_string("invalid");
                    led_sequence2();
                    xSemaphoreGive(lcdMutex);
                    break;
            }
        }

        lastKey = currentKey;  // Update last key
        vTaskDelay(pdMS_TO_TICKS(50)); // Sampling delay
    }
}

// Task to handle display updates
void displayTask(void *pvParameters) {
    char currentScore = '0';
    ScoreUpdate_t update;

    // Initial display setup
    xSemaphoreTake(lcdMutex, portMAX_DELAY);
    lcd_goto(0, 0);
    lcd_write_string("Team A");
    lcd_goto(1, 0);
    lcd_write_string("Score: 0");
    xSemaphoreGive(lcdMutex);

    while(1) {
        if(xQueueReceive(scoreQueue, &update, portMAX_DELAY) == pdTRUE) {
            switch(update.action) {
                case '1':
                    currentScore += 1;
                    break;
                case '3':
                    if(currentScore > '0') {
                        currentScore -= 1;
                    }
                    break;
                case '5':
                    currentScore = '0';
                    break;
            }

            xSemaphoreTake(lcdMutex, portMAX_DELAY);
            lcd_goto(0, 0);
            lcd_write_string("Team A ");
            lcd_goto(1, 0);
            lcd_write_string("Score: ");
            lcd_goto(1, 7);
            lcd_write_char(currentScore);
            xSemaphoreGive(lcdMutex);

            led_sequence1();
        }
    }
}

// Task to handle distance measurements
void distanceTask(void *pvParameters) {
    char buffer[50];
    while(1) {
        double distance = get_distance();
        xQueueSend(distanceQueue, &distance, portMAX_DELAY); // Send to queue
        send_string("The distance is ");
        stringify_distance(distance, buffer);
        send_string(buffer);
        send_string(" cm\r\n");
        vTaskDelay(pdMS_TO_TICKS(100)); // Adjust measurement frequency as needed
    }
}

void buzzerTask(void *pvParameters) {
    double distance;
    while(1) {
        if (xQueueReceive(distanceQueue, &distance, portMAX_DELAY)) {
            // Take the buzzer mutex before accessing the buzzer
            xSemaphoreTake(buzzerMutex, portMAX_DELAY);

            // Call your trigger_buzzer function
            trigger_buzzer(distance);

            // Release the buzzer mutex after the operation
            xSemaphoreGive(buzzerMutex);
        }
    }
}

int main(void) {
    // Initialize peripherals
    lcd_init();
    lcd_clear();
    keypad_init();
    setup_LEDS();
    setup_timer();
    setup_WDT();
    Peri_Init();
    configure_UART();
    ADC_Init();
    PWM_Init();

    // Create FreeRTOS synchronization primitives
    scoreQueue = xQueueCreate(5, sizeof(ScoreUpdate_t));
    distanceQueue = xQueueCreate(5, sizeof(double)); // Create the distance queue
    lcdMutex = xSemaphoreCreateMutex();
    buzzerMutex = xSemaphoreCreateMutex();

    if (scoreQueue == NULL || distanceQueue == NULL || lcdMutex == NULL) {
        // Handle resource creation failure
        while(1);
    }

    // Create tasks
    xTaskCreate(keypadTask,
                "Keypad",
                configMINIMAL_STACK_SIZE,
                NULL,
                KEYPAD_TASK_PRIORITY,
                &keypadTaskHandle);

    xTaskCreate(displayTask,
                "Display",
                configMINIMAL_STACK_SIZE,
                NULL,
                DISPLAY_TASK_PRIORITY,
                &displayTaskHandle);

    xTaskCreate(distanceTask,
                "Distance",
                configMINIMAL_STACK_SIZE,
                NULL,
                DISTANCE_TASK_PRIORITY,
                &distanceTaskHandle);

    xTaskCreate(ledBrightnessTask,
                "LED",
                configMINIMAL_STACK_SIZE,
                NULL,
                LED_TASK_PRIORITY,
                &ledTaskHandle);

    xTaskCreate(buzzerTask,
                "Buzzer",
                configMINIMAL_STACK_SIZE,
                NULL,
                BUZZER_TASK_PRIORITY,
                &buzzerTaskHandle);

    // Start the scheduler
    vTaskStartScheduler();

    // Should never get here
    while(1);
}

void TIM3_IRQHandler(void) {
    timerHandler();
}

#endif
