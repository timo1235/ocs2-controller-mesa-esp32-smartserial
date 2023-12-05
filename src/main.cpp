#include <includes.h>

RegisterOut outRegister(REGISTER_OUT_DATA, REGISTER_OUT_CLK, REGISTER_OUT_LATCH);
RegisterIn  inRegister(REGISTER_IN_DATA, REGISTER_IN_CLK, REGISTER_IN_LATCH, REGISTER_IN_LOAD);
ADCManager  adcManager;
Debug       debug;
bool        sserial_timeoutFlag = true;

hw_timer_t  *timer          = NULL;
uint64_t     timerCounter   = 0;
TaskHandle_t mainTaskHandle = NULL;

#include "driver/uart.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include <stdio.h>
#include <string.h>

static const char *TAG = "uart_events";

#define EX_UART_NUM     UART_NUM_0
#define PATTERN_CHR_NUM (3) /*!< Set the number of consecutive and identical characters received by receiver which defines a UART pattern*/

#define BUF_SIZE    (1024)
#define RD_BUF_SIZE (BUF_SIZE)
static QueueHandle_t uart0_queue;

void IRAM_ATTR onTimer(void *arg) { vTaskResume(mainTaskHandle); }

void setupTimer(void *pvParameters) {
    for (;;) {
        timerCounter++;
        // Print Core
        // Serial.printf("Core: %d\n", xPortGetCoreID());
        // Serial.printf("Counter: %d\n", timerCounter);

        vTaskSuspend(NULL);
        // vTaskDelay(1);   // Verhindern, dass der Task beendet wird
    }
}

void setup() {
    // pinMode(I2C_OCS2_SDA, INPUT);
    // pinMode(I2C_OCS2_SCL, INPUT);
    // Serial for debugging - too much output interferes with the smart serial communication
    // since it needs to be very fast and reliable. So, use debug output wisely.
    Serial.begin(115200);

    // Start serial instance for smart serial communication
    // Baudrate is 2.5MBit/s
    // Serial1.begin(2500000, SERIAL_8N1, SSERIAL_RXD, SSERIAL_TXD);

    // debug.init();

    // inRegister.init();
    // outRegister.init();
    // adcManager.init();

    // outRegister.set(RegisterOut::Output::ena, true);

    delay(2000);

    // sserial_init();

    // Space for testing

    // Erstellen Sie einen Task auf Kern 1, um den Timer einzurichten
    xTaskCreatePinnedToCore(setupTimer,         // Funktion, die ausgeführt werden soll
                            "TimerSetupTask",   // Name des Tasks
                            10000,              // Stackgröße des Tasks
                            NULL,               // Parameter für den Task
                            1,                  // Priorität des Tasks
                            &mainTaskHandle,    // Task-Handle
                            1                   // Kern, auf dem der Task ausgeführt wird (1 für Kern 1)
    );
    // Timer konfigurieren und starten
    esp_timer_create_args_t timer_args;
    timer_args.callback = &onTimer;
    timer_args.arg      = NULL;   // Keine Argumente benötigt
    timer_args.name     = "blink_timer";

    esp_timer_handle_t timer_handle;
    esp_timer_create(&timer_args, &timer_handle);
    esp_timer_start_periodic(timer_handle, 500);

    /* Configure parameters of an UART driver,
     * communication pins and install the driver */
    uart_config_t uart_config = {
        .baud_rate  = 115200,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };
    // Install UART driver, and get the queue.
    uart_driver_install(EX_UART_NUM, BUF_SIZE * 2, BUF_SIZE * 2, 20, &uart0_queue, 0);
    uart_param_config(EX_UART_NUM, &uart_config);
}

void loop() {
    // print counters in one line
    // Serial.printf("Task 1: %d, Task 2: %d\n", task1Counter, task2Counter);
    // delay(1000);

    // Print the value of counter every 1000ms
    Serial.printf("Counter: %d\n", timerCounter);
    delay(1000);
}
